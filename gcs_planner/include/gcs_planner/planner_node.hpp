#ifndef GCS_PLANNER__PLANNER_NODE_HPP_
#define GCS_PLANNER__PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <planner_msgs/msg/trajectory.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "gcs_core/gcs_core.hpp"
#include "gcs_core/corridor_builder.hpp"
#include "gcs_core/region_viz.hpp"

#include "gcs_planner/path_io.hpp"
#include "traversability_core/traversability_core.hpp"

#include <Eigen/Dense>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace gcs_planner {

/**
 * @brief One entry of nominal_path_csv_files (or the sole entry when falling back to
 * the flat nominal_path param): its own nominal path, segments, publishers, and
 * cached "latest" output. Each vehicle's full pipeline (corridor growth + trajectory
 * optimization, over all its segments) runs independently of the others -- in its own
 * thread, on its own topics, with trajectory timing starting at zero -- so that e.g.
 * m4, g1, and g1+m4 get separate trajectories instead of being concatenated into one.
 */
struct VehiclePlan {
    std::string vehicle; 
    Path nominal_path;
    std::vector<PathSegment> segments;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nominal_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_segments_pub;
    rclcpp::Publisher<planner_msgs::msg::Trajectory>::SharedPtr traj_pub;

    visualization_msgs::msg::MarkerArray last_corridor_markers;
    nav_msgs::msg::Path last_traj_path;
    visualization_msgs::msg::MarkerArray last_traj_segment_markers;
    planner_msgs::msg::Trajectory last_traj_msg;
    bool has_trajectory = false;
};

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();
    void startup();

private:
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief Loads and downsamples a PCD map file, then publishes it.
     * @param file Path to the PCD file.
     * @return True on success.
     */
    bool load_pcd(const std::string& file);

    /**
     * @brief Spawns one thread per VehiclePlan (run_pipeline_for_vehicle()) and joins
     * them, so all vehicles' corridors/trajectories are grown/optimized in parallel.
     */
    void run_pipeline();

    /**
     * @brief Grows the corridor and optimizes the trajectory for every segment of one
     * vehicle, then caches and publishes that vehicle's own topics.
     * @remark Runs on a dedicated worker thread spawned by run_pipeline(); touches
     * only its own VehiclePlan and the read-only shared cloud matrix.
     * @param vp The vehicle to plan for.
     * @param P Obstacle point cloud (rows = points).
     */
    void run_pipeline_for_vehicle(VehiclePlan& vp, const Eigen::MatrixXd& P);

    /**
     * @brief Computes the (static) traversability map + 2D occupancy grid from the
     * obstacle cloud, caches occupied_xy_ / occ_grid_, and publishes the two
     * visualizations. Called once per pipeline run before the vehicle threads spawn;
     * the result is shared read-only by every vehicle's GROUND segments.
     * @param P Obstacle point cloud (rows = points).
     */
    void compute_traversability_map(const Eigen::MatrixXd& P);

    /**
     * @brief Builds a trav::RobotModel from the robot_* parameters (deg -> rad).
     * @return The robot model.
     */
    trav::RobotModel robot_model() const;

    /**
     * @brief Looks up the traversability map's locally-fit ground plane at a world (x, y).
     * @remark Reads trav_map_'s smoothed plane coefficients so the ground corridor and
     * endpoint snapping latch to one globally-consistent terrain field.
     * @param x World x.
     * @param y World y.
     * @param out Filled with the cell's ground plane when it is valid.
     * @return True if a valid map cell covers (x, y); false otherwise.
     */
    bool map_ground_plane(double x, double y, gcs::GroundPlaneFit& out) const;

    /**
     * @brief Snaps a point onto free, traversable ground.
     * @remark Moves xy to the nearest free occupancy cell and sets z from the map's
     * ground plane there (falling back to a raw-cloud fit where the map is invalid).
     * @param q Query point (3D).
     * @param P Obstacle point cloud (rows = points), for the fallback plane fit.
     * @return The snapped 3D point.
     */
    Eigen::VectorXd snap_to_ground(const Eigen::VectorXd& q, const Eigen::MatrixXd& P) const;

    /**
     * @brief Plans one segment: grows its corridor and produces its trajectory.
     * @remark A resegmented TRANS segment (2 waypoints) gets a plain vertical ease
     * profile (make_vertical_trajectory()); all others go through corridor growth
     * (parallel sampling + disjoint-set-union connectivity, build_corridor_parallel())
     * followed by QP optimization (GCSCompositeBezierPlanner). q0/qT are the resolved
     * segment-boundary endpoints (shared with the neighboring segments for continuity),
     * not the segment's raw nominal waypoints.
     * @param vp The vehicle the segment belongs to.
     * @param si Segment index (for logging).
     * @param seg The segment to plan.
     * @param P Obstacle point cloud (rows = points).
     * @param q0 Start endpoint the trajectory must begin at.
     * @param qT Goal endpoint the trajectory must end at.
     * @param all_regions Regions used by the trajectory are appended here for
     * visualization: the Dijkstra-routed subsequence for composite (GROUND/AERIAL)
     * segments, or the vertical-hop seed regions for a TRANS segment.
     * @param region_tags Each appended region's tag is appended here, aligned with all_regions.
     * @return The segment's trajectory, or nullopt if it could not be planned (no
     * regions grown, an uncontained vertical hop, or optimizer failure).
     */
    std::optional<gcs::CompositeTimingTrajectory> plan_segment(
        const VehiclePlan& vp, size_t si, const PathSegment& seg, const Eigen::MatrixXd& P,
        const Eigen::VectorXd& q0, const Eigen::VectorXd& qT,
        std::vector<gcs::ConvexRegion>& all_regions,
        std::vector<std::string>& region_tags) const;

    /**
     * @brief Builds corridor/trajectory messages for one vehicle, caches them, exports
     * the trajectory CSV if traj_output_dir_ is set, and publishes on the vehicle's topics.
     * @param vp The vehicle to publish for.
     * @param all_regions The trajectory-constraining regions across the vehicle's
     * segments (routed subsequence per composite segment; seed regions per TRANS).
     * @param region_tags Each region's tag, aligned with all_regions.
     * @param seg_trajs Each planned segment's trajectory.
     * @param traj_tags Each trajectory's tag, aligned with seg_trajs.
     */
    void publish_vehicle_output(VehiclePlan& vp,
        const std::vector<gcs::ConvexRegion>& all_regions,
        const std::vector<std::string>& region_tags,
        const std::vector<gcs::CompositeTimingTrajectory>& seg_trajs,
        const std::vector<std::string>& traj_tags);

    /**
     * @brief Replaces each TRANS segment sandwiched between a GROUND segment and an
     * AERIAL segment with a straight vertical column of height
     * takeoff_landing_height_, and extends the neighboring AERIAL segment so it
     * starts/ends at the top of that column instead of the CSV-authored diagonal
     * transition.
     * @remark Run once per vehicle right after its segments are parsed (see
     * constructor). TRANS segments not bounded by GROUND/AERIAL neighbors are left as authored.
     * @param vp The vehicle whose segments to resegment.
     */
    void resegment_transitions(VehiclePlan& vp) const;

    /**
     * @brief Publishes the latest cached cloud/voxels/nominal-path/corridor/trajectory
     * messages for every vehicle.
     * @remark Called once per vehicle right after its run_pipeline_for_vehicle()
     * finishes, and then on a timer, so an RViz display that missed the
     * transient_local latch reappears within one republish period instead of only once at startup.
     */
    void publish_all();

    /**
     * @brief Converts a PointCloud2 message to an Eigen point matrix.
     * @param msg The point cloud message.
     * @return One point per row; non-finite points are dropped.
     */
    Eigen::MatrixXd cloud_to_eigen(const sensor_msgs::msg::PointCloud2& msg) const;

    /**
     * @brief Builds a header stamped now() in frame_id_.
     * @return The header, used for every outgoing message.
     */
    std_msgs::msg::Header make_header() const;

    // params
    double R_, v_max_, sample_dt_;
    double pcd_yaw_deg_;
    double cloud_voxel_leaf_;
    double ground_plane_fit_radius_, ground_plane_z_window_;
    double takeoff_landing_height_;
    double takeoff_landing_accel_limit_;
    double republish_period_sec_;
    double w_geom_accel_, w_time_accel_, w_geom_jerk_;
    double w_time_, w_geom_length_;
    double geom_accel_limit_, time_accel_limit_;
    double t_min_, t_max_, h_dot_min_;
    int    soc_facets_;
    bool   rest_to_rest_accel_;
    int    degree_, continuity_;
    int    num_threads_, max_rounds_;
    int64_t corridor_seed_;
    bool drop_covered_samples_;
    std::string frame_id_, pcd_file_, traj_output_dir_;

    // traversability / ground-corridor params
    double trav_resolution_, trav_slope_fit_radius_, trav_ground_z_window_;
    double trav_clearance_ceiling_, trav_smooth_sigma_, ground_grid_margin_;
    double ground_floor_offset_;
    int    trav_min_points_;
    double robot_max_slope_deg_, robot_max_roughness_, robot_max_step_;
    double robot_height_, robot_min_clearance_, robot_footprint_radius_;
    bool   robot_allow_stairs_;
    double occ_score_threshold_;

    std::vector<VehiclePlan> vehicles_;

    // traversability state (static map; shared read-only across vehicles once computed)
    trav::TraversabilityMap trav_map_;
    trav::OccupancyGrid2D occ_grid_;
    Eigen::MatrixXd occupied_xy_;
    bool has_trav_ = false;

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxels_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traversability_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::TimerBase::SharedPtr republish_timer_;

    std::mutex state_mtx_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
    visualization_msgs::msg::MarkerArray last_voxels_;
    visualization_msgs::msg::MarkerArray last_trav_markers_;
    nav_msgs::msg::OccupancyGrid last_occ_grid_;
};

}  // namespace gcs_planner
#endif  // GCS_PLANNER__PLANNER_NODE_HPP_
