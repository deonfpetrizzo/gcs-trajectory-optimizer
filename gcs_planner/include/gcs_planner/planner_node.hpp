#ifndef GCS_PLANNER__PLANNER_NODE_HPP_
#define GCS_PLANNER__PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <planner_msgs/msg/trajectory.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "gcs_pipeline.hpp"
#include "corridor_builder.hpp"
#include "region_viz.hpp"

#include <Eigen/Dense>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace gcs_planner {

using Path = std::vector<Eigen::VectorXd>;

// A contiguous run of waypoints sharing one seg_idx/tag from the nominal path
// CSV (e.g. "GROUND", "TRANS", "AERIAL"). Corridor generation and trajectory
// optimization run independently per segment.
struct PathSegment {
    std::string tag;
    Path waypoints;
};

// One entry of nominal_path_csv_files (or the sole entry when falling back to
// the flat nominal_path param): its own nominal path, segments, publishers,
// and cached "latest" output. Each vehicle's full pipeline (corridor growth +
// trajectory optimization, over all its segments) runs independently of the
// others -- in its own thread, on its own topics, with trajectory timing
// starting at zero -- so that e.g. m4, g1, and g1+m4 get separate
// trajectories instead of being concatenated into one.
struct VehiclePlan {
    std::string vehicle;   // key derived from the CSV filename, e.g. "m4"; "" in flat-param fallback
    Path nominal_path;
    std::vector<PathSegment> segments;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nominal_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_segments_pub;
    rclcpp::Publisher<planner_msgs::msg::Trajectory>::SharedPtr traj_pub;

    // Cached "latest" state, guarded by PlannerNode::state_mtx_: written by
    // run_pipeline_for_vehicle() (on its own worker thread) and read/republished
    // by publish_all() (on the executor thread, via republish_timer_).
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
    bool load_pcd(const std::string& file);
    // Spawns one thread per VehiclePlan (run_pipeline_for_vehicle()) and
    // joins them, so all vehicles' corridors/trajectories are grown/optimized
    // in parallel.
    void run_pipeline();
    // Grows the corridor and optimizes the trajectory for every segment of
    // one vehicle, then caches and publishes that vehicle's own topics.
    // Runs on a dedicated worker thread spawned by run_pipeline(); touches
    // only its own VehiclePlan and the (read-only, shared) cloud matrix.
    void run_pipeline_for_vehicle(VehiclePlan& vp, const Eigen::MatrixXd& P);
    // Replaces each TRANS segment sandwiched between a GROUND segment and an
    // AERIAL segment with a straight vertical column of height
    // takeoff_landing_height_ (ground point <-> ground point + height in z),
    // and extends the neighboring AERIAL segment so it starts/ends at the top
    // of that column instead of the CSV-authored diagonal transition. Run
    // once per vehicle right after its segments are parsed (see constructor);
    // run_pipeline_for_vehicle() detects a resegmented TRANS segment (exactly
    // 2 waypoints) and gives it a plain vertical ease trajectory instead of
    // the usual corridor-growth + optimization path. TRANS segments not
    // bounded by GROUND/AERIAL neighbors are left as authored.
    void resegment_transitions(VehiclePlan& vp) const;
    // Writes one vehicle's exported trajectory to
    // "<traj_output_dir_>/traj_<vehicle>.csv" with columns
    // seg_idx,tag,x,y,z,qx,qy,qz,qw (mirroring the input path CSV format,
    // plus orientation). Segment boundaries within the flat, concatenated
    // Trajectory message are detected the same way trajectory_executor_node
    // does: to_msg() always samples the exact rest-to-rest endpoint of each
    // segment, so two consecutive points sharing the same time_from_start
    // mark a boundary; `seg_tags` gives each detected segment's tag in
    // order. Orientation is built via tf2::Quaternion::setRPY() directly
    // from each point's roll/pitch/yaw fields (roll/pitch derived in
    // to_msg() -- see attitude_from_accel_yaw() in the .cpp).
    void write_trajectory_csv(const planner_msgs::msg::Trajectory& traj,
                              const std::vector<std::string>& seg_tags,
                              const std::string& file_path) const;
    // Publish the latest cached cloud/voxels/nominal-path/corridor/trajectory
    // messages for every vehicle. Called once per vehicle right after its
    // run_pipeline_for_vehicle() finishes and then on a timer, so a display
    // re-checked in RViz (which may have missed the transient_local latch,
    // or just want a fresh look) gets it again within one republish period
    // instead of only once at startup.
    void publish_all();

    // conversions / helpers
    Eigen::MatrixXd cloud_to_eigen(const sensor_msgs::msg::PointCloud2& msg) const;
    // Load x,y,z waypoints from one or more CSV files, grouped into segments
    // by contiguous seg_idx (header must include "x","y","z"; "seg_idx" and
    // "tag" are optional -- absent means one untagged segment). Segments are
    // appended across files in list order.
    std::vector<PathSegment> load_path_segments_csv(const std::vector<std::string>& files) const;

    // message builders
    planner_msgs::msg::Trajectory
    to_msg(const std::vector<gcs::CompositeTimingTrajectory>& trajs,
          const std::vector<std::string>& tags,
          const std_msgs::msg::Header& hdr) const;
    nav_msgs::msg::Path make_path_msg(const Path& pts) const;
    nav_msgs::msg::Path make_traj_path(const std::vector<gcs::CompositeTimingTrajectory>& trajs,
                                       const std_msgs::msg::Header& hdr) const;
    visualization_msgs::msg::MarkerArray
    make_corridor_markers(const std::vector<gcs::ConvexRegion>& regions,
                         const std::vector<std::string>& tags) const;
    visualization_msgs::msg::MarkerArray
    make_traj_segment_markers(const std::vector<gcs::CompositeTimingTrajectory>& trajs,
                              const std::vector<std::string>& tags,
                              const std_msgs::msg::Header& hdr) const;

    // params
    double R_, v_max_, sample_dt_;
    double pcd_yaw_deg_;
    double ground_band_half_width_, ground_plane_fit_radius_, ground_plane_z_window_;
    double takeoff_landing_height_;
    double takeoff_landing_accel_limit_;
    double republish_period_sec_;
    double w_geom_accel_, w_time_accel_, w_geom_jerk_;
    double w_time_, w_geom_length_;
    int    degree_, continuity_;
    int    num_threads_, max_rounds_;
    int64_t corridor_seed_;
    bool drop_covered_samples_;
    std::string frame_id_, pcd_file_, traj_output_dir_;
    std::vector<VehiclePlan> vehicles_;

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxels_pub_;
    rclcpp::TimerBase::SharedPtr republish_timer_;

    // Cached "latest" state, guarded by state_mtx_: written by
    // run_pipeline_for_vehicle() (on its own worker thread, one per vehicle)
    // and read/republished by publish_all() (on the executor thread, both
    // directly and via republish_timer_). Per-vehicle cached output lives in
    // each VehiclePlan (see vehicles_ above); this mutex guards all of them.
    std::mutex state_mtx_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
    visualization_msgs::msg::MarkerArray last_voxels_;
};

}  // namespace gcs_planner
#endif  // GCS_PLANNER__PLANNER_NODE_HPP_