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
#include <mutex>
#include <string>
#include <vector>

namespace gcs_planner {

using Path = std::vector<Eigen::VectorXd>;

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode();
    void startup();

private:
    void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    bool load_pcd(const std::string& file);
    void run_pipeline();                       // grow corridor, optimize, publish

    // conversions / helpers
    Eigen::MatrixXd cloud_to_eigen(const sensor_msgs::msg::PointCloud2& msg) const;

    // message builders
    planner_msgs::msg::Trajectory
    to_msg(const gcs::CompositeTimingTrajectory& tr, const std_msgs::msg::Header& hdr) const;
    nav_msgs::msg::Path make_path_msg(const Path& pts) const;
    nav_msgs::msg::Path make_traj_path(const gcs::CompositeTimingTrajectory& tr,
                                       const std_msgs::msg::Header& hdr) const;
    visualization_msgs::msg::MarkerArray
    make_corridor_markers(const std::vector<gcs::ConvexRegion>& regions) const;

    // params
    double R_, v_max_, max_step_, sample_dt_;
    int    degree_, continuity_;
    int    num_threads_, max_rounds_;
    int64_t corridor_seed_;
    std::string frame_id_, pcd_file_;
    Path nominal_path_;

    // ROS handles
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxels_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             nominal_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr corridor_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr             traj_path_pub_;
    rclcpp::Publisher<planner_msgs::msg::Trajectory>::SharedPtr   traj_pub_;

    std::mutex cloud_mtx_;
    sensor_msgs::msg::PointCloud2::SharedPtr last_cloud_;
    visualization_msgs::msg::MarkerArray last_voxels_;
};

}  // namespace gcs_planner
#endif  // GCS_PLANNER__PLANNER_NODE_HPP_