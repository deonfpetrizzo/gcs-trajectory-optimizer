#include "gcs_planner/trajectory_executor_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<gcs_planner::TrajectoryExecutorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
