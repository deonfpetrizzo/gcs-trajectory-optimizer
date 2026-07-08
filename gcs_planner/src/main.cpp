#include "gcs_planner/planner_node.hpp"
#include <thread>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor exec;
    auto node = std::make_shared<gcs_planner::PlannerNode>();
    exec.add_node(node);

    std::thread pipeline([node]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        node->startup();
    });

    exec.spin();
    pipeline.join();
    rclcpp::shutdown();
    return 0;
}
