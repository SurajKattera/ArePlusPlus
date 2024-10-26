#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "TaskPlanner.h"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskPlanner>();

    std::vector<std::pair<int, int>> initial_tasks;
    bool do_auto_load_orders = true;
    bool do_auto_load_locations = true;

    if (do_auto_load_orders) {
        initial_tasks.push_back(std::pair<int, int>(3, 1));
        initial_tasks.push_back(std::pair<int, int>(2, 3));
        initial_tasks.push_back(std::pair<int, int>(3, 2));
    }

    // Update the TaskPlanner with initial tasks
    node = std::make_shared<TaskPlanner>(initial_tasks);

    if (do_auto_load_locations) {
        //node->set_activation_state(true);
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
