#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <unordered_map>
#include <atomic>
#include <queue>
#include "Structures.h"
#include "moving_node.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h> 


class TaskPlanner : public rclcpp::Node {
public:
    TaskPlanner(std::vector<std::pair<int, int>> initial_tasks);
    TaskPlanner();

    void set_activation_state(bool state);
     

    

private:
    void timer_callback(); // This one gets to do the main logic
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // These need to be implemented & doxygened
    bool is_at_target(const Pose2d &target);       // < Thish
    void manual_go_to_point(const Pose2d &target); // < Thish
    void nav2_go_to_point(const Pose2d &target);   // < Jack
    void prep_next_order();                        // < Adi
    bool load_locations_from_file();               // < Dinh, lmk if you want a hand here.
    bool get_visible_station_code(int &tag_id);    // < Dinh

    std::vector<NavNode> generatePathToStation(const Pose2d &destination);

    // Other todo:
    // E-stop services
    // Doxygen
    // Status outputs, with a silence flag

    // Member variables
    std::queue<Order> pending_orders_;
    std::queue<NavNode> current_job_points_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int, Station> station_locations;
    std::unordered_map<int, int> product_locations; // Products are only allowed to be at stations
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    Pose2d current_pose_;

    std::atomic<bool> is_manual_mode_ = true;
    std::atomic<bool> is_nav2_mode_ = false;

    bool is_active = false; // Is the system allowed to perform operations
    bool debug_enable_logging = true;  // Is the system allowed to print debug messages
    
    int pickup_station_id = 0;
    int dropoff_station_id = 0;
    int package_id = 0;
    JobStatus status = JobStatus::Idle;

    const double xy_tolerance = 0.1;  // 10 cm position
    const double yaw_tolerance = 0.1; // 5.7 degrees ish


    std::shared_ptr<MovingNode> manual_mover;
    
    void tag_id_callback(const std_msgs::msg::Int32::SharedPtr msg);  // Declare tag_id_callback
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tag_id_subscription_;  // Subscription for tag ID
    std::optional<int> latest_detected_tag_;  // Store the latest detected tag ID (optional to handle no detection)
};

#endif // TASK_PLANNER_H
