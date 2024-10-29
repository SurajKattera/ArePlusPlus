#ifndef TASK_PLANNER_H
#define TASK_PLANNER_H

// Standard library includes
#include <atomic>
#include <queue>
#include <unordered_map>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// Message includes
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// Action includes
#include <nav2_msgs/action/navigate_to_pose.hpp>

// TF2 includes
#include <tf2/LinearMath/Matrix3x3.h> 
#include <tf2/LinearMath/Quaternion.h>

// Custom includes
#include "Structures.h"
#include "moving_node.h"
#include "artag_detector_node.h"
using NavigateToPose = nav2_msgs::action::NavigateToPose;
    

class TaskPlanner : public rclcpp::Node {
public:
    /**
     * @brief Default Constructor
     */
    TaskPlanner();

    /**
     * @brief Constructor with initial tasks and a parameter
     * @param initial_tasks A vector of product id and station id pairs
     */
    TaskPlanner(std::vector<std::pair<int, int>> initial_tasks);

    /**
     * @brief Set the activation state of the task planner
     * @param state Desired activation status
     */
    void set_activation_state(bool state);
     
private:
    // Member variables
    std::queue<Order> pending_orders_;
    std::queue<NavNode> current_job_points_;
    std::unordered_map<int, Station> station_locations;
    std::unordered_map<int, int> product_locations;
    Pose2d current_pose_;
    std::atomic<bool> is_manual_mode_ = true;
    std::atomic<bool> is_nav2_mode_ = false;
    bool is_active = false;
    bool debug_enable_logging = true;
    int pickup_station_id = 0;
    int dropoff_station_id = 0;
    int package_id = 0;
    JobStatus status = JobStatus::Idle;
    const double xy_tolerance = 0.1;
    const double yaw_tolerance = 0.1;
    std::optional<int> latest_detected_tag_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;


    // ROS2 related members
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::shared_ptr<MovingNode> manual_mover;
    std::shared_ptr<ArtagDetectorNode> tagger_node;
    
    // Member functions
    /**
     * @brief Timer callback function. Handles main buisness logic
     */
    void timer_callback();

    /**
     * @brief Odometry callback function
     * @param msg Odom message
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /**
     * @brief Check if the robot is at the target position, with tolerances
     * @param target desired target to check against
     * @return boolean is within tolerance of target
     */
    bool is_at_target(const Pose2d &target);

    /**
     * @brief Move to a point using manual control
     * @param target desired target to visit
     * @param yaw does yaw need to be obeyed
     */
    void manual_go_to_point(const Pose2d &target, bool yaw);

    /**
     * @brief Move to a point using Nav2
     * @param target desired target to visit
     */
    void nav2_go_to_point(const Pose2d &target);

    /**
     * @brief Prepare the next order. Internal only
     */
    void prep_next_order();

    /**
     * @brief Load storages from file. Internal only
     */
    bool load_locations_from_file();

    /**
     * @brief Get the visible station apriltag
     * @param tag_id Id of visible station
     * @return boolean was a station found
     */
    bool get_visible_station_code(int &tag_id);

};

#endif // TASK_PLANNER_H
