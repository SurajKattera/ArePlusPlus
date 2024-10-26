#include <unordered_map>
#include <queue>
#include <stdexcept>
#include <optional>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp" 
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <utility>

#include "TaskPlanner.h"
#include "Structures.h"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
/*
// DO NOT USE THIS ONE PRETTY PLEASE
class TaskPlanner : public rclcpp::Node {
public:
    TaskPlanner(std::vector<std::pair<int, int>> initial_tasks) : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));

        move_orders_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/move_orders", 10, std::bind(&TaskPlanner::move_orders_callback, this, std::placeholders::_1));
        
        nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
            this, "navigate_to_pose");

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&TaskPlanner::try_assign_task, this));


        for (auto task: initial_tasks){
            order_queue.push(task);
        }
        RCLCPP_INFO(this->get_logger(), "Received %li tasks", initial_tasks.size());
    }

    TaskPlanner() : Node("task_planner") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));

        move_orders_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/move_orders", 10, std::bind(&TaskPlanner::move_orders_callback, this, std::placeholders::_1));
    }

    void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received goal pose: x=%f, y=%f, z=%f, orientation: x=%f, y=%f, z=%f, w=%f",
                    msg->pose.position.x,
                    msg->pose.position.y,
                    msg->pose.position.z,
                    msg->pose.orientation.x,
                    msg->pose.orientation.y,
                    msg->pose.orientation.z,
                    msg->pose.orientation.w);
    }

    // Set a product location
    void set_product_location(int id, float x, float y, float yaw) {
        product_locations[id] = Pose2d(x, y, yaw);
    }

    // Set all product locations 
    void set_all_product_locations(const std::unordered_map<int, Pose2d>& new_locations) {
        product_locations = new_locations;
        RCLCPP_INFO(this->get_logger(), "Updated product locations. New count: %zu", product_locations.size());
    }

    // Get a product location
    Pose2d get_product_location(int id) {
        auto it = product_locations.find(id);
        if (it == product_locations.end()) {
            throw std::runtime_error("Product ID not found");
        }
        return it->second;
    }

    // Set all station locations (overwrites the entire map)
    void set_all_station_locations(const std::unordered_map<int, Pose2d>& new_locations) {
        station_locations = new_locations;
    }

    // Get a station location
    Pose2d get_station_location(int id) {
        auto it = station_locations.find(id);
        if (it == station_locations.end()) {
            throw std::runtime_error("Station ID not found");
        }
        return it->second;
    }

    // Get the next order from the queue
    std::pair<int, int> get_next_order() {
        if (order_queue.empty())
        {
            throw std::runtime_error("No orders available");
        }
        auto order = order_queue.front();
        order_queue.pop();
        return order;
    }

    // Check if there are any orders in the queue
    bool has_orders() {
        return !order_queue.empty();
    }

    bool get_next_navtask(std::vector<geometry_msgs::msg::Pose>& poses) {
        if (!has_orders()) {
            return false;
        }

        std::pair<int, int> current_order_ = get_next_order();

        try {
            poses.clear();

            // Get product location
            auto product_loc = get_product_location(current_order_.first);
            poses.push_back(product_loc.to_pose());

            // Get station location
            auto station_loc = get_station_location(current_order_.second);
            poses.push_back(station_loc.to_pose());

            return true;

        } catch (const std::runtime_error& e) {
            RCLCPP_ERROR(this->get_logger(), "Error generating navtask: %s", e.what());
            return false;
        }
    }

    void set_activation_state(bool state) {
        activate = state;
        // Todo respect activation state, autoqueue
    }

private:
    void send_movement_command(const geometry_msgs::msg::PoseStamped& target_pose) {
        // Wait for action server
        while (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for action server...");
        }

        // Create goal
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target_pose;

        // Send goal
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&TaskPlanner::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&TaskPlanner::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&TaskPlanner::result_callback, this, std::placeholders::_1);

        nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options);
        RCLCPP_INFO(this->get_logger(), "Sent movement command to navstack: x=%f, y=%f",
                    target_pose.pose.position.x, target_pose.pose.position.y);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        //RCLCPP_INFO(this->get_logger(), "Received odom: x=%f, y=%f, z=%f",
        //            msg->pose.pose.position.x,
        //            msg->pose.pose.position.y,
        //            msg->pose.pose.position.z);
    }

    void move_orders_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2)
        {
            int first = msg->data[0];
            int second = msg->data[1];
            order_queue.push(std::make_pair(first, second));
            RCLCPP_INFO(this->get_logger(), "Queued move order: (%d, %d)", first, second);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Invalid move order");
        }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
        //RCLCPP_INFO(this->get_logger(), "Distance remaining: %f", feedback->distance_remaining);
    }

    void goal_response_callback(const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult & result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                ready_for_task = true;
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                ready_for_task = true;
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
    }

    void try_assign_task(){
        if (!activate) {return;}
        if (!ready_for_task) {return;}
        if (!has_orders()) {return;}
        transmit_task();
    }

    void transmit_task() {
        if (!has_orders()) {
            RCLCPP_WARN(this->get_logger(), "No tasks in queue to process");
            return;
        }

        auto task = order_queue.front();
        order_queue.pop();

        auto product_location = get_product_location(task.first);
        
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.stamp = this->now();
        target_pose.header.frame_id = "map";
        target_pose.pose.position.x = product_location.pos_x;
        target_pose.pose.position.y = product_location.pos_y;
        target_pose.pose.position.z = 0.0;
        

        double cy = std::cos(product_location.yaw * 0.5);
        double sy = std::sin(product_location.yaw * 0.5);
        target_pose.pose.orientation.x = 0.0;
        target_pose.pose.orientation.y = 0.0;
        target_pose.pose.orientation.z = sy;
        target_pose.pose.orientation.w = cy;
        ready_for_task = false;
        send_movement_command(target_pose);
        RCLCPP_INFO(this->get_logger(), "Sent movement command for task: product %d to station %d", 
                    task.first, task.second);
        
    }

    std::unordered_map<int, Pose2d> product_locations;
    std::unordered_map<int, Pose2d> station_locations;
    std::queue<std::pair<int, int>> order_queue;
    
    bool activate = false;
    bool ready_for_task = true;
    Pose2d current_waypoint = Pose2d(0, 0, 0);
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr move_orders_sub_;
    
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp::TimerBase::SharedPtr timer_;


};
*/
