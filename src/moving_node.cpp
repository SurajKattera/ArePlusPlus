#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include "moving_node.h"

MovingNode::MovingNode(rclcpp::Node* node) : Node("my_robot_mover"), state_(1) {
    // Subscribe for the Lcommand for vel
    cmd_vel_pub_ = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscribe to Odometry
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&MovingNode::odom_callback, this, std::placeholders::_1));

    timer_ = node->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&MovingNode::moveIt, this)
    );
}

void MovingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    nav_msgs::msg::Odometry local_odom_msg = *odom_msg;
    odometry_ = local_odom_msg;
    quaternionToYaw(local_odom_msg.pose.pose.orientation);

    if (!this->is_silent) {
         RCLCPP_INFO(this->get_logger(), "OD %f", odometry_.twist.twist.angular.z);
    }
}

void MovingNode::go_to_point(Pose2d my_point) {
    tolerance_ = 0.2;
    my_goal_point_.pos_x = my_point.pos_x;
    my_goal_point_.pos_y = my_point.pos_y;
    my_goal_point_.yaw = my_point.yaw;
    move_now_ = true;
    state_ = 1;
}

// This is not a duplicate function. Do not remove
void MovingNode::go_to_point(Pose2d my_point, double tolerance) {
    tolerance_ = tolerance;
    my_goal_point_.pos_x = my_point.pos_x;
    my_goal_point_.pos_y = my_point.pos_y;
    my_goal_point_.yaw = my_point.yaw;
    move_now_ = true;
    state_ = 1;
}

bool MovingNode::moveIt() {
    double safe_stop = 0.03;

    double turn_l_r = 0.2;
    double turn_desired = 0.3;
    double move_f_b = 1.0; // This is the throttle
    double epsilon = 0.1; // Angular error
    
    if((std::fabs(std::abs(odometry_.pose.pose.position.x - my_goal_point_.pos_x)) <= safe_stop &&
        std::fabs(std::abs(odometry_.pose.pose.position.y - my_goal_point_.pos_y)) <= safe_stop)) {
        move_f_b = 0.05;
        turn_l_r = 0.04;
        epsilon = 0.03;
    }

    double desired_ang = atan2((my_goal_point_.pos_y - odometry_.pose.pose.position.y),
                                (my_goal_point_.pos_x - odometry_.pose.pose.position.x));

    // These two lines are for turning clockwise and anticlockwise according to the position
    if(desired_ang - odometry_yaw_ > 0) turn_l_r = std::abs(turn_l_r);
    else if(desired_ang - odometry_yaw_ < 0) turn_l_r = -turn_l_r;

    // Main state machine for rotating and moving linearly
    switch(state_) {
        case 1:
                cmdSender(turn_l_r, 0);
            if((std::abs(desired_ang - odometry_yaw_) <= epsilon)) {
                state_ = 2;
            }
        break;
        case 2:
            cmdSender(0, move_f_b);

            if((std::abs(desired_ang - odometry_yaw_) > epsilon)) {
                state_ = 1;
            }
            if(std::abs(odometry_.pose.pose.position.x - my_goal_point_.pos_x) < tolerance_ &&
            std::abs(odometry_.pose.pose.position.y - my_goal_point_.pos_y) < tolerance_){
                state_ = 3;
            }
        break;
        case 3:
            if(my_goal_point_.yaw - odometry_yaw_ > 0) turn_desired = std::abs(turn_desired);
            else if(my_goal_point_.yaw - odometry_yaw_ < 0) turn_desired = -turn_desired;
            
            cmdSender(turn_desired, 0);

            if(std::abs(my_goal_point_.yaw - odometry_yaw_) < 0.2){
                cmdSender(0,0);
                move_now_ = false;
                state_ = 4;
            }

        break;
        case 4:
            if (!this->is_silent) {
                RCLCPP_INFO(this->get_logger(), "Location achieved! X value: %le || Y value: %le", my_goal_point_.pos_x,  my_goal_point_.pos_y);
            }
        return true;
        break;
    }
    return false;
}

double MovingNode::quaternionToYaw(geometry_msgs::msg::Quaternion quat) {
    double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
    double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);

    // Compute yaw
    odometry_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    return std::atan2(siny_cosp, cosy_cosp);  // Yaw is returned in radians
}

void MovingNode::cmdSender(double angular_velocity, double linear_velocity) {
    geometry_msgs::msg::Twist twist_msg;

    // Compute the velocity components based on yaw
    twist_msg.linear.x = std::abs(linear_velocity * std::cos(odometry_yaw_));  // X component
    twist_msg.linear.y = std::abs(linear_velocity * std::sin(odometry_yaw_));  // Y component

    // Z-axis remains zero for ground robots
    twist_msg.linear.z = 0.0;

    twist_msg.angular.x = 0.0;
    twist_msg.angular.y = 0.0;
    twist_msg.angular.z = (angular_velocity);  // Z-axis angular velocity (yaw)

    cmd_vel_pub_->publish(twist_msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
