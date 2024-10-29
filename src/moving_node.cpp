#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include "moving_node.h"
MovingNode::MovingNode(rclcpp::Node* node) : Node("my_robot_mover"), state_(0) {
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

bool MovingNode::is_done() {
    return (state_ == 3);
}

void MovingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    nav_msgs::msg::Odometry local_odom_msg = *odom_msg;
    odometry_ = local_odom_msg;
    quaternionToYaw(local_odom_msg.pose.pose.orientation);

    if (!this->is_silent) {
         //RCLCPP_INFO(this->get_logger(), "OD %f", odometry_.twist.twist.angular.z);
    }
}
void MovingNode::go_to_point(Pose2d my_point, bool ignore_yaw) {
    tolerance_ = 0.2;
    my_goal_point_ = my_point;
    move_now_ = true;
    state_ = 0;
    yaw_check = ignore_yaw;
}

void MovingNode::go_to_point(Pose2d my_point, double tolerance) {
    tolerance_ = tolerance;
    my_goal_point_ = my_point;
    move_now_ = true;
    state_ = 0;
}

void MovingNode::stop_manual() {
    state_ = 3;
}
bool MovingNode::moveIt() {
    // @suraj uncomment this and put your service boolean in and the e-stop will work
    // if(service_boolean_ == true) return false;
    
    const double phase_1_angular_vel = 0.4;
    const double phase_2_angular_vel = 0.1;
    const double phase_3_angular_vel = 0.4;
    const double max_linear_vel = 0.5;
    const double angular_error_threshold = 0.05;
    const double distance_threshold = 1.0;
    
    double desired_ang = atan2(my_goal_point_.pos_y - odometry_.pose.pose.position.y,
                               my_goal_point_.pos_x - odometry_.pose.pose.position.x);
    double angle_diff = desired_ang - odometry_yaw_;
    angle_diff = atan2(sin(angle_diff), cos(angle_diff)); // Normalize to [-π, π]

    double distance_to_goal = hypot(my_goal_point_.pos_x - odometry_.pose.pose.position.x,
                                    my_goal_point_.pos_y - odometry_.pose.pose.position.y);
    switch(state_) {
        case 0: // Turn to face destination
            if(fabs(angle_diff) > angular_error_threshold) {
                double angular_vel = copysign(phase_1_angular_vel, angle_diff);
                cmdSender(angular_vel, 0);
                RCLCPP_INFO(this->get_logger(), "State: 0, Bot: (%f, %f), Target: (%f, %f), Command: angular_vel = %f, linear_vel = 0", 
                            odometry_.pose.pose.position.x, odometry_.pose.pose.position.y,
                            my_goal_point_.pos_x, my_goal_point_.pos_y, angular_vel);
            } else {
                state_ = 1;
            }
        break;

        case 1: // Move towards destination
            if(distance_to_goal > tolerance_) {
                double angular_correction = std::min(fabs(angle_diff) / angular_error_threshold, 1.0) * phase_2_angular_vel;
                
                // Calculate linear velocity with falloff
                double linear_vel;
                if (distance_to_goal > distance_threshold) {
                    linear_vel = max_linear_vel;
            } else {
                    linear_vel = max_linear_vel * (distance_to_goal / distance_threshold);
            }    
            double angular_vel = copysign(angular_correction, angle_diff);
            cmdSender(angular_vel, linear_vel);
            RCLCPP_INFO(this->get_logger(), "State: 1, Bot: (%f, %f), Target: (%f, %f), Command: angular_vel = %f, linear_vel = %f", 
                        odometry_.pose.pose.position.x, odometry_.pose.pose.position.y,
                        my_goal_point_.pos_x, my_goal_point_.pos_y, angular_vel, linear_vel);
            } else {
                if (yaw_check) {
                    state_ = 2;
                } else {
                    cmdSender(0, 0);
                    move_now_ = false;
                    state_ = 3;
                    if (!this->is_silent) {
                        RCLCPP_INFO(this->get_logger(), "Location achieved! Bot: (%f, %f), Target: (%f, %f)", 
                                    odometry_.pose.pose.position.x, odometry_.pose.pose.position.y,
                                    my_goal_point_.pos_x, my_goal_point_.pos_y);
                    }
                }
            }
        break;

        case 2: // Turn to desired yaw (only if yaw_check is true)
            {
                double yaw_diff = my_goal_point_.yaw - odometry_yaw_;
                yaw_diff = atan2(sin(yaw_diff), cos(yaw_diff)); // Normalize to [-π, π]
                
                if(fabs(yaw_diff) > angular_error_threshold) {
                    double angular_vel = copysign(phase_3_angular_vel, yaw_diff);
                    cmdSender(angular_vel, 0);
                    RCLCPP_INFO(this->get_logger(), "State: 2, Bot: (%f, %f), Target: (%f, %f), Command: angular_vel = %f, linear_vel = 0", 
                                odometry_.pose.pose.position.x, odometry_.pose.pose.position.y,
                                my_goal_point_.pos_x, my_goal_point_.pos_y, angular_vel);
                } else {
                    cmdSender(0, 0);
                    move_now_ = false;
                    state_ = 3;
                    if (!this->is_silent) {
                        RCLCPP_INFO(this->get_logger(), "Location achieved! Bot: (%f, %f), Target: (%f, %f)", 
                                    odometry_.pose.pose.position.x, odometry_.pose.pose.position.y,
                                    my_goal_point_.pos_x, my_goal_point_.pos_y);
                    }
                }
            }
        break;

        case 3: // Done
            return true;
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
    //RCLCPP_INFO(this->get_logger(), "cmdSender input: angular_velocity = %f, linear_velocity = %f", angular_velocity, linear_velocity);

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
    // Send command 
}
