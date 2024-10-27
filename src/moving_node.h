#ifndef MOVING_NODE_H
#define MOVING_NODE_H

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <utility>

class MovingNode : public rclcpp::Node {
public:
    MovingNode();

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry odometry_;
    double odometry_yaw_;
    int state_;
    bool move_now_;
    std::pair<double, double> my_goal_point_;
    double tolerance_;

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    void moveSetter(std::pair<double, double> my_point, double distance_to_point);
    bool moveIt();
    double quaternionToYaw(geometry_msgs::msg::Quaternion quat);
    void cmdSender(double angular_velocity, double linear_velocity);

    
};

#endif // MOVING_NODE_H