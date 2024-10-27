
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include "moving_node.h"

MovingNode::MovingNode() : Node("my_robot_mover"), state_(1) {
    // Subscribe for the Lcommand for vel
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Subscribe to Odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MovingNode::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&MovingNode::moveIt, this)
    );
}

void MovingNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
    // std::pair<double, double> point;
    // point.first = 2.0;
    // point.second = 4.0;

    nav_msgs::msg::Odometry local_odom_msg = *odom_msg;
    odometry_ = local_odom_msg;
    quaternionToYaw(local_odom_msg.pose.pose.orientation);

    if (!this->is_silent) {
         RCLCPP_INFO(this->get_logger(), "OD %f", odometry_.twist.twist.angular.z);
    }
    // moveSetter({1.0,2.0,2.0}); // delete
}

void MovingNode::go_to_point(Pose2d my_point) {
    tolerance_ = 0.2;
    my_goal_point_.pos_x = my_point.pos_x;
    my_goal_point_.pos_y = my_point.pos_y;
    my_goal_point_.yaw = my_point.yaw;
    move_now_ = true;
}

void MovingNode::go_to_point(Pose2d my_point, double tolerance) {
    tolerance_ = tolerance;
    my_goal_point_.pos_x = my_point.pos_x;
    my_goal_point_.pos_y = my_point.pos_y;
    my_goal_point_.yaw = my_point.yaw;
    move_now_ = true;
}

// int moveIt(std::pair<double, double> my_point, double distance_to_point, double my_offcourse_limit)
bool MovingNode::moveIt() {
      // double distance_to_point = 0.1;
    double safe_stop = 0.03;

    // double tolerance = distance_to_point;

    double turn_l_r = 0.1;
    double turn_desired = 0.1;
    double move_f_b = 1.0; // This is the throttle
    double epsilon = 0.1; // Angular error
    
    // my_goal.first = 3.0;
    // my_goal.second = 10.0;
    // getOdometry();
    // initial_odo_ = odometry_;

    // // This is to visualise the goal position on RVIZ
    // pfms::geometry_msgs::Point goal_point{my_goal.x, my_goal.y};
    // pfms::geometry_msgs::Goal goal{j++,goal_point};
    // pfmsConnectorPtr_->send(goal);

    // while(!(std::fabs(std::abs(odometry_.pose.pose.position.x - my_goal.first)) <= tolerance && std::fabs(std::abs(odometry_.pose.pose.position.y- my_goal.second)) <= tolerance)){

    // if((std::abs(odometry_.pose.pose.position.x - my_goal_point_.pos_x) > tolerance_ ||
    //     std::abs(odometry_.pose.pose.position.y - my_goal_point_.pos_y) > tolerance_ ||
    //     std::abs(my_goal_point_.yaw - odometry_yaw_) > 0.2 ||
    //     move_now_ == true)) {

        if((std::fabs(std::abs(odometry_.pose.pose.position.x - my_goal_point_.pos_x)) <= safe_stop &&
            std::fabs(std::abs(odometry_.pose.pose.position.y - my_goal_point_.pos_y)) <= safe_stop)) {
            move_f_b = 0.05;
            turn_l_r = 0.04;
            epsilon = 0.03;
        }

        // std::cout << "Speed-----------------------------------" << odometry_.twist.twist.linear.x << "    " << odometry_.twist.twist.linear.y << std::endl;

        double desired_ang = atan2((my_goal_point_.pos_y - odometry_.pose.pose.position.y),
                                   (my_goal_point_.pos_x - odometry_.pose.pose.position.x));

        // These two lines are for turning clockwise and anticlockwise according to the position
        if(desired_ang - odometry_yaw_ > 0) turn_l_r = std::abs(turn_l_r);
        else if(desired_ang - odometry_yaw_ < 0) turn_l_r = -turn_l_r;

        // std::cout << "STATE:::::::::::::::::::::::::: " << state_ << std::endl;

        // Main state machine for rotating and moving linearly
        switch(state_) {
            case 1:
                // while(!(std::fabs(std::abs(desired_ang - odometry_yaw_)) <= epsilon)){
                    cmdSender(turn_l_r, 0);
                    // std::cout << "Yaw                                                " << odometry_yaw_ << std::endl;
                    // std::cout << "DESIREDDDDDDDDDD " << desired_ang << std::endl;

                if((std::abs(desired_ang - odometry_yaw_) <= epsilon)) {
                    // std::cout << "STATE:::::::::::::::::::::::::: " << state_ << std::endl;
                    state_ = 2;
                }
            break;
            case 2:
                // std::cout << "linEAR" << std::endl;
                // std::cout << "----------------------------------------------------------------------------------------- " << state_ << std::endl;
                cmdSender(0, move_f_b);

                // std::cout << "POS " << odometry_.pose.pose.position.x << "&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&" << odometry_.pose.pose.position.y << std::endl;
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

                // if(std::abs(odometry_.pose.pose.position.x - my_goal_point_.pos_x) > tolerance_) std::cout << "WORKS " << std::endl;
                // if(std::abs(odometry_.pose.pose.position.y - my_goal_point_.pos_y) > tolerance_ ) std::cout << "WLORKS " << std::endl;

                // std::cout << "Yaw                                                " << odometry_yaw_ << std::endl;
                //     std::cout << "DESIREDDDDDDDDDD " << my_goal_point_.yaw << std::endl;

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
    // }
//     else {
//         // Send a last command to stop the Turtlebot from moving
            
//     //     if (!this->is_silent) {
//     //     RCLCPP_INFO(this->get_logger(), "Location achieved! X value: %le || Y value: %le", my_goal_point_.first,  my_goal_point_.second);
//     // }
//         move_now_ = false;
//         // my_goal_point_ = {2.0, 5.0};
//         std::cout << "here Hermano " << std::endl;
//         return true;
// }

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

    // rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_; ///< Publisher for cylinder visualization markers
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_; ///< Subscription to LaserScan data
    // nav_msgs::msg::Odometry odometry_;
    // double odometry_yaw_;
    // rclcpp::TimerBase::SharedPtr timer_;
    // std::pair<double,double> my_goal;
    // int state_;
    // bool move_now_;
    // std::pair<double,double> my_goal_point_;
    // double tolerance_;

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<MovingNode>());
//     rclcpp::shutdown();
//     return 0;
// }



