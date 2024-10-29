#ifndef STRUCTURE_H
#define STRUCTURE_H

#include "geometry_msgs/msg/pose.hpp"
#include <vector>


enum class ActionType {
    normal,
    start,
    finish,
    pickup,
    dropoff,
    advance_state
};

enum class JobStatus {
    Error,
    Estop,
    Idle,
    ToPickup,
    FromPickup,
    ToDestination,
    FromDestination
};

struct Pose2d {
public:
    double pos_x = 0;
    double pos_y = 0;
    double yaw = 0;

    Pose2d() = default;
    Pose2d(double x, double y, double yaw) : pos_x(x), pos_y(y), yaw(yaw) {}

    geometry_msgs::msg::Pose to_pose() const {
        geometry_msgs::msg::Pose pose;
        pose.position.x = static_cast<double>(pos_x);
        pose.position.y = static_cast<double>(pos_y);
        pose.position.z = 0.0;
        double cy = std::cos(yaw * 0.5);
        double sy = std::sin(yaw * 0.5);
        pose.orientation.x = 0.0;
        pose.orientation.y = 0.0;
        pose.orientation.z = sy;
        pose.orientation.w = cy;
        return pose;
    }
};

struct Order {
    int product_id;
    int station_id;
    
    Order(int prod, int station) : product_id(prod), station_id(station) {}
};

struct NavNode {
    Pose2d pose;
    bool is_manual_approach = false; 
    bool is_final_approach = false;
    bool respect_yaw = true;
    ActionType action_type = ActionType::normal;

    NavNode() = default; 
    explicit NavNode(ActionType type) : action_type(type) {}
    explicit NavNode(Pose2d p, bool is_man, bool is_final, bool do_yaw) {
        pose = p; 
        is_manual_approach = is_man; 
        is_final_approach = is_final;
        respect_yaw = do_yaw;
    }
};

struct Station {
    int station_id;
    Pose2d location;
    std::vector<NavNode> path;

    Station() : station_id(0) {}
    Station(int id, Pose2d loc, std::vector<NavNode> pth) 
        : station_id(id), location(std::move(loc)), path(std::move(pth)) {}
    Station(int id, Pose2d loc, NavNode pth) 
        : station_id(id), location(std::move(loc)) {
            path.push_back(pth);
        }

};

#endif  // STRUCTURE_H