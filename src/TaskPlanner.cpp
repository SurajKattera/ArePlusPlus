#include "TaskPlanner.h"

TaskPlanner::TaskPlanner(std::vector<std::pair<int, int>> initial_tasks)
    : Node("task_planner") {
    load_locations_from_file();
        
    manual_mover = std::make_shared<MovingNode>(this);
    tagger_node = std::make_shared<ArtagDetectorNode>(this);
    

     // Create the action client
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TaskPlanner::timer_callback, this));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));

    for (auto task : initial_tasks) {
        Order order(task.first, task.second);
        pending_orders_.push(order);
    }
}

TaskPlanner::TaskPlanner() : Node("task_planner") {
     
    manual_mover = std::make_shared<MovingNode>(this);
    tagger_node = std::make_shared<ArtagDetectorNode>(this);
    

     // Create the action client
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TaskPlanner::timer_callback, this));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));

}

void TaskPlanner::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pose_.pos_x = msg->pose.pose.position.x;
    current_pose_.pos_y = msg->pose.pose.position.y;
    
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose_.yaw = yaw;
}

bool TaskPlanner::is_at_target(const Pose2d &target) {
    if (is_manual_mode_) {
        return manual_mover->is_done();
    }

    double dx = std::abs(current_pose_.pos_x - target.pos_x);
    double dy = std::abs(current_pose_.pos_y - target.pos_y);
    double dyaw = std::abs(current_pose_.yaw - target.yaw);

    while (dyaw > M_PI) dyaw -= 2 * M_PI;
    dyaw = std::abs(dyaw);

    return (dx <= xy_tolerance) && (dy <= xy_tolerance) && (dyaw <= yaw_tolerance);
}

void TaskPlanner::nav2_go_to_point(const Pose2d &target) {
    if (is_manual_mode_) {
        RCLCPP_INFO(this->get_logger(), "Switching from manual mode to Nav2 mode.");
        // Here, you'd ideally stop the manual movement mode before continuing
        is_manual_mode_ = false; // Set manual mode to false
        is_nav2_mode_ = true;    // Set Nav2 mode to true
        manual_mover->stop_manual();
    }

   

    // Wait for the action server to be available
    if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "NavigateToPose action server not available after waiting");
        this->is_active = false;
        return;
    }

    // Set up the goal with the target pose
    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.pose.position.x = target.pos_x;
    goal.pose.pose.position.y = target.pos_y;
    goal.pose.pose.orientation.z = sin(target.yaw / 2.0);
    goal.pose.pose.orientation.w = cos(target.yaw / 2.0);

    // Send the goal to the action server
    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto result) {
        RCLCPP_INFO(this->get_logger(), "Navigation to target completed.");
    };
    
    action_client_->async_send_goal(goal, send_goal_options);
    RCLCPP_INFO(this->get_logger(), "Sending goal to Nav2...");

}

void TaskPlanner::manual_go_to_point(const Pose2d &target) {
    if (is_nav2_mode_) {
        RCLCPP_INFO(this->get_logger(), "Switching from Nav2 mode to manual mode.");
        is_manual_mode_ = true; 
        is_nav2_mode_ = false;   
        // TODO @jack need code here to forcefully stop the nav2 function if it's still going
    }
    manual_mover->go_to_point(target);
}

void TaskPlanner::prep_next_order() {
    if (!pending_orders_.size()) {
        return;
    }

    Order new_order = pending_orders_.front();
    pending_orders_.pop();

    if (product_locations.find(new_order.product_id) == product_locations.end()) {
        RCLCPP_WARN(this->get_logger(), "Product ID %d not found in product locations. Skipping order.", new_order.product_id);
        prep_next_order();  // Try to get a new order
        return;
    }

    if (station_locations.find(new_order.station_id) == station_locations.end()) {
        RCLCPP_WARN(this->get_logger(), "Station ID %d not found in station locations. Skipping order.", new_order.station_id);
        prep_next_order();  // Try to get a new order
        return;
    }

    while (current_job_points_.size()) {
        current_job_points_.pop();
    }

    auto center_to_pickup = station_locations[product_locations[new_order.product_id]].path;
    auto center_to_dropoff = station_locations[new_order.station_id].path;

    if (center_to_pickup.empty()) {
        RCLCPP_WARN(this->get_logger(), "Warning: center_to_pickup is empty!");
        assert(!center_to_pickup.empty() && "center_to_pickup must have at least one element");
        return;
    }

    if (center_to_dropoff.empty()) {
        RCLCPP_WARN(this->get_logger(), "Warning: center_to_dropoff is empty!");
        assert(!center_to_dropoff.empty() && "center_to_dropoff must have at least one element");
        return;
    }

    NavNode node(ActionType::start);
    current_job_points_.push(node);

    for (auto point : center_to_pickup) {
        current_job_points_.push(point);
    }

    node.action_type = ActionType::pickup;
    current_job_points_.push(node);
    node.action_type = ActionType::advance_state;
    current_job_points_.push(node);

    for (auto it = center_to_pickup.rbegin() + 1; it != center_to_pickup.rend(); ++it) {
        current_job_points_.push(*it);
    }

    node.action_type = ActionType::advance_state;
    current_job_points_.push(node);

    for (auto point : center_to_dropoff) {
        current_job_points_.push(point);
    }

    node.action_type = ActionType::dropoff;
    current_job_points_.push(node);
    node.action_type = ActionType::advance_state;
    current_job_points_.push(node);

    for (auto it = center_to_dropoff.rbegin() + 1; it != center_to_dropoff.rend(); ++it) {
        current_job_points_.push(*it);
    }

    node.action_type = ActionType::finish;
    current_job_points_.push(node);

    package_id = new_order.product_id;
    dropoff_station_id = new_order.station_id;
    pickup_station_id = product_locations[new_order.product_id];
}

bool TaskPlanner::load_locations_from_file() {
    /*
    shelf#2 (-2,2)    shelf#3 (2,2)

    shelf#1 (-2,-1)   shelf#4 (2,-1)
    */
    // Create path1 with two waypoints
    std::vector<NavNode> path1;
    std::vector<NavNode> path2;
    std::vector<NavNode> path3;
    std::vector<NavNode> path4;

    
    // First waypoint: (0.5, -1, 3.14)
    NavNode waypoint1(ActionType::normal);
    waypoint1.pose = Pose2d(0.5, -1, 3.14);
    waypoint1.is_manual_approach = true;
    waypoint1.is_final_approach = false;
    path1.emplace_back(std::move(waypoint1));  // Add the first waypoint
    
    // Second waypoint: (-2, -1, -3.14)
    NavNode waypoint2(ActionType::normal);
    waypoint2.pose = Pose2d(-2, -1, 0);
    waypoint2.is_manual_approach = true;
    waypoint2.is_final_approach = true;
    path1.emplace_back(std::move(waypoint2));  // Add the second waypoint

    // // Third waypoint: (-2.5, -1, -3.14)
    // NavNode waypoint3(ActionType::normal);
    // waypoint3.pose = Pose2d(-2.5, -1, -3.14);
    // waypoint3.is_manual_approach = true;
    // waypoint3.is_final_approach = false;
    // path1.emplace_back(std::move(waypoint3));  // Add the third waypoint, beyond the shelf
    
    // Assign the path to station_locations[-1]
    station_locations[-1] = Station(0, Pose2d(-2, -1, 0), std::move(path1));



    //path2
    // First waypoint: (0.5, 1, 3.14)
    path2.emplace_back(Pose2d(0.5, 1, 3.14), true, false);
    // Second waypoint: (-2, 2, 0)
    path2.emplace_back(Pose2d(-2, 2, 0), true, true);


    //path3
    // First waypoint: (-0.5, 1, 0)
    path3.emplace_back(Pose2d(-0.5, 1, 0), true, false);
    // Second waypoint: (2, 2, 3.14)
    path3.emplace_back(Pose2d(2, 2, 3.14), true, true);


    //path4
    // First waypoint: (-0.5, -1, 0)
    path3.emplace_back(Pose2d(-0.5, -1, 0), true, false);
    // Second waypoint: (2, -1, 3.14)
    path3.emplace_back(Pose2d(2, -1, 3.14), true, true);
    
    // // Load station locations with paths generated from the center to the station
    station_locations[1] = Station(1, Pose2d(-2.5, -2.5, 0), NavNode(Pose2d(-2.5, -2.5, 0), true, false));
    station_locations[2] = Station(2, Pose2d(0, -2.5, 0), NavNode(Pose2d(-2.5, -2.5, 0), true, false));
    station_locations[3] = Station(3, Pose2d(2.5, -2.5, 0), NavNode(Pose2d(-2.5, -2.5, 0), true, false));

    // station_locations[-1] = Station(-1, Pose2d(-2, -1, 0), generatePathToStation(Pose2d(-2, -1, 0)));
    // station_locations[-2] = Station(-2, Pose2d(-2, 2, 0), generatePathToStation(Pose2d(-2, 2, 0)));
    // station_locations[-3] = Station(-3, Pose2d(2, 2, 0), generatePathToStation(Pose2d(2, 2, 0)));
    // station_locations[-4] = Station(-4, Pose2d(2, -1, 0), generatePathToStation(Pose2d(2, -1, 0)));

    product_locations[1] = -1;
    product_locations[2] = -2;
    product_locations[3] = -3;
    product_locations[4] = -4;

    return true;
}

bool TaskPlanner::get_visible_station_code(int& tag_id) {
    // Needs: //ros2 run apriltag_ros apriltag_node --ros-args   -r image_rect:=/camera/image_raw   -r camera_info:=/camera/camera_info   -p family:=36h11   -p size:=0.5   -p max_hamming:=0
    return tagger_node->get_last_seen_tag(tag_id);
};

void TaskPlanner::timer_callback() {
    if (!this->is_active) { // If the system is disabled
        return;
    }
    // If the current job list is empty, try to get new orders
    if (current_job_points_.empty()) {
        prep_next_order();

        // If its still empty, nothing left to do
        if (current_job_points_.empty()) {
            this->is_active = false;
            return;
        }
    }

    // Check if we're at our current target
    if (is_at_target(current_job_points_.front().pose) || // This is for a normal action
    current_job_points_.front().action_type != ActionType::normal) {                              // This indicates a special action
        current_job_points_.pop(); // Clear the current task

        if (!current_job_points_.empty()) {
            switch (current_job_points_.front().action_type) {
            case ActionType::normal:
                if (current_job_points_.front().is_manual_approach) {
                    manual_go_to_point(current_job_points_.front().pose); // Command to move to the next one
                }
                else {
                    nav2_go_to_point(current_job_points_.front().pose); // Let Nav2 do it
                }
                break;
            case ActionType::advance_state:
                if (status == JobStatus::ToPickup) {
                    status = JobStatus::FromPickup;
                }
                else if (status == JobStatus::FromPickup) {
                    status = JobStatus::ToDestination;
                }
                else if (status == JobStatus::ToDestination) {
                    status = JobStatus::FromDestination;
                }
                else {
                    RCLCPP_WARN(this->get_logger(), "Unexpected job status. This may indicate that the job wasn't reset properly or has more than 4 phases.");
                }
                break;
            case ActionType::pickup:
                RCLCPP_INFO(this->get_logger(), "Packages picked up");
                break;
            case ActionType::dropoff:
                RCLCPP_INFO(this->get_logger(), "Packages dropped off");
                break;
            case ActionType::start:
                RCLCPP_INFO(this->get_logger(), "New job, moving to target location now");
                status = JobStatus::ToPickup;
                break;
            case ActionType::finish:
                RCLCPP_INFO(this->get_logger(), "Job finished, remaining stationed");
                break;
            default:
                // This means an invalid action type, trigger an error
                RCLCPP_ERROR(this->get_logger(), "Invalid action type encountered");
                break;
            }
        }

        if (current_job_points_.front().is_final_approach) {
            int station_id = 0;
            if (!get_visible_station_code(station_id))
                return;
             // The apriltag doesn't match the expected value
            if (status == JobStatus::ToPickup && station_id != station_locations[pickup_station_id].station_id) {
                RCLCPP_ERROR(this->get_logger(), "Error: Incorrect pickup location. Expected station ID %d, but found %d. Task aborted.", station_locations[pickup_station_id].station_id, station_id);
                // TODO actually abort here
            }
            if (status == JobStatus::ToDestination && station_id != station_locations[dropoff_station_id].station_id) {
                RCLCPP_ERROR(this->get_logger(), "Error: Incorrect dropoff location. Expected station ID %d, but found %d. Task aborted.", station_locations[dropoff_station_id].station_id, station_id);
                // TODO actually abort here
            }
        }
    }
}


void TaskPlanner::set_activation_state(bool state) {
    this->is_active = state;
}
