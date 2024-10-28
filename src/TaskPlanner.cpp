#include "TaskPlanner.h"

TaskPlanner::TaskPlanner(std::vector<std::pair<int, int>> initial_tasks)
    : Node("task_planner") {
    load_locations_from_file();
    
    
    manual_mover = std::make_shared<MovingNode>(this);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TaskPlanner::timer_callback, this));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));
        
    // Initialize subscription to /detected_tag_id
    //tag_id_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    //    "detected_tag_id", 10, std::bind(&TaskPlanner::tag_id_callback, this, std::placeholders::_1));

    for (auto task : initial_tasks) {
        Order order(task.first, task.second);
        pending_orders_.push(order);
    }
}

TaskPlanner::TaskPlanner()
    : Node("task_planner") {
     
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&TaskPlanner::timer_callback, this));

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&TaskPlanner::odom_callback, this, std::placeholders::_1));
        // Initialize subscription to /detected_tag_id
    //tag_id_subscription_ = this->create_subscription<std_msgs::msg::Int32>(
    //    "detected_tag_id", 10, std::bind(&TaskPlanner::tag_id_callback, this, std::placeholders::_1));
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
        // TODO @thish you need to include code here to forcefully stop the manualmover code
    }

    // Create the action client
    auto action_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this->shared_from_this(), "navigate_to_pose");

    // Wait for the action server to be available
    if (!action_client->wait_for_action_server(std::chrono::seconds(5))) {
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
        is_nav2_mode_ = false;  // Reset Nav2 mode after completion
    };
    
    action_client->async_send_goal(goal, send_goal_options);
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

std::vector<NavNode> TaskPlanner::generatePathToStation(const Pose2d &destination) {
    std::vector<NavNode> path;

    // Assume the robot starts at the origin (0, 0, 0)
    Pose2d current_pose(0.0, 0.0, 0.0);

    double step_size = 0.5;  // Distance between points along the path

    // Calculate the differences between the current pose and the destination
    double dx = destination.pos_x - current_pose.pos_x;
    double dy = destination.pos_y - current_pose.pos_y;
    double dyaw = destination.yaw - current_pose.yaw;

    // Determine the number of steps 
    int steps = std::max(std::abs(dx / step_size), std::abs(dy / step_size));

    // Generate intermediate points along the path 
    for (int i = 1; i <= steps; ++i) {
        double x = current_pose.pos_x + (i * dx) / steps;
        double y = current_pose.pos_y + (i * dy) / steps;
        double yaw = current_pose.yaw + (i * dyaw) / steps;

        Pose2d intermediate_pose(x, y, yaw);

        // Create a NavNode for the intermediate pose with ActionType::normal
        NavNode node(ActionType::normal);
        node.pose = intermediate_pose;

        // Optionally, set flags for manual or final approach if needed
        if (i == steps) {
            node.is_final_approach = true;  // Mark the last node as the final approach
        }

        path.push_back(node);
    }

    // Add the final destination node with ActionType::normal
    NavNode destination_node(ActionType::normal);
    destination_node.pose = destination;
    destination_node.is_final_approach = true;  // Ensure this is marked as the final approach
    path.push_back(destination_node);

    if (this->debug_enable_logging) {
        RCLCPP_INFO(this->get_logger(), "Generated path to destination (%f, %f, %f):", 
                    destination.pos_x, destination.pos_y, destination.yaw);
        for (const auto& node : path) {
            RCLCPP_INFO(this->get_logger(), "Path point: (%f, %f, %f)", 
                        node.pose.pos_x, node.pose.pos_y, node.pose.yaw);
        }
    }

    return path;
}


bool TaskPlanner::load_locations_from_file() {
    /*
    shelf#2 (-2,2)    shelf#3 (2,2)

    shelf#1 (-2,-1)   shelf#4 (2,-1)
    */

    // Load station locations with paths generated from the center to the station
    station_locations[1] = Station(1, Pose2d(1, 1, 0), generatePathToStation(Pose2d(1, 1, 0)));
    station_locations[2] = Station(2, Pose2d(2, 1, 0), generatePathToStation(Pose2d(2, 1, 0)));
    station_locations[3] = Station(3, Pose2d(5, 1, 0), generatePathToStation(Pose2d(5, 1, 0)));

    station_locations[-1] = Station(-1, Pose2d(-2, -1, 0), generatePathToStation(Pose2d(-2, -1, 0)));
    station_locations[-2] = Station(-2, Pose2d(-2, 2, 0), generatePathToStation(Pose2d(-2, 2, 0)));
    station_locations[-3] = Station(-3, Pose2d(2, 2, 0), generatePathToStation(Pose2d(2, 2, 0)));
    station_locations[-4] = Station(-4, Pose2d(2, -1, 0), generatePathToStation(Pose2d(2, -1, 0)));

    product_locations[1] = -1;
    product_locations[2] = -2;
    product_locations[3] = -3;
    product_locations[4] = -4;

    return true;
}

// void TaskPlanner::tag_id_callback(const std_msgs::msg::Int32::SharedPtr msg) {
//     RCLCPP_INFO(this->get_logger(), "Received tag ID: %d", msg->data);
//     latest_detected_tag_ = msg->data;  // Store the detected tag ID
// }


//ros2 run apriltag_ros apriltag_node --ros-args   -r image_rect:=/camera/image_raw   -r camera_info:=/camera/camera_info   -p family:=36h11   -p size:=0.5   -p max_hamming:=0
bool TaskPlanner::get_visible_station_code(int& tag_id) {
    if (latest_detected_tag_) {  // Check if a tag ID was detected
        tag_id = *latest_detected_tag_;  // Dereference the optional value
        RCLCPP_INFO(this->get_logger(), "Detected station tag ID: %d", tag_id);
        latest_detected_tag_.reset();  // Clear the stored tag after using it
        return true;
    } else {
        RCLCPP_WARN(this->get_logger(), "No station tag detected.");
        return false;
    }
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
                    // TODO @suraj Print a more descriptive error message, not error. This should not trigger, implies the job wasn't reset, or somehow had more than 4 phases
                    RCLCPP_WARN(this->get_logger(), "ERROR");
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
                break;
            }
        }
    }

    if (current_job_points_.front().is_final_approach) {
        int station_id = 0;
        if (!get_visible_station_code(station_id))
            return;

        // The apriltag doesn't match the expected value
        if (status == JobStatus::ToPickup && station_id != pickup_station_id) {
            RCLCPP_WARN(this->get_logger(), "Incorrect pickup location, task aborted");
            // TODO actually abort here
        }
        if (status == JobStatus::ToDestination && station_id != dropoff_station_id) {
            RCLCPP_WARN(this->get_logger(), "Incorrect dropoff location, task aborted");
            // TODO actually abort here
        }
    }
}

void TaskPlanner::set_activation_state(bool state) {
    this->is_active = state;
}
