/**
 * @file cylinder_detection_node.cpp
 * @brief ROS2 Node for detecting 30 cm diameter cylinders using laser scan data.
 *
 * This node subscribes to laser scan data and analyzes it to detect cylindrical objects with a diameter of 30 cm.
 * If a cylinder is detected, the node publishes a blue point marker for each cylinder to visualize the detected
 * positions using Rviz or another visualization tool. The cylinder detection algorithm looks for clusters of points
 * in the laser scan data that match the expected diameter of the cylinder.
 *
 * Subscribed Topic:
 * - /scan (sensor_msgs::msg::LaserScan): Laser scan data from a LIDAR sensor.
 *
 * Published Topic:
 * - /cylinder_markers (visualization_msgs::msg::Marker): Visualization markers representing detected cylinders.
 *
 * @version 1.0
 * @date 2024-10-09
 */

#include <memory>
#include <vector>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>

/**
 * @class Cylinder
 * @brief A ROS2 Node that detects 30 cm diameter cylinders in laser scan data and visualizes them as blue points.
 */
class Cylinder : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the Cylinder class.
     *
     * Initializes the ROS2 node, sets up subscriptions and publishers.
     */
    Cylinder()
        : Node("cylinder_detection")
    {
        // Subscribe to the LaserScan topic
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Cylinder::scanCallback, this, std::placeholders::_1));

        // Publisher for visualization markers (to visualize detected cylinders)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/cylinder_markers", 10);
    }

private:
    /**
     * @brief Callback function for processing incoming laser scan data.
     *
     * This function analyzes the laser scan data to detect cylinders with a 30 cm diameter.
     * Detected cylinder positions are stored and passed to the marker publisher.
     *
     * @param scan Shared pointer to the incoming laser scan message.
     */
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // Parameters for cylinder detection
        const float cylinder_diameter = 0.30;  // Cylinder diameter in meters
        const float cylinder_radius = cylinder_diameter / 2.0;
        const float tolerance = 0.05;  // Tolerance in the radius (adjustable)

        // A list to hold the detected cylinder positions
        std::vector<std::pair<float, float>> detected_cylinders;

        // Loop through the scan data to find cylindrical clusters
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            float range = scan->ranges[i];

            // Check if the range is valid (not NaN or Inf and within the sensor's max range)
            if (std::isfinite(range))
            {
                // Calculate the expected range for the cylinder's radius
                float next_range = scan->ranges[(i + 1) % scan->ranges.size()];

                // Detect a cluster of points forming a circle with the expected radius
                if (std::abs(range - next_range) < tolerance && std::abs(range - cylinder_radius) < tolerance)
                {
                    // Calculate the position of the cylinder in Cartesian coordinates
                    float angle = scan->angle_min + i * scan->angle_increment;
                    float x = range * std::cos(angle);
                    float y = range * std::sin(angle);

                    // Add detected cylinder coordinates to the list
                    detected_cylinders.emplace_back(x, y);
                }
            }
        }

        // Publish markers for the detected cylinders
        publishCylinders(detected_cylinders);
    }

    /**
     * @brief Publishes the detected cylinders as blue points using markers.
     *
     * This method creates a marker message containing the positions of all detected cylinders
     * and publishes it for visualization in Rviz or another visualization tool.
     *
     * @param cylinders A vector of pairs representing the (x, y) positions of detected cylinders.
     */
    void publishCylinders(const std::vector<std::pair<float, float>> &cylinders)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";  // Use the appropriate frame id
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "cylinder_detection";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;  // Size of the points
        marker.scale.y = 0.05;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;  // Blue color for the cylinder points
        marker.color.a = 1.0;  // Fully opaque

        // Add points for each detected cylinder
        for (const auto &cylinder : cylinders)
        {
            geometry_msgs::msg::Point point;
            point.x = cylinder.first;
            point.y = cylinder.second;
            point.z = 0.0;  // Assume the cylinder is on the ground plane
            marker.points.push_back(point);
        }

        // Publish the marker
        marker_pub_->publish(marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; ///< Subscription to LaserScan data
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_; ///< Publisher for cylinder visualization markers
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cylinder>());
    rclcpp::shutdown();
    return 0;
}
