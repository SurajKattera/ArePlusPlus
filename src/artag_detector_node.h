#ifndef ARTAG_DETECTOR_NODE_H
#define ARTAG_DETECTOR_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

class ArtagDetectorNode : public rclcpp::Node {
public:
    ArtagDetectorNode();

    bool get_visible_station_code(int& tag_id);

private:
    // Callback functions
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

    // Helper function to draw tag detection on the image
    void draw_tag_on_image(const apriltag_msgs::msg::AprilTagDetection& detection);

    // Latest data storage
    cv::Mat current_image_;
    apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr latest_detection_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscription_;
};

#endif  // ARTAG_DETECTOR_NODE_H
