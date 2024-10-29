#ifndef ARTAG_DETECTOR_NODE_H
#define ARTAG_DETECTOR_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"
#include <atomic>

class ArtagDetectorNode : public rclcpp::Node {
public:
    ArtagDetectorNode(rclcpp::Node* node);
    bool get_last_seen_tag(int& tag_id);
private:
    std::atomic<bool> got_tag = false;
    std::atomic<int> last_tag = 0;
    cv::Mat current_image_;
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tag_id_publisher_;
};

#endif // ARTAG_DETECTOR_NODE_H
