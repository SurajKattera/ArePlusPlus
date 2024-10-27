#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "std_msgs/msg/int32.hpp"  // Include for publishing tag ID

class ArtagDetectorNode : public rclcpp::Node {
public:
    ArtagDetectorNode() : Node("artag_detector_node") {
        // Subscriptions
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 10, 
            std::bind(&ArtagDetectorNode::image_callback, this, std::placeholders::_1));

        apriltag_subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
            "/detections", 10, 
            std::bind(&ArtagDetectorNode::apriltag_callback, this, std::placeholders::_1));

        // Publishers
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_rect", 10);
        tag_id_publisher_ = this->create_publisher<std_msgs::msg::Int32>("detected_tag_id", 10);  // Tag ID publisher

        RCLCPP_INFO(this->get_logger(), "ARTag detector node started.");
    }

private:
    cv::Mat current_image_;  // Store the latest image

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS Image message to OpenCV image
            current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Display the image
            cv::imshow("TurtleBot Camera", current_image_);
            cv::waitKey(1);

            // Publish the image to /camera/image_rect
            image_publisher_->publish(*msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
    }

    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image received yet.");
            return;
        }

        if (msg->detections.empty()) {
            RCLCPP_INFO(this->get_logger(), "No AR tags detected.");
            return;
        }

        // Process each detected tag
        for (const auto& detection : msg->detections) {
            std::vector<cv::Point> corners;
            for (const auto& corner : detection.corners) {
                corners.emplace_back(corner.x, corner.y);
            }

            // Draw lines between the corners
            for (size_t i = 0; i < corners.size(); ++i) {
                cv::line(current_image_, corners[i], corners[(i + 1) % corners.size()], 
                         cv::Scalar(0, 255, 0), 2);  // Green lines
            }

            // Display the tag ID at the center
            cv::Point center(detection.centre.x, detection.centre.y);
            cv::putText(current_image_, std::to_string(detection.id), center, 
                        cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);  // Red text

            RCLCPP_INFO(this->get_logger(), "Detected AR tag: ID = %d", detection.id);

            // Publish the detected tag ID
            std_msgs::msg::Int32 tag_id_msg;
            tag_id_msg.data = detection.id;
            tag_id_publisher_->publish(tag_id_msg);
        }

        // Display the image with the wireframes
        cv::imshow("TurtleBot Camera", current_image_);
        cv::waitKey(1);
    }

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscription_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr tag_id_publisher_;  // Publisher for tag ID
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArtagDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
