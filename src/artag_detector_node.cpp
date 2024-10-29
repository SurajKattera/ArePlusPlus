#include "artag_detector_node.h"

ArtagDetectorNode::ArtagDetectorNode(rclcpp::Node* node) : Node("artag_detector_node") {
    image_subscription_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, 
        std::bind(&ArtagDetectorNode::image_callback, this, std::placeholders::_1));

    apriltag_subscription_ = node->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10, 
        std::bind(&ArtagDetectorNode::apriltag_callback, this, std::placeholders::_1));

    image_publisher_ = node->create_publisher<sensor_msgs::msg::Image>("/camera/image_rect", 10);
    tag_id_publisher_ = node->create_publisher<std_msgs::msg::Int32>("detected_tag_id", 10);

    RCLCPP_INFO(this->get_logger(), "ARTag detector node started.");
}

bool ArtagDetectorNode::get_last_seen_tag(int& tag_id) {
    if (got_tag) tag_id = last_tag;
    return got_tag;
}

void ArtagDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
        //cv::imshow("TurtleBot Camera", current_image_);
        //cv::waitKey(1);
        image_publisher_->publish(*msg);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
    }
}

void ArtagDetectorNode::apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    if (current_image_.empty()) {
        //RCLCPP_WARN(this->get_logger(), "No image received yet.");
        got_tag = false;
        return;
    }

    if (msg->detections.empty()) {
        got_tag = false;
        //RCLCPP_INFO(this->get_logger(), "No AR tags detected.");
        return;
    }

    for (const auto& detection : msg->detections) {
        std::vector<cv::Point> corners;
        for (const auto& corner : detection.corners) {
            corners.emplace_back(corner.x, corner.y);
        }

        for (size_t i = 0; i < corners.size(); ++i) {
            cv::line(current_image_, corners[i], corners[(i + 1) % corners.size()], 
                     cv::Scalar(0, 255, 0), 2);
        }

        cv::Point center(detection.centre.x, detection.centre.y);
        cv::putText(current_image_, std::to_string(detection.id), center, 
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);

        //RCLCPP_INFO(this->get_logger(), "Detected AR tag: ID = %d", detection.id);

        std_msgs::msg::Int32 tag_id_msg;
        tag_id_msg.data = detection.id;
        tag_id_publisher_->publish(tag_id_msg);
        got_tag = true;
        last_tag = detection.id;
    }

    //cv::imshow("TurtleBot Camera", current_image_);
    //cv::waitKey(1);
}

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ArtagDetectorNode>());
//     rclcpp::shutdown();
//     return 0;
// }
