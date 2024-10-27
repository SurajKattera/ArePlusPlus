#include "artag_detector_node.h"

ArtagDetectorNode::ArtagDetectorNode() 
    : Node("artag_detector_node") {
    // Initialize subscriptions
    image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        std::bind(&ArtagDetectorNode::image_callback, this, std::placeholders::_1));

    apriltag_subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
        "/detections", 10,
        std::bind(&ArtagDetectorNode::apriltag_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "ARTag detector node started.");
}

void ArtagDetectorNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
    }
}

void ArtagDetectorNode::apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
    latest_detection_ = msg;  // Store the latest detections
}

bool ArtagDetectorNode::get_visible_station_code(int &tag_id) {
    // Ensure we have a valid image and detection message
    if (current_image_.empty()) {
        RCLCPP_WARN(this->get_logger(), "No image available.");
        return false;
    }

    if (!latest_detection_ || latest_detection_->detections.empty()) {
        RCLCPP_INFO(this->get_logger(), "No AR tags detected.");
        return false;
    }

    // Use the first detected tag for simplicity
    const auto &detection = latest_detection_->detections[0];
    tag_id = detection.id;

    RCLCPP_INFO(this->get_logger(), "Detected AR tag: ID = %d", tag_id);

    // Draw the detected tag on the image
    draw_tag_on_image(detection);

    // Display the annotated image (optional)
    cv::imshow("Tag Detection", current_image_);
    cv::waitKey(1);

    return true;
}

void ArtagDetectorNode::draw_tag_on_image(const apriltag_msgs::msg::AprilTagDetection& detection) {
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
}
