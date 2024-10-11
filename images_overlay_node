#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>

class MapOverlay : public rclcpp::Node {
public:
    MapOverlay() : Node("map_overlay") {

      // Load the images
        cv::Mat img1 = cv::imread("/home/student/ros2_ws/map.pgm");
        cv::Mat img2 = cv::imread("/home/student/ros2_ws/slam_generated_map.pgm");

        if (img1.empty() || img2.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "Could not open or find the images!");
            return;
        }

        // Ensure both images are of the same size
        if (img1.size() != img2.size())
        {    
            cv::resize(img2, img2, img1.size()); // Resize img2 to match img1's size if they are not the same size
        }

        // Create an output image to hold the overlay
        cv::Mat output;

        // Overlay the images (change the alpha value to adjust the transparency)
        double alpha = 0.5;  // Weight of the first image
        double beta = 1.0 - alpha;  // Weight of the second image
        cv::addWeighted(img1, alpha, img2, beta, 0.0, output);

        // Display the result
        cv::imshow("Overlayed Maps", output);
        cv::waitKey(0);  // Wait for a key press

        // Clean up
        cv::destroyAllWindows();
        
    }


};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOverlay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
