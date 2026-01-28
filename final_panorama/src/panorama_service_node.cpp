#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching.hpp>

#include <memory>
#include <string>
#include <vector>
#include <filesystem>
#include <thread>
#include <mutex>
#include <cmath>

namespace fs = std::filesystem;

class PanoramaServiceNode : public rclcpp::Node
{
public:
    PanoramaServiceNode() : Node("panorama_service_node")
    {
        // --- PARAMETERS ---
        // 'divide_images': The logic you provided to split images into 3 chunks
        this->declare_parameter("divide_images", false); 
        this->declare_parameter("mode", "panorama"); 
        // Default to standard laptop webcam topic
        this->declare_parameter("image_topic", "/image_raw"); 

        std::string img_topic = this->get_parameter("image_topic").as_string();

        // --- CALLBACK GROUP ---
        // CRITICAL: Reentrant allows the service to 'wait' while subscribers still receive data
        callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = callback_group_;

        // --- SUBSCRIBERS ---
        // 1. Angle Subscriber
        angle_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/servo_angle", 
            10, 
            std::bind(&PanoramaServiceNode::angle_callback, this, std::placeholders::_1),
            sub_opt);

        // 2. Camera Subscriber
        RCLCPP_INFO(this->get_logger(), "Subscribing to camera: %s", img_topic.c_str());
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            img_topic, 
            10, 
            std::bind(&PanoramaServiceNode::image_callback, this, std::placeholders::_1),
            sub_opt);

        // --- SERVICE ---
        service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_panorama",
            std::bind(&PanoramaServiceNode::panorama_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        // Init State
        current_angle_ = -999.0;
        has_image_ = false;

        RCLCPP_INFO(this->get_logger(), "Panorama Service Ready. Waiting for trigger...");
    }

private:
    // ROS Components
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    // Thread Safety
    std::mutex image_mutex_;
    std::mutex angle_mutex_;
    
    // State
    cv::Mat latest_image_;
    bool has_image_;
    double current_angle_;
    
    // File Paths
    const std::string temp_dir_ = "/tmp/panorama_temp/";
    const std::string final_output_ = "final_panorama_stitched.jpg";

    // --- CALLBACKS ---

    void angle_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(angle_mutex_);
        current_angle_ = msg->data;
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            std::lock_guard<std::mutex> lock(image_mutex_);
            latest_image_ = cv_ptr->image;
            has_image_ = true;
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge error: %s", e.what());
        }
    }

    // --- MAIN SERVICE LOGIC ---
    void panorama_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "--- Service Triggered ---");

        // 1. Initial Data Check
        double start_angle = 0.0;
        {
            std::lock_guard<std::mutex> lock(angle_mutex_);
            if (current_angle_ == -999.0) {
                response->success = false;
                response->message = "No angle data! Please publish to /servo_angle first.";
                RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
                return;
            }
            start_angle = current_angle_;
        }

        // 2. Prepare Temp Directory (Delete old, create new)
        if (fs::exists(temp_dir_)) fs::remove_all(temp_dir_);
        fs::create_directory(temp_dir_);

        std::vector<std::string> captured_files;
        RCLCPP_INFO(this->get_logger(), "Starting Sequence. Initial Angle: %.2f", start_angle);

        // 3. THE LOOP: 0, 20, 40 ... 180
        // We need 10 steps to cover 180 degrees (0 inclusive)
        for (int i = 0; i < 10; ++i) {
            double target_angle = start_angle + (i * 20.0);
            
            RCLCPP_INFO(this->get_logger(), "Step %d/10: Waiting for angle %.2f...", i+1, target_angle);

            // --- BLOCKING WAIT FOR ANGLE ---
            bool reached_target = false;
            while (!reached_target && rclcpp::ok()) {
                double current_val;
                {
                    std::lock_guard<std::mutex> lock(angle_mutex_);
                    current_val = current_angle_;
                }

                // Tolerance +/- 2.0 degrees
                if (std::abs(current_val - target_angle) <= 2.0) {
                    reached_target = true;
                } else {
                    // Sleep to let other threads run (IMPORTANT)
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            }
            
            // Wait a moment for camera to stabilize after movement stops
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            // --- CAPTURE ---
            cv::Mat temp_img;
            {
                std::lock_guard<std::mutex> lock(image_mutex_);
                if (has_image_) temp_img = latest_image_.clone();
            }

            if (!temp_img.empty()) {
                std::string filename = temp_dir_ + "img_" + std::to_string(i) + ".jpg";
                cv::imwrite(filename, temp_img);
                captured_files.push_back(filename);
                RCLCPP_INFO(this->get_logger(), "Captured Image @ %.2f deg", target_angle);
            } else {
                RCLCPP_WARN(this->get_logger(), "Camera frame empty!");
            }
        }

        // 4. STITCHING
        RCLCPP_INFO(this->get_logger(), "180 degrees reached. Stitching %zu images...", captured_files.size());
        
        cv::Mat final_result = stitch_images_custom(captured_files);

        // 5. SAVE & CLEANUP
        if (!final_result.empty()) {
            cv::imwrite(final_output_, final_result);
            response->success = true;
            response->message = "Panorama saved successfully to " + final_output_;
            RCLCPP_INFO(this->get_logger(), "SUCCESS: %s", response->message.c_str());
        } else {
            response->success = false;
            response->message = "Stitching failed.";
        }

        RCLCPP_INFO(this->get_logger(), "Cleaning up temporary images...");
        fs::remove_all(temp_dir_);
    }

    // --- YOUR CUSTOM STITCHING LOGIC ---
    cv::Mat stitch_images_custom(const std::vector<std::string>& file_paths)
    {
        bool divide_images = this->get_parameter("divide_images").as_bool();
        std::string mode_str = this->get_parameter("mode").as_string();

        cv::Stitcher::Mode mode = cv::Stitcher::PANORAMA;
        if (mode_str == "scans") mode = cv::Stitcher::SCANS;

        std::vector<cv::Mat> imgs;

        for (const auto& path : file_paths) {
            cv::Mat img = cv::imread(path);
            if (img.empty()) continue;

            if (divide_images) {
                // YOUR SPLITTING LOGIC: 
                // "internally creates three chunks of each image to increase stitching success"
                cv::Rect rect(0, 0, img.cols / 2, img.rows);
                imgs.push_back(img(rect).clone());
                
                rect.x = img.cols / 3;
                imgs.push_back(img(rect).clone());
                
                rect.x = img.cols / 2;
                imgs.push_back(img(rect).clone());
            } else {
                imgs.push_back(img);
            }
        }

        if (imgs.empty()) return cv::Mat();

        cv::Mat pano;
        cv::Ptr<cv::Stitcher> stitcher = cv::Stitcher::create(mode);
        cv::Stitcher::Status status = stitcher->stitch(imgs, pano);

        if (status != cv::Stitcher::OK) {
            RCLCPP_ERROR(this->get_logger(), "Stitching Error Code: %d", int(status));
            return cv::Mat();
        }

        return pano;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PanoramaServiceNode>();
    
    // CRITICAL: Must use MultiThreadedExecutor so the service can block 
    // while the subscribers continue to update angle/image data.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
