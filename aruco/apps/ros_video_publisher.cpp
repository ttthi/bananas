#ifdef ENABLE_ROS2

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/header.hpp>

namespace {

const std::string node_name{"video_publisher"};
const std::string topic_name{"aruco_camera/image"};

} // namespace

int main(int argc, char *argv[]) {
    const auto args{rclcpp::init_and_remove_ros_arguments(argc, argv)};
    if (args.size() != 2) {
        std::cerr << "Expected 1 argument, got: "
                  << (args.empty() ? 0 : args.size() - 1) << '\n';
        return EXIT_FAILURE;
    }
    const std::string &video_file{args[1]};

    const auto node{
        rclcpp::Node::make_shared(node_name, rclcpp::NodeOptions{})};
    image_transport::ImageTransport it{node};
    auto publisher{it.advertise(topic_name, 1)};

    cv::VideoCapture capture{video_file};
    cv::Mat image{};
    rclcpp::Clock clock{};
    const rclcpp::Time start_time{clock.now()};
    while (rclcpp::ok()) {
        const bool got_frame{capture.read(image)};
        if (!got_frame) {
            break;
        }

        const rclcpp::Duration target_after_start{std::chrono::nanoseconds{
            1'000'000 *
            static_cast<std::int64_t>(capture.get(cv::CAP_PROP_POS_MSEC))}};
        clock.sleep_until(start_time + target_after_start);

        const auto message{
            cv_bridge::CvImage(
                std_msgs::msg::Header{},
                static_cast<const char *>(sensor_msgs::image_encodings::BGR8),
                image)
                .toImageMsg()};
        publisher.publish(message);
        rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}

#endif // ENABLE_ROS2
