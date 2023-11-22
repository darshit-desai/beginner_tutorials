/**
 * @file publisher_member_function.cpp
 * @author Darshit Desai(darshit@umd.edu)
 * @brief An example of a publisher node that publishes a message to a topic
 * which can be changed by a service
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023 Darshit Desai
 * This code is licensed under the Apache 2.0 License. Please see the
 * accompanying LICENSE file for the full text of the license.
 *
 */

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <chrono>
#include <cpp_pubsub/srv/mod_output.hpp>
#include <cstdlib>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std::chrono_literals;

/**
 * @brief A simple publisher class that publishes a message to a topic
 * that can be changed by a service call
 *
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    this->declare_parameter("freq", 0.5);  // Default value is 0.5 Hz
    double publish_frequency = this->get_parameter("freq").as_double();
    // if frequency less then zero give error
    if (publish_frequency < 0) {
      RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger("minimal_publisher"),
                               "Frequency parameter must be greater than zero");
      exit(1);
    } else if (publish_frequency > 100) {
      // else if frequency >100 then give warning
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("minimal_publisher"),
                          "Frequency parameter is greater than 100 Hz");
    } else {
      // else start the publisher with a delay of 1/frequency and
      // print the debug stream
      RCLCPP_DEBUG_STREAM(
          rclcpp::get_logger("minimal_publisher"),
          "Frequency parameter is " << publish_frequency << " Hz");

      RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                         "Publishing at " << publish_frequency << " Hz");
    }
    publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_frequency),
        std::bind(&MinimalPublisher::timer_callback, this));
    service_ = this->create_service<cpp_pubsub::srv::ModOutput>(
        "change_output_server",
        [this](
            const std::shared_ptr<cpp_pubsub::srv::ModOutput::Request> request,
            std::shared_ptr<cpp_pubsub::srv::ModOutput::Response> response) {
          change_output_service_request(request, response);
        });
  }

 private:
  /**
   * @brief A callback function that publishes a message to a topic
   *
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = getData();
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "Publishing: " << message.data.c_str());
    publisher_->publish(message);
    broadcast_tf();
  }
  /**
   * @brief A callback function that changes the output string
   *
   * @param request
   * @param response
   */
  void change_output_service_request(
      const std::shared_ptr<cpp_pubsub::srv::ModOutput::Request> request,
      std::shared_ptr<cpp_pubsub::srv::ModOutput::Response> response) {
    RCLCPP_WARN_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "received request with data " << request->new_output);
    setData(request->new_output);
    response->set__success(true);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("minimal_publisher"),
                       "String updated to: " << base_output_string_);
    RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("minimal_publisher"),
                            "finished processing request");
  }
  /**
   * @brief A function that returns the data to be published
   *
   * @return std::string
   */
  std::string getData() {
    return base_output_string_ + " " + std::to_string(count_++);
  }
  /**
   * @brief A function that sets the data to be published
   *
   * @param data
   */
  void setData(const std::string& data) { base_output_string_ = data; }
  /**
   * @brief A function that broadcasts a transform
   *
   */
  void broadcast_tf() {
    static tf2_ros::StaticTransformBroadcaster broadcaster(this);
    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped.header.stamp = this->now();
    transformStamped.header.frame_id = "world";
    transformStamped.child_frame_id = "talk";
    transformStamped.transform.translation.x =
        1.0;  // Set your desired translation
    transformStamped.transform.rotation.x = 0.0;  // Set your desired rotation
    broadcaster.sendTransform(transformStamped);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<cpp_pubsub::srv::ModOutput>::SharedPtr service_;
  size_t count_;
  std::string base_output_string_ = "Hello, world!";
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
