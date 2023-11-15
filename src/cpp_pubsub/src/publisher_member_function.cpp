/**
 * @file publisher_member_function.cpp
 * @author Darshit Desai(darshit@umd.edu)
 * @brief An example of a publisher node that publishes a message to a topic
 * which can be changed by a service
 * @version 0.1
 * @date 2023-11-14
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <cpp_pubsub/srv/mod_output.hpp>
#include <cstdlib>
#include <functional>
#include <memory>
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
      RCLCPP_FATAL(this->get_logger(),
                   "Frequency parameter must be greater than zero");
      exit(1);
    }
    // else if frequency >100 then give warning
    else if (publish_frequency > 100) {
      RCLCPP_ERROR(this->get_logger(),
                   "Frequency parameter is greater than 100 Hz");
    }
    // else start the publisher with a delay of 1/frequency and print the debug
    // stream
    else {
      RCLCPP_DEBUG(this->get_logger(), "Frequency parameter is %f Hz",
                   publish_frequency);
      RCLCPP_INFO(this->get_logger(), "Publishing at %f Hz", publish_frequency);
    }
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
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
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Publishing: " << message.data.c_str());
    publisher_->publish(message);
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
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "received request with data %s",
                request->new_output.c_str());
    setData(request->new_output);
    response->set__success(true);
    RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"),
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
