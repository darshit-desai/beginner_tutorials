/**
 * @file publisher_member_function.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief A simple publisher node that publishes a message to a topic
 * @version 0.1
 * @date 2023-11-07
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cpp_pubsub/srv/mod_output.hpp>

using namespace std::chrono_literals;

/**
 * @brief A simple publisher class that publishes a message to a topic by creating a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer.
 * 
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher()
      : Node("minimal_publisher"), count_(0) {
    this->declare_parameter("freq", 0.5);  // Default value is 0.5 Hz
    double publish_frequency = this->get_parameter("freq").as_double();
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_frequency),
    std::bind(&MinimalPublisher::timer_callback, this));
    service_ = this->create_service<cpp_pubsub::srv::ModOutput>(
            "change_output_server",
            [this](const std::shared_ptr<cpp_pubsub::srv::ModOutput::Request> request,
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
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  void change_output_service_request(const std::shared_ptr<cpp_pubsub::srv::ModOutput::Request> request,
                         std::shared_ptr<cpp_pubsub::srv::ModOutput::Response> response) {
        RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"), "received request with data " << request->new_output);
        setData(request->new_output);
        response->set__success(true);
        RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"), "finished processing request");
    }
  std::string getData() {
    return base_output_string_ + " " + std::to_string(count_++);
  }
  void setData(std::string data) {
    base_output_string_ = data;
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