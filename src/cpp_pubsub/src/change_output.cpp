// #include <rclcpp/rclcpp.hpp>
// #include <cpp_pubsub/srv/mod_output.hpp>

// class ChangeOutputServer : public rclcpp::Node {
// public:
//   ChangeOutputServer() : Node("change_output_server") {
//     auto callback = [this](const std::shared_ptr<rmw_request_id_s> request_header,
//                        const std::shared_ptr<cpp_pubsub::srv::ModOutput_Request_<std::allocator<void>>> request,
//                        std::shared_ptr<cpp_pubsub::srv::ModOutput_Response_<std::allocator<void>>> response) {
//   this->handle_service_request(request_header, request, response);
// };
//   auto custom_callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
//   auto service = create_service<cpp_pubsub::srv::ModOutput>(
//   "change_output_server", std::move(callback), rmw_qos_profile_services_default, custom_callback_group);

//   }

// private:
//   void handle_service_request(
//       const std::shared_ptr<rmw_request_id_t> request_header,
//       const std::shared_ptr<cpp_pubsub::srv::ModOutput::Request> request,
//       const std::shared_ptr<cpp_pubsub::srv::ModOutput::Response> response) {
//     // Modify the base output string based on the request
//     base_output_string_ = request->new_output;
//     RCLCPP_INFO(this->get_logger(), "Base output string changed to: '%s'", base_output_string_.c_str());
//     response->success = true;
//   }

//   rclcpp::Service<cpp_pubsub::srv::ModOutput>::SharedPtr service_;
//   std::string base_output_string_ = "Hello, world!";
// };

// int main(int argc, char *argv[]) {
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<ChangeOutputServer>());
//   rclcpp::shutdown();
//   return 0;
// }

#include "rclcpp/rclcpp.hpp"
#include "cpp_pubsub/srv/mod_output.hpp"  // Update this to match your service definition

#include <memory>

void modifyOutput(const std::shared_ptr<cpp_pubsub::srv::ModOutput::Request> request,
                   std::shared_ptr<cpp_pubsub::srv::ModOutput::Response> response)
{
  // Modify the response based on your application logic
  // For example, indicate success and log the new output
  response->success = true;

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nNew Output: %s",
               request->new_output.c_str());
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Change successful: [%d]", response->success);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  
  // Create a Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("change_output_server");

  // Create a service for your custom service type
  rclcpp::Service<cpp_pubsub::srv::ModOutput>::SharedPtr service =
    node->create_service<cpp_pubsub::srv::ModOutput>("change_output_server", &modifyOutput);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to change the output.");

  // Spin the node
  rclcpp::spin(node);

  // Shutdown when the node is done
  rclcpp::shutdown();

  return 0;
}
