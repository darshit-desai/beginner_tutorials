/**
 * @file integration_test.cpp
 * @author Darshit Desai (darshit@umd.edu)
 * @brief Level 2 integration test for the cpp_pubsub package
 * @version 0.1
 * @date 2023-11-19
 *
 * @copyright Copyright (c) 2023 Darshit Desai
 * This code is licensed under the Apache 2.0 License. Please see the
 * accompanying LICENSE file for the full text of the license.
 *
 */

#include <gtest/gtest.h>
#include <stdlib.h>

#include <cpp_pubsub/srv/mod_output.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
/**
 * @brief Class for testing the cpp_pubsub package
 *
 */
class TaskPlanningFixture : public testing::Test {
 public:
  TaskPlanningFixture() : node_(std::make_shared<rclcpp::Node>("basic_test")) {
    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH CONSTRUCTOR!!");
  }
  /**
   * @brief Destroy the Task Planning Fixture object
   *
   */
  void SetUp() override {
    // Setup things that should occur before every test instance should go here

    /*
     * 1.) Define any ros2 package and exectuable you want to test
     *  example: package name = cpp_pubsub, node name = minimal_publisher,
     * executable = talker
     */
    bool retVal = StartROSExec("cpp_pubsub", "minimal_publisher", "talker");
    ASSERT_TRUE(retVal);

    RCLCPP_INFO_STREAM(node_->get_logger(), "DONE WITH SETUP!!");
  }
  /**
   * @brief Destroy the Task Planning Fixture object
   *
   */
  void TearDown() override {
    // Tear things that should occur after every test instance should go here

    // Stop the running ros2 node, if any.
    bool retVal = StopROSExec();
    ASSERT_TRUE(retVal);

    std::cout << "DONE WITH TEARDOWN" << std::endl;
  }

 protected:
  rclcpp::Node::SharedPtr node_;
  std::stringstream cmd_ss, cmdInfo_ss, killCmd_ss;
  /**
   * @brief Method to start the ros2 node
   *
   * @param pkg_name
   * @param node_name
   * @param exec_name
   * @return true
   * @return false
   */
  bool StartROSExec(const char* pkg_name, const char* node_name,
                    const char* exec_name) {
    // build command strings
    cmd_ss << "ros2 run " << pkg_name << " " << exec_name
           << " > /dev/null 2> /dev/null &";
    cmdInfo_ss << "ros2 node info "
               << "/" << node_name << " > /dev/null 2> /dev/null";
    char execName[16];
    // snprintf(execName, 16, "%s",
    //  exec_name);  // pkill uses exec name <= 15 char only
    snprintf(execName, sizeof(execName), "%s", exec_name);
    killCmd_ss << "pkill --signal SIGINT " << execName
               << " > /dev/null 2> /dev/null";

    // First kill the ros2 node, in case it's still running.
    StopROSExec();

    // Start a ros2 node and wait for it to get ready:
    int retVal = system(cmd_ss.str().c_str());
    if (retVal != 0) return false;

    retVal = -1;
    while (retVal != 0) {
      retVal = system(cmdInfo_ss.str().c_str());
      sleep(1);
    }
    return true;
  }
  /**
   * @brief Method to stop the running ros2 node
   *
   * @return true
   * @return false
   */
  bool StopROSExec() {
    if (killCmd_ss.str().empty()) return true;

    int retVal = system(killCmd_ss.str().c_str());
    return retVal == 0;
  }
};

/**
 * @brief Test case to check if the node is running
 *
 */
TEST_F(TaskPlanningFixture, TrueIsTrueTest) {
  std::cout << "TEST BEGINNING!!" << std::endl;
  EXPECT_TRUE(true);

  /*
   * 2.) subscribe to the topic
   */
  using std_msgs::msg::String;
  using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;
  bool hasData = false;
  SUBSCRIBER subscription = node_->create_subscription<String>(
      "chatter", 10,
      // Lambda expression begins
      [&](const std_msgs::msg::String& msg) {
        RCLCPP_INFO(node_->get_logger(), "I heard: '%s'", msg.data.c_str());
        hasData = true;
      });

  /*
   * 3.) check to see if we get data winhin 3 sec
   */
  using timer = std::chrono::system_clock;
  using namespace std::chrono_literals;
  timer::time_point clock_start;
  timer::duration elapsed_time;
  clock_start = timer::now();
  elapsed_time = timer::now() - clock_start;
  rclcpp::Rate rate(2.0);  // 2hz checks
  while (elapsed_time < 3s) {
    rclcpp::spin_some(node_);
    rate.sleep();
    elapsed_time = timer::now() - clock_start;
  }
  EXPECT_TRUE(hasData);
}
/**
 * @brief Test case to check if the node is running and the service call is
 * working
 *
 */
TEST_F(TaskPlanningFixture, Assert_check_test) {
  // @TODO : Write a test case that uses the ASSERT_* macros
  // Example: Call the service to change the output string
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  // Subscribe to the chatter topic
  using std_msgs::msg::String;
  using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;
  SUBSCRIBER subscription;  // Declare the subscription outside the block

  // Call the service to change the output string
  auto client =
      node_->create_client<cpp_pubsub::srv::ModOutput>("change_output_server");
  ASSERT_TRUE(client->wait_for_service(std::chrono::seconds(1)));

  auto request = std::make_shared<cpp_pubsub::srv::ModOutput::Request>();
  request->new_output = "NewOutputString";

  auto result_future = client->async_send_request(request);
  RCLCPP_INFO(node_->get_logger(),
              "Calling service to change output string...");

  if (executor.spin_until_future_complete(result_future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto result = result_future.get();
    ASSERT_TRUE(result->success);

    // Subscribe to the chatter topic inside the if block
    subscription = node_->create_subscription<String>(
        "chatter", 10,
        // Lambda expression begins
        [&](const std_msgs::msg::String& msg) {
          RCLCPP_INFO(node_->get_logger(), "Received message: '%s'",
                      msg.data.c_str());
        });

    // Allow the subscription callback to execute for a certain duration
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout_duration = std::chrono::seconds(5);

    while (std::chrono::steady_clock::now() - start_time < timeout_duration) {
      executor.spin_once(std::chrono::milliseconds(100));
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(node_->get_logger(),
                "Finished waiting for subscription callback");
  } else {
    FAIL() << "Service call failed";
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  std::cout << "DONE SHUTTING DOWN ROS" << std::endl;
  return result;
}
