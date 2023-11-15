cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ModOutput.srv"
  DEPENDENCIES std_msgs
)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)

#add_executable(change_output_server src/change_output.cpp)
#ament_target_dependencies(change_output_server rclcpp std_msgs)

install(TARGETS
  talker
  listener
  #change_output_server
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
        launch
        DESTINATION share/${PROJECT_NAME}/
)

rosidl_target_interfaces(talker ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_target_interfaces(listener ${PROJECT_NAME} "rosidl_typesupport_cpp")
#rosidl_target_interfaces(change_output_server ${PROJECT_NAME} "rosidl_typesupport_cpp")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
