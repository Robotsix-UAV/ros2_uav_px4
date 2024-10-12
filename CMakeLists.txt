cmake_minimum_required(VERSION 3.8)
project(ros2_uav_px4)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

find_package(ament_cmake REQUIRED)
find_package(ros2_uav_parameters REQUIRED)
find_package(ros2_uav_interfaces REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # xmllint does not work on current package.xml
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

add_executable(controller src/controller.cpp src/px4_interface/px4_comm.cpp)
target_include_directories(controller PUBLIC include)
ament_target_dependencies(controller ros2_uav_interfaces uav_cpp ros2_uav_parameters px4_msgs)

add_executable(disturbance_observer src/disturbance_observer.cpp)
target_include_directories(disturbance_observer PUBLIC include)
ament_target_dependencies(disturbance_observer ros2_uav_interfaces uav_cpp ros2_uav_parameters px4_msgs)

add_executable(thrust_matcher src/thrust_matcher.cpp src/model_identification/thrust_matcher_node.cpp)
target_include_directories(thrust_matcher PUBLIC include)
ament_target_dependencies(thrust_matcher uav_cpp rclcpp px4_msgs ros2_uav_interfaces ros2_uav_parameters)

install(TARGETS
  controller
  disturbance_observer
  thrust_matcher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
