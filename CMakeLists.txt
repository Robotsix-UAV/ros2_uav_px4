cmake_minimum_required(VERSION 3.8)
project(ros2_uav_px4)

set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_definitions(-DCMAKE_EXPORT_COMPILE_COMMANDS=ON)

find_package(ament_cmake REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(ros2_uav_parameters REQUIRED)
find_package(ros2_uav_cpp REQUIRED)
find_package(ros2_uav_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # xmllint does not work on current package.xml
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(utils_test test/utils_test.cpp)
  ament_target_dependencies(utils_test ros2_uav_interfaces uav_cpp px4_ros2_cpp)
  target_include_directories(utils_test PUBLIC include)
endif()

# Generate files from templates
set(MODES_RUNNER_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/src/runner_template/modes_runner.jinja")
set(MODES_SERVICE_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/src/runner_template/modes_service.jinja")
set(MISSION_EXECUTOR_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/src/mission_template/mission_executor.jinja")
set(MODES_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/px4_modes.yaml")
set(MISSION_EXECUTOR_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/mission.yaml")
set(MODES_RUNNER_CPP "${CMAKE_CURRENT_BINARY_DIR}/modes_runner.cpp")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/srv)
set(MODES_MESSAGE "${CMAKE_CURRENT_BINARY_DIR}/srv/ModeSelector.srv")
set(MISSION_EXECUTOR_CPP "${CMAKE_CURRENT_BINARY_DIR}/mission_executor.cpp")

# Include the generation utility function
include(${CMAKE_CURRENT_SOURCE_DIR}/template_generator/GenerateUtils.cmake)

check_and_generate_if_needed(${MODES_CONFIG_FILE} ${MODES_MESSAGE} ${MODES_SERVICE_TEMPLATE})
check_and_generate_if_needed(${MODES_CONFIG_FILE} ${MODES_RUNNER_CPP} ${MODES_RUNNER_TEMPLATE})
check_and_generate_if_needed(${MISSION_EXECUTOR_CONFIG_FILE} ${MISSION_EXECUTOR_CPP} ${MISSION_EXECUTOR_TEMPLATE})

# Generate the ROS2 service for mode selection
rosidl_generate_interfaces(${PROJECT_NAME} ${CMAKE_CURRENT_BINARY_DIR}:srv/ModeSelector.srv)

add_executable(modes_runner ${MODES_RUNNER_CPP})
target_include_directories(modes_runner PUBLIC include)
ament_target_dependencies(modes_runner px4_ros2_cpp ros2_uav_interfaces uav_cpp tf2_ros)
target_link_libraries(modes_runner uav_cpp::pipelines ros2_uav_parameters::parameter_client ros2_uav_cpp::ros2_uav_logger)
rosidl_target_interfaces(modes_runner ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(mission_executor ${MISSION_EXECUTOR_CPP})
ament_target_dependencies(mission_executor ros2_uav_interfaces rclcpp)
rosidl_target_interfaces(mission_executor ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(thrust_matcher src/thrust_matcher.cpp src/model_identification/thrust_matcher_node.cpp)
target_include_directories(thrust_matcher PUBLIC include)
ament_target_dependencies(thrust_matcher uav_cpp rclcpp px4_msgs ros2_uav_interfaces ros2_uav_cpp)
target_link_libraries(thrust_matcher ros2_uav_parameters::parameter_client)

add_executable(tf_publisher src/tf_publisher.cpp)
ament_target_dependencies(tf_publisher rclcpp tf2_ros px4_msgs)

install(TARGETS
  modes_runner
  mission_executor
  thrust_matcher
  tf_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
