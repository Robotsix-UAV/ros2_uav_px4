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

# Generate files from templates
set(MODES_RUNNER_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/src/templates/modes_runner.jinja")
set(MODES_SERVICE_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/src/templates/modes_service.jinja")
set(MISSION_EXECUTOR_TEMPLATE "${CMAKE_CURRENT_SOURCE_DIR}/src/templates/mission_executor.jinja")
set(MODES_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/px4_modes.yaml")
set(MISSION_EXECUTOR_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/config/mission.yaml")
set(MODES_RUNNER_CPP "${CMAKE_CURRENT_BINARY_DIR}/modes_runner.cpp")
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/srv)
set(MODES_MESSAGE "${CMAKE_CURRENT_BINARY_DIR}/srv/ModeSelector.srv")
set(MISSION_EXECUTOR_CPP "${CMAKE_CURRENT_BINARY_DIR}/mission_executor.cpp")

# Force reconfiguration of the project if the templates or the config files have changed
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
  ${MODES_CONFIG_FILE}
  ${MISSION_EXECUTOR_CONFIG_FILE}
  ${MODES_RUNNER_TEMPLATE}
  ${MODES_SERVICE_TEMPLATE}
  ${MISSION_EXECUTOR_TEMPLATE}
)

# Include the generation utility function
include(${CMAKE_CURRENT_SOURCE_DIR}/template_generator/GenerateUtils.cmake)

check_and_generate_if_needed(${MODES_CONFIG_FILE} ${MODES_MESSAGE} ${MODES_SERVICE_TEMPLATE})
check_and_generate_if_needed(${MODES_CONFIG_FILE} ${MODES_RUNNER_CPP} ${MODES_RUNNER_TEMPLATE})
check_and_generate_if_needed(${MISSION_EXECUTOR_CONFIG_FILE} ${MISSION_EXECUTOR_CPP} ${MISSION_EXECUTOR_TEMPLATE})

# Generate the ROS2 service for mode selection
rosidl_generate_interfaces(${PROJECT_NAME} ${CMAKE_CURRENT_BINARY_DIR}:srv/ModeSelector.srv)
rosidl_get_typesupport_target(ROS_SERVICE ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(modes_runner ${MODES_RUNNER_CPP})
target_include_directories(modes_runner PUBLIC include)

ament_target_dependencies(modes_runner px4_ros2_cpp ros2_uav_interfaces uav_cpp ros2_uav_parameters)
target_link_libraries(modes_runner
  ${ROS_SERVICE})

add_executable(disturbance_observer src/disturbance_observer.cpp)
target_include_directories(disturbance_observer PUBLIC include)
ament_target_dependencies(disturbance_observer px4_ros2_cpp ros2_uav_interfaces uav_cpp ros2_uav_parameters)

add_executable(mission_executor ${MISSION_EXECUTOR_CPP})
ament_target_dependencies(mission_executor ros2_uav_interfaces rclcpp uav_cpp)
target_link_libraries(mission_executor ${ROS_SERVICE})

add_executable(thrust_matcher src/thrust_matcher.cpp src/model_identification/thrust_matcher_node.cpp)
target_include_directories(thrust_matcher PUBLIC include)
ament_target_dependencies(thrust_matcher uav_cpp rclcpp px4_msgs ros2_uav_interfaces ros2_uav_parameters)

install(TARGETS
  modes_runner
  disturbance_observer
  mission_executor
  thrust_matcher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
