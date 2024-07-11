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
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(utils_test test/utils_test.cpp)
  ament_target_dependencies(utils_test ros2_uav_interfaces uav_cpp px4_ros2_cpp)
  target_include_directories(utils_test PUBLIC include)
endif()

add_executable(modes_runner src/modes_runner.cpp)
target_include_directories(modes_runner PUBLIC include)
ament_target_dependencies(modes_runner px4_ros2_cpp ros2_uav_interfaces uav_cpp tf2_ros)
target_link_libraries(modes_runner uav_cpp::modes ros2_uav_parameters::parameter_client ros2_uav_cpp::ros2_uav_logger)

install(TARGETS
  modes_runner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
