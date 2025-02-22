cmake_minimum_required(VERSION 3.8)
project(amr_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(amr_custom_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_executable(random_mover src/random_mover.cpp)
ament_target_dependencies(random_mover rclcpp geometry_msgs)

add_executable(trajectory_publisher_saver src/trajectory_publisher_saver.cpp)
ament_target_dependencies(trajectory_publisher_saver rclcpp turtlesim visualization_msgs amr_custom_msgs)

add_executable(trajectory_reader_publisher src/trajectory_reader_publisher.cpp)
ament_target_dependencies(trajectory_reader_publisher rclcpp turtlesim visualization_msgs amr_custom_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  random_mover
  trajectory_publisher_saver
  trajectory_reader_publisher
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
