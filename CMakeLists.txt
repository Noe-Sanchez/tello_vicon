cmake_minimum_required(VERSION 3.8)
project(tello_vicon)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_executable(asmc_node src/asmc.cpp)
ament_target_dependencies(asmc_node rclcpp std_msgs geometry_msgs)
add_executable(simple_dynamics_node src/simple_dynamics.cpp)
ament_target_dependencies(simple_dynamics_node rclcpp std_msgs geometry_msgs tf2_ros)
add_executable(vectorfviz_node src/vectorf_visualization.cpp)
ament_target_dependencies(vectorfviz_node rclcpp std_msgs geometry_msgs visualization_msgs)
add_executable(differentiator_node src/differentiator.cpp)
ament_target_dependencies(differentiator_node rclcpp std_msgs geometry_msgs tf2_ros)
add_executable(pid_node src/pid.cpp)
ament_target_dependencies(pid_node rclcpp std_msgs geometry_msgs)
add_executable(pid2_node src/pid2.cpp)
ament_target_dependencies(pid2_node rclcpp std_msgs geometry_msgs)
add_executable(formations_node src/formations.cpp)
ament_target_dependencies(formations_node rclcpp std_msgs geometry_msgs visualization_msgs tf2_ros)
add_executable(unity_ui_node src/unity_ui.cpp)
ament_target_dependencies(unity_ui_node rclcpp std_msgs geometry_msgs)
add_executable(asmc_avoidance_node src/asmc_avoidance.cpp)
ament_target_dependencies(asmc_avoidance_node rclcpp std_msgs geometry_msgs)
add_executable(traffic_control_node src/traffic_control.cpp)
ament_target_dependencies(traffic_control_node rclcpp std_msgs geometry_msgs)

add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node rclcpp std_msgs geometry_msgs)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  tello_vicon/tello_wrapper.py
  tello_vicon/reference.py
  tello_vicon/multi_reference.py
  tello_vicon/showtime.py
  tello_vicon/tello_joy.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  asmc_node
  asmc_avoidance_node
  simple_dynamics_node
  differentiator_node
  pid_node
  pid2_node
  formations_node
  test_node
  unity_ui_node
  vectorfviz_node
  traffic_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
