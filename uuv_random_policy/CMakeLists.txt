cmake_minimum_required(VERSION 3.8)
project(uuv_random_policy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)

# Executable per la Random Policy
add_executable(random_node src/random_policy.cpp)
ament_target_dependencies(random_node rclcpp sensor_msgs std_msgs geometry_msgs) 

# Executable per la VFH Policy (Training Base)
add_executable(vfh_node src/vfh_node.cpp)
ament_target_dependencies(vfh_node rclcpp sensor_msgs std_msgs nav_msgs tf2 tf2_geometry_msgs ros_gz_interfaces) 

# Executable per il Multi-Agent VFH Policy (Test Swarm)
add_executable(vfh_multi_node src/vfh_multi_node.cpp)
ament_target_dependencies(vfh_multi_node rclcpp sensor_msgs std_msgs nav_msgs tf2 tf2_geometry_msgs ros_gz_interfaces) 

# Executable per il Machine Learning (PPO)
add_executable(nn_executor_node src/nn_executor_node.cpp)
ament_target_dependencies(nn_executor_node rclcpp std_msgs)

install(TARGETS
  random_node
  vfh_node
  vfh_multi_node
  nn_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
