cmake_minimum_required(VERSION 3.8)
project(moveit_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


add_executable(moveit_node src/moveit_node.cpp)
add_executable(dual_ur_inspire_node src/dual_ur_inspire_node.cpp)
add_executable(test_ros2bridge src/test_ros2bridge.cpp)
add_executable(tcp_server_node src/tcp_server_node.cpp)
add_executable(joint_state_filter src/joint_state_filter.cpp)
add_executable(hands_node src/hands_node.cpp)
add_executable(left_arm_node src/left_arm_node.cpp)
add_executable(right_arm_node src/right_arm_node.cpp)

target_include_directories(moveit_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_node
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(dual_ur_inspire_node rclcpp 
                            sensor_msgs
                            rclcpp_action
                            control_msgs
                            trajectory_msgs
                            moveit_ros_planning_interface
                            )

ament_target_dependencies(left_arm_node rclcpp 
                            rclcpp_action
                            moveit_ros_planning_interface
                            )
ament_target_dependencies(right_arm_node rclcpp 
                            rclcpp_action
                            moveit_ros_planning_interface
                            )

ament_target_dependencies(hands_node rclcpp 
                          sensor_msgs
                          )

ament_target_dependencies(joint_state_filter rclcpp 
                            sensor_msgs
                          )

ament_target_dependencies(test_ros2bridge rclcpp 
                            sensor_msgs
                            rclcpp_action
                            control_msgs
                            trajectory_msgs
                            )
ament_target_dependencies(tcp_server_node rclcpp)

install(TARGETS moveit_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS dual_ur_inspire_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS left_arm_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS right_arm_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS hands_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS test_ros2bridge
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS tcp_server_node
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS joint_state_filter
  DESTINATION lib/${PROJECT_NAME}) 

include_directories(
    include
  )
install(DIRECTORY include/
  DESTINATION include
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
