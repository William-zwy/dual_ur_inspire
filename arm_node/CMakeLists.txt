cmake_minimum_required(VERSION 3.8)
project(arm_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(right_arm_node src/right_arm_node.cpp)
add_executable(left_arm_node src/left_arm_node.cpp)

ament_target_dependencies(right_arm_node 
                          rclcpp 
                          trac_ik_lib
                          geometry_msgs
                          std_msgs
                          sensor_msgs
                          )

ament_target_dependencies(left_arm_node 
                          rclcpp 
                          trac_ik_lib
                          geometry_msgs
                          std_msgs
                          sensor_msgs
                          )

install(TARGETS right_arm_node
    DESTINATION lib/${PROJECT_NAME}
        )
install(TARGETS left_arm_node
    DESTINATION lib/${PROJECT_NAME}
        )

include_directories(
  include
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
