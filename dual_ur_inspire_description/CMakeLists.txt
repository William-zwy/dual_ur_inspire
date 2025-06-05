cmake_minimum_required(VERSION 3.8)
project(dual_ur_inspire_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(HW_IF_INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_control_interface REQUIRED) 


foreach(Dependency IN ITEMS ${HW_IF_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Install directories
install(DIRECTORY
  launch
  urdf
  meshes
  config
  world
  DESTINATION share/${PROJECT_NAME}
)

# ament_target_dependencies(
#   rclcpp
#   ${HW_IF_INCLUDE_DEPENDS}
#   # moveit_ros_planning_interface
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
