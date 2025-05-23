#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char *argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "moveit_node",
      rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("moveit_node");

  // Next step goes here

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "left_arm");

  // Set a target Pose
  auto const target_pose = []
  {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = 0.5;
    msg.orientation.y = -0.5;
    msg.orientation.z = 0.5;
    msg.orientation.w = 0.5;
    msg.position.x = -0.82;
    msg.position.y = 0.49;
    msg.position.z = 2.0;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);

  // Create a plan to that target pose
  auto const [success, plan] = [&move_group_interface]
  {
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  RCLCPP_INFO(logger, "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());
  RCLCPP_INFO(logger, "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());
  RCLCPP_INFO(logger, "Pose reference frame: %s", move_group_interface.getPoseReferenceFrame().c_str());


  // Execute the plan
  if (success)
  {
    move_group_interface.execute(plan);
  }
  else
  {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
