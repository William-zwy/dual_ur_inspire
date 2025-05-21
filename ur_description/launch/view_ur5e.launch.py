<?xml version="1.0"?>
<launch>
  <include file="$(find-pkg-share ur_description)/launch/load_ur5e.launch.py"/>

  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui" name="joint_state_publisher_gui"/>
  <node pkg="robot_state_publisher" exec="robot_state_publisher" name="robot_state_publisher"/>
  <node pkg="rviz2" exec="rviz2" name="rviz2" args="-d $(find-pkg-share ur_description)/config/view_robot.rviz"/>
</launch>
