# Dual arm robot ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)

Tested on Ubuntu 22.04 with ROS 2 Humble.

### The features and usage of the dual arm driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   


## Robot simulation
### Run gazebo, moveit, Rviz   
```bash
ros2 launch robot_gazebo dual_ur_robotiq.launch.py world:=table_box
ros2 launch robot_moveit_config robot_moveit_planning_execution.launch.py sim:=true
ros2 launch robot_moveit_config moveit_rviz.launch.py
```   
### Pick and place simulation
```bash
ros2 run bimanual_manipulation pick_place_collision
```

## Real robot execution: Bring up grippers and robots, Run moveit and Rviz   
Run the lines below in the respective terminals.

```bash
# Terminal 1
ros2 launch robotiq_3f_gripper_control dual_gripper_tcp.launch.py
ros2 launch robotiq_3f_gripper_joint_state_publisher dual_gripper_joint_state_publisher.launch.py
ros2 launch robotiq_3f_gripper_visualization robotiq_gripper_upload.launch.py

# Terminal 2
ros2 launch ur_robot_driver robot_bringup.launch.py   
ros2 launch robot_moveit_config robot_moveit_planning_execution.launch.py sim:=false     
ros2 launch robot_moveit_config moveit_rviz.launch.py   
```

## Gripper 
- Mode: basic, pinch, wide scissor
- Action: close, open   

```bash
ros2 run robotiq_3f_gripper_control Robotiq3FGripperSimpleController.py  
ros2 run robotiq_3f_gripper_control Robotiq3FGripperStatusListener.py
```

## Contact
All bug reports, feedback, comments, contributions or remarks are welcome.
