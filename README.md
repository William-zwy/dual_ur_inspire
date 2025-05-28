# Dual arm robot ros driver

This repository provides our customized UR5e integration with robotiq 3 Finger gripper.  

![real_robot](https://user-images.githubusercontent.com/6389003/141102453-e75c4ded-fe8f-4a26-9d55-d97c1e357f7d.JPG)

Tested on Ubuntu 22.04 with ROS 2 Humble.

### The features and usage of the dual arm driver are described on the [WIKI](https://github.com/yaesolKim/dual_ur5e/wiki).   


## Robot simulation
### Run moveit, Rviz   
```bash
ros2 launch dual_ur_inspire_bringup dual_ur_inspire_bringup.launch.py
```   
### Test
```bash
ros2 launch dual_ur_inspire_description dual_arm_moveitctrl.launch.py

ros2 launch dual_ur_inspire_description dual_arm_control.launch.py  
```

## CPP node pkg

```bash
moveit_node
```

## Isaacsim config

```bash
src/dual_ur_inspire/isaacsim_model
```
- src/dual_ur_inspire/isaacsim_model/dual_ur_arm_unmimic.usd是配置文件
- src/dual_ur_inspire/isaacsim_model/dual_ur_inspire_unmimic/dual_ur_inspire_unmimic.usd是模型

## Python 
在Python_code文件夹下，需要放到以下目录，若不需要isaacgym，直接在最后的while中注释即可。
```bash
teleop/teleop_hand.py

```
## Contact
已完成手关节控制节点分离，还需测试quest3中手腕的位姿与机械臂中位姿的对应关系
