# Universal Robot Tele-operation ROS Package

This ros package provides the universal robot tele-operation node with Force Dimension haptic device.

This package aimed to provide tele-operation, basic autonomy mode, and shared autonomy mode function for universal robot.


# Requirements
- Force dimension haptic device omega 3 for teleoperation


# Usage
```sh
# run the ur_modern_driver node firstly
roslaunch ur_modern_driver urXX_bringup.launch robot_ip:=ROBOT_IP_ADDRESS

# Run teleoperation node
rosrun ur_teleop listener
```

# More details
**NOTE:** currently this repo uses UR velocity control mode as default.
