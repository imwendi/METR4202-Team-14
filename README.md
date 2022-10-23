# METR4202 Team 14 Source Code

## Test scripts
-  robot_test_1.py: used for Task1/2/3b for moving turn table
-  robot_unmoving: used for Task 3a for immobile turn table  

## Own modules (in `src`)
-  `claw` - claw control node
- `joint_controller` - node for processing desired poses and moving the robot to those poses
- `kinematics` - forwards/inverse kinematics computations and inverse kinematic solution selection based on collision handling heuristics
    - code in `plotting` unused by actual robot. They contain various `matplotlib` based simulations for experimenting with the inverse kinematics code
- `robot` - high-level robot state machines
- `vision` - nodes and utility classes for processing camera data for Aruco tags and color detection

## Third-party dependencies
- `numpy`
- `matplotlib`
- `modern_robotics`
- Requires running ROS nodes from
    - Dynamixel interface (https://github.com/UQ-METR4202/dynamixel_interface)
    - Ximea ROS (https://github.com/UQ-METR4202/metr4202_ximea_ros)

## Installation instructions
1. Install listed third-party dependencies
1. (optional) add aliases from `bash_aliases.txt` to bash_aliases on install machine
1. Clone repo to `src` folder of a catkin workspace
1. Build catkin workspace

## Usage instructions
1. Run commands under alias `start_all` in `bash_aliases.txt`
1. Use `rosrun` to run either of the two test scripts, under the package `team14`

## Repo
Github repository can be found at
```
https://github.com/imwendi/METR4202-Team-14/
```



 
