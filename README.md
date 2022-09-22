# metr4202_w7_prac

In this practical the `dynamixel_interface` ROS package was introduced to allow for control of the joint angles for each Dynamixel motor. The package was originally developed by `csiro-robotics` which has been forked under `UQ-METR4202` to pre-configure the package for the course project. Follow the steps under **Dynamixel Setup** in the course resources repository, `RPi4_Setup.md`, to begin utilising the `dynamixel_interface` package.

Once the packages are cloned, built and sourced, the `dynamixel_interface_controller_node` can be launched alongside a slider GUI for your convenience. The commands to run are the following.

```SH
# Only the Dynamixel controller
roslaunch dynamixel_interface dynamixel_interface_controller.launch
```

```SH
# Both controller and slider
roslaunch dynamixel_interface dynamixel_interface_slider.launch
```

*Be mindful both should not run simultaneously.*

The preceding commands launches the `dynamixel_interface_controller_node` which subscribes to a `/desired_joint_states` topic. Your team will need to publish to this topic in order to command the Dynamixel motors using the `sensor_msgs/JointState` message type. The controller node also publishes to a `/joint_states` topic which provides feedback of the actual state of each motor.

## joint_states_publisher.py

This repository is a ROS package which you can clone into your workspace under `~/catkin_ws/src`. The following steps are reminders to clone and build packages.

```sh
# Go to workspace source directory
cd ~/catkin_ws/src
# Clone this repository
git clone https://github.com/UQ-METR4202/metr4202_w7_prac
# Go back a directory (i.e., ~/catkin_ws)
cd ..
# Build workspace
catkin build
# Source workspace
# (only when adding new packages/scripts, always with new terminals)
source devel/setup.bash
```

## Usage

Run the following commands in **three separate** terminals. Don't forget to source! Make sure there is plenty of room around your robot **before** running as it'll start **dancing**.

```sh
# Terminal 1
roslaunch dynamixel_interface dynamixel_interface_controller.launch
# Terminal 2
rosrun metr4202_w7_prac joint_states_publisher.py
# Terminal 3
rostopic pub /desired_pose geometry_msgs/Pose -r 1 -- '[0, 0, 0]' '[0, 0, 0, 0]'
```

The script initialises a node that subscribes to the `/desired_pose` topic and publishes to the `/desired_joint_states` topic as a callback to the subscription.

## Hints

You may refer to the code under `scripts/` to get an idea of how to begin your inverse kinematics node if that is how you want to structure your ROS stack, but remember, it is entirely up to you and your team to design the ROS architecture.

Try to understand why the following commands are run, especially with terminal 3. Do you really need to run it in order for the `joint_states_publisher.py` to publish joint states?