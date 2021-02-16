Seabotix Vehicle + Alpha Arm Package
---

This repository contains packages for an underwater vehicle-manipulator system 
consisting of a Seabotix vLBV300 underwater vehicle and a Reach Alpha arm.

Installation
---
This repository requires [Project Dave](https://github.com/Field-Robotics-Lab/dave).

See my install scripts [here](https://github.com/scchow/uvms_simulator_install) or install according to the instructions on the [Project Dave wiki](https://github.com/Field-Robotics-Lab/dave/wiki/Installation).

After building/installing Project Dave, clone this repository into your catkin workspace. 

Then rebuild the catkin workspace and `source devel/setup.bash` as usual.

Launch files
---

- `roslaunch seabotix_alpha_description valve_manipulation_demo.launch` - launches the whole system in an underwater environment

- `roslaunch seabotix_alpha_description seabotix_vehicle.launch` - launches the Seabotix vehicle only in an underwater environment

- `roslaunch alpha_description alpha_arm.launch` - launches the Reach 5 arm only in an underwater environment

- `roslaunch seabotix_alpha_moveit_config demo.launch` - launches the MoveIt! Rviz interface to generate plans for the vehicle + arm and execute on a dummy robot.

- `roslaunch seabotix_alpha_description moveit_demo.launch` + `roslaunch seabotix_alpha_moveit_config moveit_planning_execution.launch` - launches the whole system with controllers compatible with Moveit + the MoveIt! Rviz interface to generate plans. This will let you generate plans in MoveIt! and execute on the simulated robot in Gazebo.
