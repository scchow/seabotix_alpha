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

Make sure that all the Python files in the `seabotix_alpha/seabotix_alpha_description/scripts` are given permission to be executable by running:

```
chmod +x seabotix_alpha/seabotix_alpha_description/scripts/*.py
```

Launch files
---

- `roslaunch seabotix_alpha_description valve_manipulation_demo.launch` - launches the whole system in an underwater environment

- `roslaunch seabotix_alpha_description seabotix_vehicle.launch` - launches the Seabotix vehicle only in an underwater environment

- `roslaunch alpha_description alpha_arm.launch` - launches the Reach 5 arm only in an underwater environment

- `roslaunch seabotix_alpha_moveit_config demo.launch` - launches the MoveIt! Rviz interface to generate plans for the vehicle + arm and execute on a dummy robot.

- `roslaunch seabotix_alpha_description moveit_demo.launch` + `roslaunch seabotix_alpha_moveit_config moveit_planning_execution.launch` - launches the whole system with controllers compatible with Moveit + the MoveIt! Rviz interface to generate plans. This will let you generate plans in MoveIt! and execute on the simulated robot in Gazebo.

- `roslaunch seabotix_alpha_description uvms_world.launch` + `rosrun seabotix_alpha_description turbulence_experiment.py` - launches the Gazebo environment with the Seabotix vehicle and Alpha arm + controllers for position/joint angle control. The second script adds oscillatory currents to test its stationkeeping ability. Results are logged in `/home/developer/uuv_ws/experiment_logs/<timestamp>`.

Scripts
---

- `rosrun seabotix_alpha_description currents.py` - causes currents in the environment based on vehicle pose. Modify the parameters in this script to set different current fields.

- `rosrun seabotix_alpha_description currents_reset.py` - sets the currents back to 0. 

Recomputing the Thruster Allocation Matrix
---

The simulator precomputes the Jacobian that maps thruster output to vehicle motion.
This thruster allocation matrix will need to be updated if the vehicle mass/hydrodynamic
parameters are changed. To do so, run the launch files with the `reset_tam` flag set to true:

`roslaunch seabotix_alpha_description valve_manipulation_demo.launch reset_tam:=true`
