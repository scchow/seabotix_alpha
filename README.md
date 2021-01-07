Seabotix Vehicle + Alpha Arm Package
---

This repository contains packages for an underwater vehicle-manipulator system 
consisting of a Seabotix vLBV300 underwater vehicle and a Reach Alpha arm.


Setup
---
Because of how the directories are set up, the configuration files for the controls and thrusters must be located in
`uuv_control_cascaded_pids/` and `uuv_thruster_manager/config` respectively.

So copy the `seabotix` folder (not the contents of the `seabotix` folder) in `config_files/uuv_control_cascaded_pids_config` into `uuv_simulator/uuv_control_cascaded_pids/config`
And copy the `seabotix` folder (not the contents of the `seabotix` folder) in `config_files/uuv_thruster_manager_config` into `uuv_simulator/uuv_thruster_manager/config/`

```
cp config_files/uuv_control_cascaded_pids_config/seabotix <path to catkin_ws>/uuv_simulator/uuv_control_cascaded_pids/config
cp config_files/uuv_thruster_manager_config/seabotix <path to catkin_ws>/uuv_simulator/uuv_thruster_manager/config
```