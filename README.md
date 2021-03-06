# mt-MPC (multi-trajectory Model Predictive Control) ROS packages

This repository contains the ROS packages developed and used for simulation and experimental tests of mt-MPC controller.
The control algorithm is described in the article **insert article here**.

## Test videos

### mt-MPC experimental Tests, ROS + Octocopter Drone
[![mt-MPC experimental test](img/mt_test.png)](https://www.youtube.com/watch?v=0suE8IxzbC0)

### mt-MPCsimulation on MatLab/Simulink
[![mt-MPC matlab simulation](img/multi-traj.jpg)](https://www.youtube.com/watch?v=V0cSYErvVtQ)

## Scheme of the packages
![scheme of mt-MPC package](img/mpc_scheme.png "packages scheme")
This scheme represents the interactions between the packages:
Package `s1000_interface` is used to interact with the lower level controller of the drone (a DJI S1000, controlled with a DJI A3 board) and the LiDAR sensor.
Package `mpc_s1000_controller` is where the control algorithm is actually implemented.
