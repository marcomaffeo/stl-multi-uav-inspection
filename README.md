# Signal Temporal Logic Motion Planning for Multi-UAV Power Tower Inspection

This repository contains a framework for encoding, planning, and executing complex inspection missions for a fleet of quadrotors operating near electrical power lines. The project leverages Signal Temporal Logic (STL) to rigorously define spatial and temporal constraints, ensuring safe and feasible trajectory generation for multi-rotor systems.

## Project Overview

Deploying Unmanned Aerial Vehicles (UAVs) for the inspection of civilian infrastructure, such as power transmission lines, introduces significant technical challenges, including obstacle avoidance and operation within strictly bounded timeframes. 

This project addresses these challenges through a three-phase approach:
1. **Requirement Codification:** Identification of safety and mission requirements from state-of-the-art methods and their mathematical formalization using Signal Temporal Logic (STL).
2. **Multi-UAV Trajectory Generation:** Development of a complex mission planner for up to four UAVs. The planner generates feasible trajectories (position, velocity, and acceleration) that strictly satisfy physical constraints, obstacle avoidance, minimum time-on-target requirements, and return-to-base protocols.
3. **Simulation and Validation:** Implementation and testing of the generated trajectories in a realistic environment using the Gazebo robotics simulator and the MRS UAV system for drone dynamics and onboard control.

## Media

### Single UAV Inspection
Demonstration of a single drone executing the STL-encoded inspection task, maintaining safety bounds and temporal constraints around the power tower.

https://github.com/user-attachments/assets/8095014d-d8ec-4b86-afa0-787048e40cab

### Multi-UAV Fleet Inspection
Coordinated inspection mission involving a fleet of drones. The UAVs visit respective target regions, satisfy time requirements, and safely return to their initial poses without inter-robot collisions.

https://github.com/user-attachments/assets/466a081c-246d-486e-91fe-86e172fc524e

## Architecture and Tools

*   **Mathematical Modeling & Planning:** MATLAB is used as the primary engine for formulating and solving the STL-based optimization problems. The repository includes heavy dependencies such as CasADi and CVX to handle the underlying non-linear and convex optimization tasks.
*   **Physics Simulation:** The Gazebo simulator provides a realistic physics environment for the civilian infrastructure and the UAVs.
*   **Robot Operating System (ROS):** The framework integrates with ROS and the Multi-Robot Systems (MRS) UAV system to bridge the gap between kinematic trajectory generation and dynamic closed-loop control.

## Dependencies

*   **MATLAB:** Required for running the STL optimization scripts (compatible with Windows/Linux).
*   [CasADi](https://web.casadi.org/) & [CVX](http://cvxr.com/cvx/) (Included in the `Software` directory).
*   **Ubuntu:** Mandatory for the ROS/Gazebo simulation stack.
*   [ROS](https://www.ros.org/) (Standard distributions compatible with MRS).
*   [Gazebo 11](https://gazebosim.org/)
*   [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system): For simulating realistic drone dynamics and low-level control.
