# Semester Project STL 2024

This repository contains MATLAB and ROS scripts for the semester project on Signal Temporal Logic (STL).

## Disclaimer

The simulation results were obtained using **MATLAB 2019b** on Ubuntu 18.04 (also compatible with Ubuntu 20.04) and on Windows 10 (regardless of the software release). CasADi and CVX were utilized as tools to solve the optimization problem. Specifically, versions `R2014b or later` of CasADi and version `2.2, January 2020, Build 1148` of CVX were employed. The libraries are included in this repository.

For visualizing all LaTeX equations mentioned in this repository, you can use [this useful web editor](https://www.codecogs.com/latex/eqneditor.php).

## Installation Instructions

To run the code listed in this repository, you will need the following packages:

- [MPT 3.+ toolboxes](https://www.mpt3.org/Main/Installation)
- [CVX](http://cvxr.com/cvx/download/). On Windows, ensure you install the MEX compiler. All necessary information depending on your MATLAB version can be found [here](https://www.mathworks.com/support/requirements/previous-releases.html).
- [Casadi for MATLAB](https://github.com/casadi/casadi/wiki/InstallationInstructions)
- [HSL routines for IPOPT](http://www.hsl.rl.ac.uk/ipopt/) (optional)

In the repository, particularly in the `Software\MATLAB\MPT3` directory, an installation file (i.e., a `.m` script) is provided. This script simplifies the installation of MPT3 and all related toolboxes. Simply open the script with MATLAB and run it. The script will download all necessary files from the web and configure the packages to make them available in MATLAB (i.e., they will be added to MATLAB paths). On Linux, ensure you install MATLAB with sudo permissions to avoid issues with toolbox usage. After the installation, restart MATLAB.

For the CVX toolbox, download and extract the contents of the `.zip` file from the [CVX website](http://cvxr.com/cvx/download/). Then, copy it to the `Software\MATLAB\cvx` folder. Next, run the `cvx\cvx_setup.m` file in MATLAB (similar to the MPT3 installation). The script will verify that everything is working correctly. The toolbox can be used with or without a license. To unlock all features, it is suggested to run the `cvx_setup ../cvx_license.dat` command. More information on obtaining a free academic license can be found [here](http://cvxr.com/cvx/academic/).

Finally, for the CasADi toolbox, download and extract the contents of the `.zip` file from the [Casadi website](https://github.com/casadi/casadi/wiki/InstallationInstructions) into either the `Software\MATLAB\casadi-matlabR2014b-v3.3.0` folder (for Windows) or the `Software\MATLAB\casadi` folder (for Linux). No additional steps are required.

The numerical simulations in MATLAB and the experimental results have been obtained using the following toolboxes:

```
Toolbox "mpt:3.2.1:all" added to the Matlab path.
Toolbox "mptdoc:3.0.4:all" added to the Matlab path.
Toolbox "cddmex:1.0.1:glnxa64" added to the Matlab path.
Toolbox "fourier:1.0:glnxa64" added to the Matlab path.
Toolbox "glpkmex:1.0:glnxa64" added to the Matlab path.
Toolbox "hysdel:2.0.6:glnxa64" added to the Matlab path.
Toolbox "lcp:1.0.3:glnxa64" added to the Matlab path.
Toolbox "sedumi:1.3:glnxa64" added to the Matlab path.
Toolbox "yalmip:R20200116:all" added to the Matlab path.
```

**[NOTE]**: Below is the procedure to install Gurobi for CVX.

1. Visit the [Gurobi website](https://www.gurobi.com/downloads/end-user-license-agreement-academic/) and request a free academic license.
2. With a free Gurobi license, visit [this website](https://www.gurobi.com/downloads/end-user-license-agreement-academic/) to obtain the HEX code for CVX.
3. Install Gurobi as an independent package by following the instructions on the [official webpage](https://www.gurobi.com/downloads/gurobi-optimizer-eula/).
4. Finally, follow the instructions provided [here](http://cvxr.com/cvx/doc/gurobi.html) to complete the installation process.

**[NOTE]**: Gurobi can speed up the initial guess solution, particularly useful for complex scenarios (e.g., with a high number of UAVs or target regions). However, it is unnecessary for very simple scenarios (e.g., up to 20 targets and 4 drones).

## Repository Content

- The `Software` folder contains all scripts developed to formulate the motion planning problem in MATLAB/Simulink. Additionally, TMUX scripts for Gazebo simulations are provided. Each folder contains a `README.md` file with a brief description of the script's content and operation, along with a complete and detailed list of parameters.

## QA

- `Warning: Name is nonexistent or not a directory`
  - Run `restoredefaultpath` and then `savepath` in MATLAB.
- Additional tools and solvers can be installed to improve algorithm performance. For IPOPT, refer to [this useful package for MATLAB](https://github.com/gsilano/mexIPOPT).
- `Warning: MATLAB did not appear to successfully set the search path..`
  - Run the command `which pathdef`. Then, open the given file with any text editor with sudo permissions. Finally, remove any unnecessary paths from the list.
