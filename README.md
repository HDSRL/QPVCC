


# Quadratic-Programming Based Virtual Constraint Controllers for Quadrupedal Robots 
- This repository was created for the paper [Robust stabilization of periodic gaits for quadrupedal locomotion via QP-based virtual constraint controllers](https://ieeexplore.ieee.org/abstract/document/9638982)
- Another implementation (simulation only) can be found in [Quadrupedal Locomotion via Event-Based Predictive Control and QP-Based Virtual Constraints](https://ieeexplore.ieee.org/abstract/document/9113252)

## Overview
- Controller of the A1 Robot in the Hybrid Dynamic Systems and Robot Locomotion Lab at Virginia Tech (Lab Website: https://www.kavehakbarihamed.com/)
- This code uses the Raibert heuristic and an I-O linearizing QP
- When using this code/method, please cite other works in the lab appropriately
- This code uses the [RaiSim](https://github.com/raisimTech/raisimLib) physics engine for simulation
- The visualization is done using the [RaisimOgre](https://github.com/raisimTech/raisimOgre)
- The code is written in C++ and uses the Eigen library for linear algebra
- The code is designed to be run on Linux, and has been tested on Ubuntu 20.04/22.04



## Code Organization
The bulk of the code and computations is run through the class "LocoWrapper.cpp". However, the main function for initiating RaiSim and the simulations is in src/A1_Sim.cpp..

## Where to git clone
Clone this repository under raisim_workspace which was created while installing RaiSim.

## Prerequisite
- In bash, the following must be defined
```sh
export WORKSPACE=${HOME}/raisim_workspace
export LOCAL_BUILD=${HOME}/raisim_workspace/build
```
- Be sure to add your license to the /rsc folder, and change the license name in src/A1_Sim.cpp
- Make sure to provide correct paths to RaiSim and RaiSimOgre in the CMakeLists.txt file


## Building and Running Code
### Build Code From Scratch
Make your way into the project directory
```sh
cd mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=$LOCAL_BUILD
make -j8
```

### Build and Run Code Using Script
Make your way into the project directory
```sh
./run_Raisim.sh
```

### Run Example from Command Line
```sh
cd ./build
./Run_Sim ../params/LL_w_CLF.txt ../params/MPC_params.txt ../params/Walking_params.txt
```
- The inputs into the function call are optional, and may have different names so long as the naming convention is followed (see "Changing System Parameters" below)

## Changing System Parameters
- Open the files in /params
- There are three seperate parameter files. "LL" contains low level parameters, "MPC" contains the MPC parameters, and "Walking" contains the walking parameters.
- Note that new parameter files may created with different names, so long as the following naming convention is adhered to:
    - The low level parameters file must contain "LL" in the name
    - The MPC parameters file must contain "MPC" in the name
    - The walking parameters file must contain "Walking" in the name
- The bottom of the parameters files include information on the parameters that can be used/changed

Here is an example of the contents of a low level parameters file:
```
0.6
400
40
0
1
0.1
1000000
100000000
100
0.8


// =========================================== //
// ======= Ordering and default values ======= //
// =========================================== //
// Default Low Level
mu = 0.6;    // coefficient of friction
kp = 400;   // proportional gain on IO linearization
kd = 40;    // derivative gain on IO linearization
useCLF = 0;  // 1 or 0, indicates whether or not to use the CLF 

// QP Cost
tauPen = 1e0; // input weight
dfPen  = 1e-1; // ||F-F_d|| weight
auxPen = 1e6; // auxiliary var weight
clfPen = 1e8; // clf defect var weight

// QP Constraints
auxMax = 100;  // max aux value
clfEps = 0.8; // CLF convergence tuning variable

```
Notice that the bottom half of the file contains information about the ordering and default values, while the top half is the portion that should actually be changed to alter the behavior. Changing this file does not require that the executable is recompiled. Each text file provided will be read at the time of execution.

## Visualization Parameters
There are a series of parameters at the bottom of the simulation script that allows the user to control the visualization properties in RaiSim, as well as produce a recording automatically, most of which are self explainatory. In particular, the following parameters are available:

```
raisim::gui::showContacts = false;
raisim::gui::showForces = false;
raisim::gui::showCollision = false;
raisim::gui::showBodies = true;

std::string cameraview = "isoside"; // iso, side, top, front, sideiso
bool panX = true;                   // Pan view with robot during walking (X direction)
bool panY = false;                  // Pan view with robot during walking (Y direction)
bool record = false;                // Record?
double startTime = 0*ctrlHz;        // Recording start time
double simlength = 300*ctrlHz;      // Sim end time
double fps = 30;                  
std::string directory = "PATH_TO_OUTPUT_DIRECTORY";
std::string filename = "NAME_OF_VIDEO";
```

Note that the first four parameters, as well as panX and panY, can also be enabled/disabled using the GUI interface during simulation. Another important note is that, when panX or panY are enabled, the user will obtain erratic behavior when clicking on an object to change the view via the mouse. 

Finally, there are several different checkerboard colors provided in the material file (rsc/material/myMaterials.material) and can be changed in the A1_Sim.cpp file. To generate a different cherckerboard color pattern, use the MATLAB script (rsc/material/ColoredChecker.m) and create a new material in myMaterials.material following the same convention as the other checkerboard materials.

## Logging Data
In order to log data, open /src/LocoWrapper.cpp and change the "filename" string in the constructor. If this string is empty, no data will be logged. Be sure to include the entire path, including the filename and extension (.txt or .csv are recommended). 


## Please Cite Us
Please cite the following paper if you use this code:

**R. T. Fawcett, A. Pandala, A. D. Ames and K. Akbari Hamed,**"[Robust Stabilization of Periodic Gaits for Quadrupedal Locomotion via QP-Based Virtual Constraint Controllers](https://ieeexplore.ieee.org/abstract/document/9638982)," **in IEEE Control Systems Letters, vol. 6, pp. 1736-1741, 2022, doi: 10.1109/LCSYS.2021.3133198.**

```
@ARTICLE{9638982,
  author={Fawcett, Randall T. and Pandala, Abhishek and Ames, Aaron D. and Akbari Hamed, Kaveh},
  journal={IEEE Control Systems Letters}, 
  title={Robust Stabilization of Periodic Gaits for Quadrupedal Locomotion via QP-Based Virtual Constraint Controllers}, 
  year={2022},
  volume={6},
  number={},
  pages={1736-1741},
  keywords={Orbits;Quadrupedal robots;Stability analysis;Legged locomotion;Numerical stability;Asymptotic stability;Switches;Stability of hybrid systems;robotics;stability of nonlinear systems},
  doi={10.1109/LCSYS.2021.3133198}}
  ```