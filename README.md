# Master Thesis: Asymptotically Optimal Belief Space Planning

The code implemented as part of the thesis consists of two parts:

1. Implementation of the PTO planner in OMPL: https://github.com/janisfreund/thesis-ompl
The code is a fork of https://github.com/ompl/omplapp.

2. Implementation of the simulation environment in PyBullet: https://github.com/janisfreund/thesis-pybullet
The code builds on the OMPL/PyBullet Interface https://github.com/lyfkyle/pybullet_ompl.


## OMPL

### Installation

First, both repositories have to be cloned to the same directory. OMPL can be installed by performing the following steps:

1. Run the installation script provided by ompl: https://ompl.kavrakilab.org/install-ompl-ubuntu.sh using the command:`./install-ompl-ubuntu.sh --python`
2. Create build directory in `./thesis-ompl` with `mkdir build` and go to the folder with `cd build`
2. Generate Python bindings with `make -j 4 update_bindings`
3. Compile with `make -j 4`
4. Install required Python packages with `pip install pybullet numpy opencv-python scipy matplotlib progressbar`

### Relevant files
The most relevant files for the thesis are:
- `./thesis-ompl/ompl/src/ompl/geometric/planners/partial/Partial.h` and `./thesis-ompl/ompl/src/ompl/geometric/planners/partial/src/Partial.cpp` implement the PTO planner
- `./thesis-ompl/ompl/src/ompl/base/World.h` and `./thesis-ompl/ompl/src/ompl/base/src/World.cpp` implement the world class


## PyBullet

The most relavant files are:
- `./thesis-pybullet/pb_ompl.py`: Implements the interface to OMPL
- `./thesis-pybullet/scripts/benchmarks.py`: Generates benchmark plots
- `./thesis-pybullet/scripts/camera_state_sampler.py`: Implements the camera-based state sampler
- `./thesis-pybullet/scripts/create_env_imgs.py`: Can be used to create images of environments
- `./thesis-pybullet/scripts/demos.py`: Demos planned path trees
- `./thesis-pybullet/scripts/environments.py`: Specifies the environments
- `./thesis-pybullet/scripts/robots.py`: Defines the robots
- `./thesis-pybullet/scripts/test_environments.py`: Tests specific robot configurations in specified environmnets

Models of the robots and environments are stored in the directory `./thesis-pybullet/models`.
This includes models from the following sources:
- https://github.com/AutonomyLab/create_robot
- https://www.turbosquid.com/de/3d-models/3d-model-golden-retriever-1850419
- https://www.turbosquid.com/de/3d-models/3d-model-lowpoy-suv-car-1937266
- https://sketchfab.com/3d-models/isometric-office-d31464eed8044190911b221648aca432
- https://sketchfab.com/3d-models/floor-plan-with-furniture-813f854c770b4678a616a9ebf7534fe6
- https://www.turbosquid.com/de/3d-models/bookshelf-for-low-poly-games-3d-model-1860043
- https://www.turbosquid.com/de/3d-models/3d-industrial-shed-warehouse-building-1603744