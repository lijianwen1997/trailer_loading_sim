# Trailer Loading Simulator
This repository contains a simulation environment for trailer loading, adapted from the VRX simulator [VRX simulator](https://github.com/osrf/vrx).
Building upon the original VRX setup, this version introduces enhanced models and features to simulate realistic trailer loading scenarios. Key improvements include the addition of trailer and truck models, as well as a terrain model for Lake Harner, IN. These updates enable more accurate and detailed simulations for testing and development in the context of trailer loading systems.

## Prerequisite

Please follow the [tutorial](https://github.com/osrf/vrx/wiki/preparing_system_tutorial) to install ROS2 Humble and Gazebo Garden.

## Install the trailer loading simulator

Follow the steps to download and build the trailer loading simulator:
1. Create a colcon workspace and clone the vrx repository
```commandline
mkdir -p ~/trailer_ws/src
cd ~/trailer_ws 
git clone https://github.com/lijianwen1997/trailer_loading_sim.git
mv /trailer_loading_sim /src
```

2. Build and source the workspace
```commandline
colcon build --merge-install
. install/setup.bash
```

## Run the simulator
```commandline
ros2 launch vrx_gz trailer.launch.py 
```
