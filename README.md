# unreal_airsim
This repo contains simulation tools and utilities to perform realistic simulations base on [Unreal Engine](https://www.unrealengine.com/en-US/) (UE4), using microsoft [AirSim](https://github.com/microsoft/AirSim) as interface to UE4.
 
# Table of Contents
* [Installation](#Instalation)
* [Examples](#Examples)
* [Troubleshooting](#Troubleshooting)

# Installation
The following 3 components are necessary to utilize the full stack of unreal_airsim tools.

**Unreal Engine**

Install Unreal Engine. This repository was developped and tested on UE 4.24, which is the recommended version.
To install UE4 on linux, you need to register with Epic Games and build it from source. 
Please follow the detailed instructions on [their website](https://docs.unrealengine.com/en-US/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html) to set everything up.
If you plan to use *only* pre-compiled binaries as simulation worlds, this section can be omitted,

**Airsim**

Install *our fork* of AirSim, the UE4 Plugin:
```shell script
cd </where/to/install>
git clone https://github.com/ethz-asl/AirSim.git
cd Airsim
./setup.sh 
./build.sh
```

**unreal_airsim**

Install unreal_airsim, containing the simulation ROS-package and tools.

Dependencies:
```shell script
sudo apt-get install TODO
```

Install:
```shell script
cd ~/catkin_ws/src
git clone https://github.com/ethz-asl/unreal_airsim.git
cd unreal_airsim
```
In `CMakeLists.txt`, change the airsim root path in line 7 to your install directory `set(AIRSIM_ROOT /where/to/install/AirSim)`.
```shell script
catkin build --this
source ../../devel/setup.bash
```
# Examples
TODO

# Troubleshooting

## Installation
* **Include error 'xlocale.h" not found:**

    xlocale was removed/renamed from glibc somewhen, can fix via symlink:
    ```shell script
     ln -s /usr/include/locale.h /usr/include/xlocale.h
    ```
## Starting the simulation
* **Error at startup: bind(): address already in use:**

    Airsim connects to UE via a TCP port. If multiple instances of Airsim (i.e. the unreal game) are running, the selected port will already be taken.
    Note that the editor itself already loads the Airsim plugin, thus closing all instances of UE4 (editor or game) and starting a single game/editor usually fixes this.



