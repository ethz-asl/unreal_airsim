# Settings
## Introduction
AirSim uses its own settings file, which is loaded from `~/Documents/AirSim/settings.json`, to setup the simulation in UE4.
Because the unreal_airsim simulator needs to be aware of the same settings and for ease of use, a **single** settings file from the unreal_airsim repo is used for both.

Specifiy your required settings (see [below](#Using-the-unreal_airsim-settings)) in `my_settings.yaml` and then use the Config Parser 
```
roslaunch unreal_airsim parse_config_to_airsim.launch source:=path/to/my_settings.yaml
```
to generate the `settings.json` for you. 
The parser still allows you to specify arbitrary AirSim-Settings-fields, but also takes care of e.g. frame conversions and required settings for the simulator to run properly.
Make sure to always have matching settings, i.e. load `my_settings.yaml` into the airsim simulator node and rerun the settings parser and restart the unreal game whenever the settings are changed.

## Using the unreal_airsim settings
The unreal_airsim config file is structured into two parts, being the **General Settings** and **Components**.

* The **General Settings** contain all parameters for the simulator and general settings for AirSim.

* As of this writing, there exists 2 types of **Components**: sensors and processors, where sensors specify the sensor setup that is to be simulated, and processors can perform arbitrary pre- or post-processing steps that require access to the simulation.
  The components are all organized as `component_type/name/params`. 
  The `name` is an unique specifier for the component to be built, but aside from default e.g. topic names it has no physical meaning.
  
All parameters for general settings and sensors are listed in the `online_simulator/simulator.h` in the Config struct and set in `readParamsFromRos()`.
All parameters for processors can be found in their individual `setupFromRos()` function.

The parameter naming is such that all unreal_airsim params are in `lower_case`. 
To set AirSim params (as in settings.json), just add them with identical name and value in `CamelCase` to my_settings.yaml.
They will be forwarded to the AirSim settings except when they interfere with crucial prerequisites.

An example of how to construct an unreal_airsim settings file is given in `cfg/demo.yaml`.
![settings](https://user-images.githubusercontent.com/36043993/80593316-48505700-8a21-11ea-8f67-7108c47d6f1e.png)