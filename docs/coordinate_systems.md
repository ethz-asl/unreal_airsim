# Coordinate Systems
Unreal Engine, AirSim, and unreal_airsim all use different coordinate systems and parametrization conventions. 
These systems and conventions as well as conversions are thus briefly introduced.

![frames](https://user-images.githubusercontent.com/36043993/81938412-2811c200-95f5-11ea-8a80-5c95efa48a88.png)

## Unreal Engine
UE4 uses standard xyz-coordinates, although the coordinate system is **not** right-handed (i.e. x-forward, y-right, z-up).
Default units is cm.
Rotations are parametrized as Euler-XYZ (roll-pitch-yaw), however, rotations are mathematically positive around the x-axis and negative around the y- and z-axes.
Default units are degrees.

This convention is used for everything inside the UE4 Editor, e.g. when creating worlds or exporting ground truth.

## AirSim
Airsim uses North-East-Down (NED) coordinates, where north is aligned with the Unreal Engine x-axis. 
Default units are m. 
Rotations are parametrized as Euler-ZYX (yaw-pitch-roll), default units are degrees.

Airsim uses different frames, where the output of data is typically in `AirsimLocal` frame, which is (according to their docs) aligned with PlayerStart in UE4.
Notice, that because the frame is NED, it is invariant w.r.t the orientation of PlayerStart.
For data obtained in other frames (e.g. Lidar in SensorLocalFrame), the parametrization is still NED but with North aligned with the sensor-x-axis.

* **Note:** Although `AirsimLocal` is supposed to lie at PlayerStart, there seems to be a lot of strange things in there.
  E.g. that the origin of a robot is not it's center but is shifted by its height/2 to the lowermost point. 
  Furthermore, it's not clear whether they read out PlayerStart Coordinates or setup on connection, leading to strange behavior when connecting for multiple experiment runs inside a singel game.\
    **TODO: Verify T_UnrealWorld_AirsimLocal**, as it is important for evaluation vs ground truth.

This convention is used for everything interfacing with AirSim, such as client requests and responses, as well as in settings.json.

## unreal_airsim
Unreal_airsim uses the [ASL coordinate conventions](https://github.com/ethz-asl/mav_tools/wiki/Coordinate-Systems), i.e. all coordinate systems are right handed (x-forward, y-left, z-up).
The simulator frame is aligned with `AirsimLocal`, including the orientation of PlayerStart, thus coinciding fully with PlayerStart (odom frame) (ideally, as soon as above note is fixed).
Note that cameras operate in camera frame (x-right, y-down, z-depth).

Use this convention for everything that interfaces with the unreal_airsim simulator, e.g. published and subscribed topics, as well as in the unreal_airsim_settings.yaml.
