#ifndef UNREAL_AIRSIM_ONLINE_SIMULATOR_SENSOR_TIMER_H_
#define UNREAL_AIRSIM_ONLINE_SIMULATOR_SENSOR_TIMER_H_

#include "unreal_airsim/frame_converter.h"

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

// Airsim
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

#include <string>

namespace unreal_airsim {
class AirsimSimulator;

/***
 *  This class manages the sensor timers and callbacks for the simulator.
 */
class SensorTimer {
 public:
  SensorTimer(const ros::NodeHandle &nh,
              double rate,
              bool is_private,
              const std::string &vehicle_name,
              AirsimSimulator* parent);
  virtual ~SensorTimer() = default;

  void timerCallback(const ros::TimerEvent &);

  double getRate() const;
  bool isPrivate() const;
  void signalShutdown();
  void addSensor(const AirsimSimulator &simulator, int sensor_index);

 protected:
  AirsimSimulator* parent_;   // Acces to owner

  // general
  bool is_shutdown_;
  double rate_;    // rate of the sensor callback
  bool is_private_;  // whether this timer runs on multiple sensors or a single one
  msr::airlib::MultirotorRpcLibClient airsim_client_;
  ros::Timer timer_;
  std::string vehicle_name_;
  ros::NodeHandle nh_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  // methods
  void processCameras();
  void processLidars();
  void processImus();

  // cameras
  std::vector<ros::Publisher> camera_pubs_;
  std::vector<std::string> camera_frame_names_;
  std::vector<msr::airlib::ImageCaptureBase::ImageRequest> image_requests_;

  // lidars
  std::vector<ros::Publisher> lidar_pubs_;
  std::vector<std::string> lidar_names_;
  std::vector<std::string> lidar_frame_names_;

  // imus
  std::vector<ros::Publisher> imu_pubs_;
  std::vector<std::string> imu_names_;
  std::vector<std::string> imu_frame_names_;
};

} // namespcae unreal_airsim

#endif // UNREAL_AIRSIM_ONLINE_SIMULATOR_SENSOR_TIMER_H_
