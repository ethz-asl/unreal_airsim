#ifndef UNREAL_AIRSIM_ONLINE_SIMULATOR_SENSOR_TIMER_H_
#define UNREAL_AIRSIM_ONLINE_SIMULATOR_SENSOR_TIMER_H_

#include <string>
#include <vector>

#include <kindr/minimal/quat-transformation.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include "unreal_airsim/frame_converter.h"

namespace unreal_airsim {
class AirsimSimulator;

/***
 *  This class manages the sensor timers and callbacks for the simulator.
 */
class SensorTimer {
 public:
  SensorTimer(const ros::NodeHandle& nh, double rate, bool is_private,
              const std::string& vehicle_name, AirsimSimulator* parent);
  virtual ~SensorTimer() = default;

  void timerCallback(const ros::TimerEvent&);

  double getRate() const;
  bool isPrivate() const;
  void signalShutdown();
  void addSensor(const AirsimSimulator& simulator, int sensor_index);

 protected:
  AirsimSimulator* parent_;  // Acces to owner

  // general
  bool is_shutdown_;
  double rate_;      // rate of the sensor callback
  bool is_private_;  // whether this timer runs on multiple sensors or a single
                     // one
  msr::airlib::MultirotorRpcLibClient airsim_client_;
  ros::Timer timer_;
  std::string vehicle_name_;
  ros::NodeHandle nh_;

  // methods
  void processCameras();
  void processLidars();
  void processImus();

  // cameras
  struct Sensor {
    ros::Publisher pub;
    std::string frame_name;
  };
  struct Camera : public Sensor {
    msr::airlib::ImageCaptureBase::ImageRequest request;
    kindr::minimal::QuatTransformationTemplate<double> T_S_B;
  };
  std::vector<Camera> cameras_;
  std::vector<msr::airlib::ImageCaptureBase::ImageRequest> camera_requests_;

  // lidars
  struct Lidar : public Sensor {
    std::string name;
    kindr::minimal::QuatTransformationTemplate<double> T_S_B;
  };
  std::vector<Lidar> lidars_;

  // imus
  struct Imu : public Sensor {
    std::string name;
  };
  std::vector<Imu> imus_;
};

}  // namespace unreal_airsim

#endif  // UNREAL_AIRSIM_ONLINE_SIMULATOR_SENSOR_TIMER_H_
