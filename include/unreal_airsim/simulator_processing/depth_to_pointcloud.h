#ifndef UNREAL_AIRSIM_SIMULATOR_PROCESSOR_DEPTH_TO_POINTCLOUD_H_
#define UNREAL_AIRSIM_SIMULATOR_PROCESSOR_DEPTH_TO_POINTCLOUD_H_

#include "unreal_airsim/simulator_processing/processor_base.h"

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <string>
#include <deque>

namespace unreal_airsim::simulator_processor {
/***
 * Absorbs a depth and optionally a color image and transforms it into a point cloud in camera frame
 * (x left, y down, z - into image plane)
*/
class DepthToPointcloud : public ProcessorBase {
 public:
  DepthToPointcloud() = default;
  virtual ~DepthToPointcloud() = default;

  bool setupFromRos(const ros::NodeHandle &nh, const std::string &ns) override;

  // ROS callbacks
  void depthIamgeCallback(const sensor_msgs::ImagePtr &msg);
  void colorIamgeCallback(const sensor_msgs::ImagePtr &msg);

 protected:
  // setup
  static  ProcessorFactory::Registration<DepthToPointcloud> registration_;

  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber color_sub_;

  // queues
  std::deque<sensor_msgs::ImagePtr> depth_queue_;
  std::deque<sensor_msgs::ImagePtr> color_queue_;

  // variables
  bool use_color_;
  int max_queue_length_;
  bool is_setup_;
  float fov_;    // depth cam intrinsics, fov in degrees
  float focal_length_;
  float vx_;
  float vy_;

  // methods
  void publishPointcloud(const sensor_msgs::ImagePtr &depth_ptr, const sensor_msgs::ImagePtr &color_ptr);
};

} // namespcae unreal_airsim::simulator_processor

#endif // UNREAL_AIRSIM_SIMULATOR_PROCESSOR_DEPTH_TO_POINTCLOUD_H_
