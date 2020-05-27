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
  void depthImageCallback(const sensor_msgs::ImagePtr &msg);
  void colorImageCallback(const sensor_msgs::ImagePtr &msg);
  void segmentationImageCallback(const sensor_msgs::ImagePtr &msg);

 protected:
  // setup
  static ProcessorFactory::Registration<DepthToPointcloud> registration_;

  // ROS
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber color_sub_;
  ros::Subscriber segmentation_sub_;

  // queues
  std::deque<sensor_msgs::ImagePtr> depth_queue_;
  std::deque<sensor_msgs::ImagePtr> color_queue_;
  std::deque<sensor_msgs::ImagePtr> segmentation_queue_;

  // variables
  bool use_color_;
  bool use_segmentation_;
  bool is_setup_;
  float fov_;    // depth cam intrinsics, fov in degrees
  float focal_length_;
  float vx_;
  float vy_;
  uint8_t infrared_compensation_[256]
      {0, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 1, 255, 255, 255, 255, 255, 255, 255, 255, 2, 255,
       255, 255, 255, 255, 3, 255, 255, 255, 255, 255, 4, 255, 255, 255, 5, 255, 255, 255, 6, 255, 255, 255, 7, 255,
       255, 8, 255, 255, 255, 9, 255, 255, 10, 255, 255, 11, 255, 12, 255, 255, 13, 255, 14, 255, 255, 15, 255, 16, 255,
       17, 255, 18, 255, 19, 255, 20, 255, 21, 255, 22, 255, 23, 24, 255, 25, 255, 26, 27, 255, 28, 255, 29, 30, 255,
       31, 32, 255, 33, 34, 35, 255, 36, 37, 255, 38, 39, 40, 255, 41, 42, 43, 44, 45, 255, 46, 47, 48, 49, 50, 51, 255,
       52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 76, 77, 78, 79, 80,
       81, 82, 83, 85, 86, 87, 88, 90, 91, 92, 93, 94, 96, 97, 98, 100, 101, 103, 104, 105, 107, 108, 109, 111, 112,
       113, 115, 116, 118, 119, 121, 122, 124, 125, 127, 128, 130, 131, 133, 134, 136, 137, 139, 141, 142, 144, 145,
       147, 149, 151, 152, 154, 156, 157, 159, 161, 162, 164, 166, 168, 169, 171, 173, 175, 177, 178, 180, 182, 184,
       186, 188, 190, 192, 193, 195, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 217, 219, 221, 223, 225, 228,
       230, 232, 234, 236, 239, 241, 243, 245, 248, 250, 252, 255};

  // params
  int max_queue_length_;
  float max_depth_; // points beyond this depth [m] will be discarded
  float max_ray_length_; // points beyond this ray length [m] will be discarded
  bool use_infrared_compensation_;

  // methods
  void findMatchingMessagesToPublish(const sensor_msgs::ImagePtr &reference_msg);
  void publishPointcloud(const sensor_msgs::ImagePtr &depth_ptr,
                         const sensor_msgs::ImagePtr &color_ptr,
                         const sensor_msgs::ImagePtr &segmentation_ptr);
};

} // namespcae unreal_airsim::simulator_processor

#endif // UNREAL_AIRSIM_SIMULATOR_PROCESSOR_DEPTH_TO_POINTCLOUD_H_
