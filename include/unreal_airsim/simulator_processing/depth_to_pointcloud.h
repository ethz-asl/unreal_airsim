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
  int infrared_compensation_[256]
      {0, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1, -1, 2, -1, -1, -1, -1, -1, 3,
       -1, -1, -1, -1, -1, 4, -1, -1, -1, 5, -1, -1, -1, 6, -1, -1, -1, 7, -1, -1, -1, 8, -1, -1, 9, -1, -1, 10, -1, -1,
       11, -1, 12, -1, -1, 13, -1, 14, -1, -1, 15, -1, 16, -1, 17, -1, 18, -1, 19, -1, 20, -1, 21, -1, 22, -1, 23, 24,
       -1, 25, -1, 26, 27, -1, 28, -1, 29, 30, -1, 31, 32, -1, 33, 34, -1, 35, 36, 37, -1, 38, 39, 40, -1, 41, 42, 43,
       44, -1, 45, 46, 47, 48, 49, 50, -1, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69,
       70, 71, 72, 73, 74, 75, 77, 78, 79, 80, 81, 82, 83, 85, 86, 87, 88, 89, 91, 92, 93, 95, 96, 97, 98, 100, 101,
       102, 104, 105, 106, 108, 109, 111, 112, 113, 115, 116, 118, 119, 121, 122, 124, 125, 127, 128, 129, 131, 133,
       134, 136, 137, 139, 140, 142, 144, 145, 147, 149, 150, 152, 154, 155, 157, 159, 160, 162, 164, 166, 168, 169,
       171, 173, 175, 177, 178, 180, 182, 184, 186, 188, 190, 192, 193, 195, 197, 199, 201, 203, 205, 207, 209, 211,
       213, 216, 218, 220, 222, 224, 226, 228, 230, 232, 235, 237, 239, 241, 243, 246, 248, 250, 252, 255};

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
