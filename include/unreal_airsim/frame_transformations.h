#ifndef UNREAL_AIRSIM_FRAME_TRANSFORMATIONS_H_
#define UNREAL_AIRSIM_FRAME_TRANSFORMATIONS_H_

// ROS
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <Eigen/Geometry>



namespace unreal_airsim {

/***
 * Conversions between ros and airsim frames. See the coordinate/frames section in the Readme/doc for more info.
 * Currently uses only rotation, no offset. The rotation setup is in ROS coordinates.
*/
class FrameConverter {
 public:
  FrameConverter();
  virtual ~FrameConverter() = default;

  void reset();
  void setupFromYaw(double yaw);  // rad
  void setupFromQuat(const Eigen::Quaterniond &quat);

  // interfaces
  void airsimToRos(geometry_msgs::Vector3* point) const;
  void airsimToRos(Eigen::Vector3d* point) const;
  void airsimToRos(geometry_msgs::Point* point) const;
  void airsimToRos(geometry_msgs::Quaternion* orientation) const;
  void airsimToRos(Eigen::Quaterniond* orientation) const;
  void airsimToRos(geometry_msgs::Pose* pose) const;
  void airsimToRos(geometry_msgs::Transform * pose) const;

  // transformations
  void transfrormPointAirsimToRos(double* x, double* y, double* z) const;
  void transfrormOrientationAirsimToRos(double* x, double* y, double* z, double* w) const;

 protected:
  Eigen::Matrix3d rotation_;
};

} // namespcae unreal_airsim

#endif //UNREAL_AIRSIM_FRAME_TRANSFORMATIONS_H_
