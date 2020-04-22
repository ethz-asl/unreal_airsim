#include "unreal_airsim/frame_transformations.h"

namespace unreal_airsim {

FrameConverter::FrameConverter() {
  reset();
}

void FrameConverter::reset() {
  rotation_ = Eigen::Matrix3d::Identity();
  rot_inverse_ = Eigen::Matrix3d::Identity();
}

void FrameConverter::setupFromYaw(double yaw) {
  double yaw_offset = std::fmod(yaw, 2.0 * M_PI);
  rotation_ = Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ());
  rot_inverse_ = rotation_.inverse();
}

void FrameConverter::setupFromQuat(const Eigen::Quaterniond &quat) {
  rotation_ = quat;
  rotation_.normalize();
  rot_inverse_ = rotation_.inverse();
}

void FrameConverter::transfrormPointAirsimToRos(double *x, double *y, double *z) const {
  // axis orientations
  Eigen::Vector3d p(*x, -*y, -*z);  // coordinate axis inversions
  p = rotation_ * p;            // rotate
  *x = p[0];
  *y = p[1];
  *z = p[2];
}

void FrameConverter::transfrormOrientationAirsimToRos(double *w, double *x, double *y, double *z) const {
  Eigen::Quaterniond q(*w, *x, -*y, -*z); // this defines a mirroring of q on the YZ-plane
  q.normalize();    // just to make sure we return a unit quaternion
  q = rotation_ * q;  // rotate
  *x = q.x();
  *y = q.y();
  *z = q.z();
  *w = q.w();
}

void FrameConverter::transfrormPointRosToAirsim(double *x, double *y, double *z) const {
  // axis orientations
  Eigen::Vector3d p(*x, *y, *z);
  p = rot_inverse_ * p;  // rotate
  *x = p[0];  // coordinate axis inversions
  *y = -p[1];
  *z = -p[2];
}

void FrameConverter::transfrormOrientationRosToAirsim(double *w, double *x, double *y, double *z) const {
  Eigen::Quaterniond q(*w, *x, *y, *z);
  q.normalize();    // just to make sure we return a unit quaternion
  q = rot_inverse_ * q;  // rotate
  *x = q.x();   // this defines a mirroring of q on the YZ-plane
  *y = -q.y();
  *z = -q.z();
  *w = q.w();
}

// Interfaces

void FrameConverter::airsimToRos(geometry_msgs::Vector3 *point) const {
  transfrormPointAirsimToRos(&(point->x), &(point->y), &(point->z));
}

void FrameConverter::airsimToRos(geometry_msgs::Point *point) const {
  transfrormPointAirsimToRos(&(point->x), &(point->y), &(point->z));
}

void FrameConverter::airsimToRos(Eigen::Vector3d *point) const {
  transfrormPointAirsimToRos(&(point->x()), &(point->y()), &(point->z()));
}

void FrameConverter::airsimToRos(geometry_msgs::Quaternion *orientation) const {
  transfrormOrientationAirsimToRos(&(orientation->w), &(orientation->x), &(orientation->y), &(orientation->z));
}

void FrameConverter::airsimToRos(Eigen::Quaterniond *orientation) const {
  transfrormOrientationAirsimToRos(&(orientation->w()), &(orientation->x()), &(orientation->y()), &(orientation->z()));
}

void FrameConverter::airsimToRos(geometry_msgs::Pose *pose) const {
  airsimToRos(&(pose->position));
  airsimToRos(&(pose->orientation));
}

void FrameConverter::airsimToRos(geometry_msgs::Transform *pose) const {
  airsimToRos(&(pose->translation));
  airsimToRos(&(pose->rotation));
}

void FrameConverter::rosToAirsim(Eigen::Vector3d *point) const {
  transfrormPointRosToAirsim(&(point->x()), &(point->y()), &(point->z()));
}

void FrameConverter::rosToAirsim(geometry_msgs::Point *point) const {
  transfrormPointRosToAirsim(&(point->x), &(point->y), &(point->z));
}

void FrameConverter::rosToAirsim(geometry_msgs::Quaternion *orientation) const {
  transfrormOrientationRosToAirsim(&(orientation->w), &(orientation->x), &(orientation->y), &(orientation->z));
}

void FrameConverter::rosToAirsim(Eigen::Quaterniond *orientation) const {
  transfrormOrientationRosToAirsim(&(orientation->w()), &(orientation->x()), &(orientation->y()), &(orientation->z()));
}

void FrameConverter::rosToAirsim(geometry_msgs::Pose *pose) const {
  rosToAirsim(&(pose->position));
  rosToAirsim(&(pose->orientation));
}

} // namespcae unreal_airsim