#include "unreal_airsim/frame_converter.h"

#include <glog/logging.h>
#include <cmath>

namespace unreal_airsim {

FrameConverter::FrameConverter() { reset(); }

void FrameConverter::reset() {
  rotation_ = Eigen::Matrix3d::Identity();
  rot_inverse_ = Eigen::Matrix3d::Identity();
}

void FrameConverter::setupFromYaw(double yaw) {
  double yaw_offset = std::fmod(yaw, 2.0 * M_PI);
  rotation_ = Eigen::AngleAxisd(yaw_offset, Eigen::Vector3d::UnitZ());
  rot_inverse_ = rotation_.inverse();
}

void FrameConverter::setupFromQuat(const Eigen::Quaterniond& quat) {
  rotation_ = quat;
  rot_inverse_ = rotation_.inverse();
}

void FrameConverter::transformPointAirsimToRos(double* x, double* y,
                                               double* z) const {
  // axis orientations
  Eigen::Vector3d p(*x, -*y, -*z);  // coordinate axis inversions
  p = rotation_ * p;                // rotate
  *x = p[0];
  *y = p[1];
  *z = p[2];
}

void FrameConverter::transformOrientationAirsimToRos(double* w, double* x,
                                                     double* y,
                                                     double* z) const {
  Eigen::Quaterniond q(*w, *x, -*y,
                       -*z);  // this defines a mirroring of q on the YZ-plane
  if (std::fabs(q.norm() - 1.0) > 1e-3) {
    LOG(WARNING) << "Received non-normalized quaternion (norm is " << q.norm()
                 << ").";
    q.normalize();
  }
  q = rotation_ * q;  // rotate
  *x = q.x();
  *y = q.y();
  *z = q.z();
  *w = q.w();
}

void FrameConverter::transformPointRosToAirsim(double* x, double* y,
                                               double* z) const {
  // axis orientations
  Eigen::Vector3d p(*x, *y, *z);
  p = rot_inverse_ * p;  // rotate
  *x = p[0];             // coordinate axis inversions
  *y = -p[1];
  *z = -p[2];
}

void FrameConverter::transformOrientationRosToAirsim(double* w, double* x,
                                                     double* y,
                                                     double* z) const {
  Eigen::Quaterniond q(*w, *x, *y, *z);
  if (std::fabs(q.norm() - 1.0) > 1e-3) {
    LOG(WARNING) << "Received non-normalized quaternion (norm is " << q.norm()
                 << ").";
    q.normalize();
  }
  q = rot_inverse_ * q;  // rotate
  *x = q.x();            // this defines a mirroring of q on the YZ-plane
  *y = -q.y();
  *z = -q.z();
  *w = q.w();
}

// Interfaces

void FrameConverter::airsimToRos(geometry_msgs::Vector3* point) const {
  transformPointAirsimToRos(&(point->x), &(point->y), &(point->z));
}

void FrameConverter::airsimToRos(geometry_msgs::Point* point) const {
  transformPointAirsimToRos(&(point->x), &(point->y), &(point->z));
}

void FrameConverter::airsimToRos(Eigen::Vector3d* point) const {
  transformPointAirsimToRos(&(point->x()), &(point->y()), &(point->z()));
}

void FrameConverter::airsimToRos(geometry_msgs::Quaternion* orientation) const {
  transformOrientationAirsimToRos(&(orientation->w), &(orientation->x),
                                  &(orientation->y), &(orientation->z));
}

void FrameConverter::airsimToRos(Eigen::Quaterniond* orientation) const {
  transformOrientationAirsimToRos(&(orientation->w()), &(orientation->x()),
                                  &(orientation->y()), &(orientation->z()));
}

void FrameConverter::airsimToRos(geometry_msgs::Pose* pose) const {
  airsimToRos(&(pose->position));
  airsimToRos(&(pose->orientation));
}

void FrameConverter::airsimToRos(geometry_msgs::Transform* pose) const {
  airsimToRos(&(pose->translation));
  airsimToRos(&(pose->rotation));
}

void FrameConverter::rosToAirsim(Eigen::Vector3d* point) const {
  transformPointRosToAirsim(&(point->x()), &(point->y()), &(point->z()));
}

void FrameConverter::rosToAirsim(geometry_msgs::Point* point) const {
  transformPointRosToAirsim(&(point->x), &(point->y), &(point->z));
}

void FrameConverter::rosToAirsim(geometry_msgs::Vector3* vector) const {
  transformPointRosToAirsim(&(vector->x), &(vector->y), &(vector->z));
}

void FrameConverter::rosToAirsim(geometry_msgs::Quaternion* orientation) const {
  transformOrientationRosToAirsim(&(orientation->w), &(orientation->x),
                                  &(orientation->y), &(orientation->z));
}

void FrameConverter::rosToAirsim(Eigen::Quaterniond* orientation) const {
  transformOrientationRosToAirsim(&(orientation->w()), &(orientation->x()),
                                  &(orientation->y()), &(orientation->z()));
}

void FrameConverter::rosToAirsim(geometry_msgs::Pose* pose) const {
  rosToAirsim(&(pose->position));
  rosToAirsim(&(pose->orientation));
}

void FrameConverter::rosToAirsim(geometry_msgs::Transform* pose) const {
  rosToAirsim(&(pose->translation));
  rosToAirsim(&(pose->rotation));
}

}  // namespace unreal_airsim
