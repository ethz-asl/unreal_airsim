#ifndef UNREAL_AIRSIM_MATH_UTILS_H_
#define UNREAL_AIRSIM_MATH_UTILS_H_

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace math_utils {

template <typename T>
inline T rad2deg(const T radians) {
  return (radians / M_PI) * 180.0;
}

template <typename T>
inline T deg2rad(const T degrees) {
  return (degrees / 180.0) * M_PI;
}

template <typename T>
inline T wrap_to_pi(T radians) {
  int m = (int)(radians / (2 * M_PI));
  radians = radians - m * 2 * M_PI;
  if (radians > M_PI)
    radians -= 2.0 * M_PI;
  else if (radians < -M_PI)
    radians += 2.0 * M_PI;
  return radians;
}

template <typename T>
inline void wrap_to_pi_inplace(T& a) {
  a = wrap_to_pi(a);
}

template <class T>
inline T angular_dist(T from, T to) {
  wrap_to_pi_inplace(from);
  wrap_to_pi_inplace(to);
  T d = to - from;
  if (d > M_PI)
    d -= 2. * M_PI;
  else if (d < -M_PI)
    d += 2. * M_PI;
  return d;
}

inline double get_yaw_from_quat_msg(const geometry_msgs::Quaternion& quat_msg) {
  tf2::Quaternion quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(quat_msg, quat_tf);
  tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);
  return yaw;
}
}  // namespace math_utils

#endif  // UNREAL_AIRSIM_MATH_UTILS_H_