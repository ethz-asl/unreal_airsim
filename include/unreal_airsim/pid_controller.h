#ifndef UNREAL_AIRSIM_PID_CONTROLLER_H_
#define UNREAL_AIRSIM_PID_CONTROLLER_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <common/CommonStructs.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include "unreal_airsim/math_utils.h"

namespace unreal_airsim {

class PIDParams {
 public:
  double kp_x;
  double kp_y;
  double kp_z;
  double kp_yaw;
  double kd_x;
  double kd_y;
  double kd_z;
  double kd_yaw;

  double reached_thresh_xyz;
  double reached_yaw_degrees;

  PIDParams()
      : kp_x(0.5),
        kp_y(0.5),
        kp_z(0.10),
        kp_yaw(0.4),
        kd_x(0.3),
        kd_y(0.3),
        kd_z(0.2),
        kd_yaw(0.1),
        reached_thresh_xyz(0.1),
        reached_yaw_degrees(0.1) {}

  bool load_from_rosparams(const ros::NodeHandle& nh);
};

// todo should be a common representation
struct XYZYaw {
  double x;
  double y;
  double z;
  double yaw;
};

// todo should be a common representation
class DynamicConstraints {
 public:
  double max_vel_horz_abs;  // meters/sec
  double max_vel_vert_abs;
  double max_yaw_rate_degree;

  DynamicConstraints()
      : max_vel_horz_abs(1.0),
        max_vel_vert_abs(0.5),
        max_yaw_rate_degree(10.0) {}

  bool load_from_rosparams(const ros::NodeHandle& nh);
};


class PIDPositionController {
 public:
  PIDPositionController(const ros::NodeHandle& nh,
                        const ros::NodeHandle& nh_private);

  // ROS service callbacks
  // bool local_position_goal_srv_cb(airsim_ros_pkgs::SetLocalPosition::Request&
  // request, airsim_ros_pkgs::SetLocalPosition::Response& response);
  // ROS subscriber callbacks
  //void airsim_odom_cb(const nav_msgs::Odometry& odom_msg);
  //void set_airsim_odom(const geometry_msgs::Pose& pose);
  void set_yaw_position(const double x, const double y, const double z,
                        const double yaw);
  bool set_target(const double x, const double y, const double z,
                  const double yaw);
  void update_control_cmd_timer_cb(const ros::TimerEvent& event);
  void tick();
  void reset_errors();

  void initialize_ros();
  void compute_control_cmd();
  void enforce_dynamic_constraints();
  void publish_control_cmd();
  void check_reached_goal();
  bool is_goal_reached() const;
  bool get_velocity(geometry_msgs::Twist&) const;
  msr::airlib::YawMode getYawMode() const;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  DynamicConstraints constraints_;
  PIDParams params_;
  XYZYaw target_position_;
  XYZYaw curr_position_;
  XYZYaw prev_error_;
  XYZYaw curr_error_;

  nav_msgs::Odometry curr_odom_;
  geometry_msgs::Twist vel_cmd_;
  bool reached_goal_;
  bool has_goal_;
  bool has_odom_;
  bool got_goal_once_;
  // todo check for odom msg being older than n sec

  ros::Publisher airsim_vel_cmd_world_frame_pub_;
  ros::Subscriber airsim_odom_sub_;
  ros::ServiceServer local_position_goal_srvr_;
  ros::Timer update_control_cmd_timer_;
};

}  // namespace unreal_airsim
#endif  // UNREAL_AIRSIM_PID_CONTROLLER_H_
