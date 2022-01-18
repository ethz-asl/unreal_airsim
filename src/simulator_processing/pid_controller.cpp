#include "unreal_airsim/simulator_processing/pid_controller.h"

namespace unreal_airsim {

bool PIDParams::load_from_rosparams(const ros::NodeHandle& nh) {
  bool found = true;

  found = nh.getParam("kp_x", kp_x) && found;
  found = nh.getParam("kp_y", kp_y) && found;
  found = nh.getParam("kp_z", kp_z) && found;
  found = nh.getParam("kp_yaw", kp_yaw) && found;

  found = nh.getParam("kd_x", kd_x) && found;
  found = nh.getParam("kd_y", kd_y) && found;
  found = nh.getParam("kd_z", kd_z) && found;
  found = nh.getParam("kd_yaw", kd_yaw) && found;

  found = nh.getParam("reached_thresh_xyz", reached_yaw_degrees) && found;
  found = nh.getParam("reached_yaw_degrees", reached_yaw_degrees) && found;

  return found;
}

bool DynamicConstraints::load_from_rosparams(const ros::NodeHandle& nh) {
  bool found = true;

  found = nh.getParam("max_vel_horz_abs", max_vel_horz_abs) && found;
  found = nh.getParam("max_vel_vert_abs", max_vel_vert_abs) && found;
  found = nh.getParam("max_yaw_rate_degree", max_yaw_rate_degree) && found;

  return found;
}

PIDPositionController::PIDPositionController(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      has_odom_(false),
      has_goal_(false),
      reached_goal_(false),
      got_goal_once_(false) {
  params_.load_from_rosparams(nh_private_);
  constraints_.load_from_rosparams(nh_);
  initialize_ros();
  reset_errors();
}

void PIDPositionController::reset_errors() {
  prev_error_.x = 0.0;
  prev_error_.y = 0.0;
  prev_error_.z = 0.0;
  prev_error_.yaw = 0.0;
}

void PIDPositionController::initialize_ros() {}

void PIDPositionController::set_yaw_position(const double x, const double y,
                                             const double z, const double yaw) {
  has_odom_ = true;
  curr_position_.x = x;
  curr_position_.y = y;
  curr_position_.z = z;
  curr_position_.yaw = yaw;
}

bool PIDPositionController::set_target(const double x, const double y,
                                       const double z, const double yaw) {
  if (!got_goal_once_) got_goal_once_ = true;

  if (has_goal_ && !reached_goal_) {
    // todo maintain array of position goals
    ROS_ERROR_STREAM(
        "[PIDPositionController] denying position goal request. I am still "
        "following the previous goal");
    return false;
  }

  if (!has_goal_) {
    target_position_.x = x;
    target_position_.y = y;
    target_position_.z = z;
    target_position_.yaw = yaw;

    // ROS_INFO_STREAM("[PIDPositionController] got goal: x=" <<
    // target_position_.x << " y=" << target_position_.y << " z=" <<
    // target_position_.z << " yaw=" << target_position_.yaw);

    // todo error checks
    // todo fill response
    has_goal_ = true;
    reached_goal_ = false;
    reset_errors();  // todo
    return true;
  }

  // Already have goal, and have reached it
  return false;
}

void PIDPositionController::check_reached_goal() {
  double diff_xyz = sqrt((target_position_.x - curr_position_.x) *
                             (target_position_.x - curr_position_.x) +
                         (target_position_.y - curr_position_.y) *
                             (target_position_.y - curr_position_.y) +
                         (target_position_.z - curr_position_.z) *
                             (target_position_.z - curr_position_.z));

  double diff_yaw =
      math_utils::angularDistance(target_position_.yaw, curr_position_.yaw);

  // todo save this in degrees somewhere to avoid repeated conversion
  if (diff_xyz < params_.reached_thresh_xyz &&
      std::abs(diff_yaw) < math_utils::degToRad(params_.reached_yaw_degrees)) {
    reached_goal_ = true;
  }
}

bool PIDPositionController::is_goal_reached() const { return reached_goal_; }

void PIDPositionController::update_control_cmd_timer_cb(
    const ros::TimerEvent& event) {
  tick();
}

void PIDPositionController::tick() {
  // todo check if odometry is too old!!
  // if no odom, don't do anything.
  if (!has_odom_) {
    ROS_ERROR_STREAM("[PIDPositionController] Waiting for odometry!");
    return;
  }

  if (has_goal_) {
    check_reached_goal();
    if (reached_goal_) {
      // ROS_INFO_STREAM("[PIDPositionController] Reached goal! Hovering at
      // position.");
      has_goal_ = false;
      // dear future self, this function doesn't return coz we need to keep on
      // actively hovering at last goal pose. don't act smart
    } else {
      // ROS_INFO_STREAM("[PIDPositionController] Moving to goal.");
    }
  }

  // only compute and send control commands for hovering / moving to pose, if we
  // received a goal at least once in the past
  if (got_goal_once_) {
    compute_control_cmd();
    enforce_dynamic_constraints();
    publish_control_cmd();
  }
}

void PIDPositionController::compute_control_cmd() {
  curr_error_.x = target_position_.x - curr_position_.x;
  curr_error_.y = target_position_.y - curr_position_.y;
  curr_error_.z = target_position_.z - curr_position_.z;
  curr_error_.yaw =
      math_utils::angularDistance(curr_position_.yaw, target_position_.yaw);

  double p_term_x = params_.kp_x * curr_error_.x;
  double p_term_y = params_.kp_y * curr_error_.y;
  double p_term_z = params_.kp_z * curr_error_.z;
  double p_term_yaw = params_.kp_yaw * curr_error_.yaw;

  double d_term_x = params_.kd_x * prev_error_.x;
  double d_term_y = params_.kd_y * prev_error_.y;
  double d_term_z = params_.kd_z * prev_error_.z;
  double d_term_yaw = params_.kp_yaw * prev_error_.yaw;

  prev_error_ = curr_error_;

  vel_cmd_.linear.x = p_term_x + d_term_x;
  vel_cmd_.linear.y = p_term_y + d_term_y;
  vel_cmd_.linear.z = p_term_z + d_term_z;
  vel_cmd_.angular.z = p_term_yaw + d_term_yaw;  // todo
}

void PIDPositionController::enforce_dynamic_constraints() {
  double vel_norm_horz = sqrt((vel_cmd_.linear.x * vel_cmd_.linear.x) +
                              (vel_cmd_.linear.y * vel_cmd_.linear.y));

  if (vel_norm_horz > constraints_.max_vel_horz_abs) {
    vel_cmd_.linear.x =
        (vel_cmd_.linear.x / vel_norm_horz) * constraints_.max_vel_horz_abs;
    vel_cmd_.linear.y =
        (vel_cmd_.linear.y / vel_norm_horz) * constraints_.max_vel_horz_abs;
  }

  if (std::fabs(vel_cmd_.linear.z) > constraints_.max_vel_vert_abs) {
    vel_cmd_.linear.z = (vel_cmd_.linear.z / std::fabs(vel_cmd_.linear.z)) *
                        constraints_.max_vel_vert_abs;
  }
  // todo yaw limits
  if (std::fabs(vel_cmd_.linear.z) > constraints_.max_yaw_rate_degree) {
    vel_cmd_.linear.z = (vel_cmd_.linear.z / std::fabs(vel_cmd_.linear.z)) *
                        constraints_.max_yaw_rate_degree;
  }
}

void PIDPositionController::publish_control_cmd() {
  airsim_vel_cmd_world_frame_pub_.publish(vel_cmd_);
}

bool PIDPositionController::get_velocity(geometry_msgs::Twist& vel) const {
  if (got_goal_once_) {
    vel = vel_cmd_;
    return true;
  }
  return false;
}

msr::airlib::YawMode PIDPositionController::getYawMode() const {
  msr::airlib::YawMode yaw_mode;
  yaw_mode.is_rate = true;
  yaw_mode.yaw_or_rate = math_utils::radToDeg(vel_cmd_.angular.z);
  return yaw_mode;
}

}  // namespace unreal_airsim
