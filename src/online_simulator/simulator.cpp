#include "unreal_airsim/online_simulator/simulator.h"

#include <memory>
#include <string>
#include <vector>

#include "unreal_airsim/simulator_processing/processor_factory.h"

STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif  // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <minkindr_conversions/kindr_msg.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf/transform_datatypes.h>

#include <glog/logging.h>

#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <thread>

namespace unreal_airsim {

// bool PIDParams::load_from_rosparams(const ros::NodeHandle& nh)
// {
//     bool found = true;

//     found = found && nh.getParam("kp_x", kp_x);
//     found = found && nh.getParam("kp_y", kp_y);
//     found = found && nh.getParam("kp_z", kp_z);
//     found = found && nh.getParam("kp_yaw", kp_yaw);

//     found = found && nh.getParam("kd_x", kd_x);
//     found = found && nh.getParam("kd_y", kd_y);
//     found = found && nh.getParam("kd_z", kd_z);
//     found = found && nh.getParam("kd_yaw", kd_yaw);

//     found = found && nh.getParam("reached_thresh_xyz", reached_thresh_xyz);
//     found = found && nh.getParam("reached_yaw_degrees", reached_yaw_degrees);

//     return found;
// }

// bool DynamicConstraints::load_from_rosparams(const ros::NodeHandle& nh)
// {
//     bool found = true;

//     found = found && nh.getParam("max_vel_horz_abs", max_vel_horz_abs);
//     found = found && nh.getParam("max_vel_vert_abs", max_vel_vert_abs);
//     found = found && nh.getParam("max_yaw_rate_degree", max_yaw_rate_degree);

//     return found;
// }

// PIDPositionController::PIDPositionController(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
//     : nh_(nh), nh_private_(nh_private), has_odom_(false), has_goal_(false), reached_goal_(false), got_goal_once_(false)
// {
//     params_.load_from_rosparams(nh_private_);
//     constraints_.load_from_rosparams(nh_);
//     initialize_ros();
//     reset_errors();
// }

// void PIDPositionController::reset_errors()
// {
//     prev_error_.x = 0.0;
//     prev_error_.y = 0.0;
//     prev_error_.z = 0.0;
//     prev_error_.yaw = 0.0;
// }

// void PIDPositionController::initialize_ros()
// {
//     //vel_cmd_ = airsim_ros_pkgs::VelCmd();
//     // ROS params
//     // double update_control_every_n_sec;
//     // nh_private_.getParam("update_control_every_n_sec", update_control_every_n_sec);

//     // ROS publishers
//     // airsim_vel_cmd_world_frame_pub_ = nh_private_.advertise<geometry_msgs::Twist>("/vel_cmd_world_frame", 1);

//     // ROS subscribers
//     //airsim_odom_sub_ = nh_.subscribe("/airsim_drone/ground_truth/odometry", 50, &PIDPositionController::airsim_odom_cb, this);
//     // todo publish this under global nodehandle / "airsim node" and hide it from user
//     //local_position_goal_srvr_ = nh_.advertiseService("/local_position_goal", &PIDPositionController::local_position_goal_srv_cb, this);
    
//     // ROS timers
//     //update_control_cmd_timer_ = nh_private_.createTimer(ros::Duration(update_control_every_n_sec), &PIDPositionController::update_control_cmd_timer_cb, this);
// }

// // void PIDPositionController::airsim_odom_cb(const nav_msgs::Odometry& odom_msg)
// // {
// //     has_odom_ = true;
// //     curr_odom_ = odom_msg;
// //     curr_position_.x = odom_msg.pose.pose.position.x;
// //     curr_position_.y = odom_msg.pose.pose.position.y;
// //     curr_position_.z = odom_msg.pose.pose.position.z;
// //     curr_position_.yaw = math_utils::get_yaw_from_quat_msg(odom_msg.pose.pose.orientation);
// // }

// // void PIDPositionController::airsim_odom(const geometry_msgs::Pose &pose)
// // {
// //     has_odom_ = true;
// //     curr_odom_ = odom_msg;
// //     curr_position_.x = pose.position.x;
// //     curr_position_.y = pose.position.y;
// //     curr_position_.z = pose.position.z;
// //     curr_position_.yaw = math_utils::get_yaw_from_quat_msg(pose.orientation);
// // }

// void PIDPositionController::set_yaw_position(const double x, const double y, const double z, const double yaw)
// {
//     has_odom_ = true;
//     curr_position_.x = x;
//     curr_position_.y = y;
//     curr_position_.z = z;
//     curr_position_.yaw = yaw;
// }

// bool PIDPositionController::set_target(const double x, const double y, const double z, const double yaw)
// {
//     if (!got_goal_once_)
//         got_goal_once_ = true;

//     if (has_goal_ && !reached_goal_) {
//         // todo maintain array of position goals
//         ROS_ERROR_STREAM("[PIDPositionController] denying position goal request. I am still following the previous goal");
//         return false;
//     }

//     if (!has_goal_) {
//         target_position_.x = x;
//         target_position_.y = y;
//         target_position_.z = z;
//         target_position_.yaw = yaw;

//         ROS_INFO_STREAM("[PIDPositionController] got goal: x=" << target_position_.x << " y=" << target_position_.y << " z=" << target_position_.z << " yaw=" << target_position_.yaw);

//         // todo error checks
//         // todo fill response
//         has_goal_ = true;
//         reached_goal_ = false;
//         reset_errors(); // todo
//         return true;
//     }

//     // Already have goal, and have reached it
//     ROS_INFO_STREAM("[PIDPositionController] Already have goal and have reached it");
//     return false;
// }

// bool PIDPositionController::check_reached_goal()
// {
//     double diff_xyz = sqrt((target_position_.x - curr_position_.x) * (target_position_.x - curr_position_.x) + (target_position_.y - curr_position_.y) * (target_position_.y - curr_position_.y) + (target_position_.z - curr_position_.z) * (target_position_.z - curr_position_.z));

//     double diff_yaw = math_utils::angular_dist(target_position_.yaw, curr_position_.yaw);

//     // todo save this in degrees somewhere to avoid repeated conversion
//     if (diff_xyz < params_.reached_thresh_xyz && diff_yaw < math_utils::deg2rad(params_.reached_yaw_degrees))
//         reached_goal_ = true;
//         return true;

//   return false;
// }

// void PIDPositionController::update_control_cmd_timer_cb(const ros::TimerEvent& event)
// {
//     tick();
// }

// void PIDPositionController::tick()
// {
//     // todo check if odometry is too old!!
//     // if no odom, don't do anything.
//     if (!has_odom_) {
//         ROS_ERROR_STREAM("[PIDPositionController] Waiting for odometry!");
//         return;
//     }

//     if (has_goal_) {
//         check_reached_goal();
//         if (reached_goal_) {
//             ROS_INFO_STREAM("[PIDPositionController] Reached goal! Hovering at position.");
//             has_goal_ = false;
//             // dear future self, this function doesn't return coz we need to keep on actively hovering at last goal pose. don't act smart
//         }
//         else {
//             ROS_INFO_STREAM("[PIDPositionController] Moving to goal.");
//         }
//     }

//     // only compute and send control commands for hovering / moving to pose, if we received a goal at least once in the past
//     if (got_goal_once_) {
//         compute_control_cmd();
//         enforce_dynamic_constraints();
//         publish_control_cmd();
//     }
// }

// void PIDPositionController::compute_control_cmd()
// {
//     curr_error_.x = target_position_.x - curr_position_.x;
//     curr_error_.y = target_position_.y - curr_position_.y;
//     curr_error_.z = target_position_.z - curr_position_.z;
//     curr_error_.yaw = math_utils::angular_dist(curr_position_.yaw, target_position_.yaw);

//     double p_term_x = params_.kp_x * curr_error_.x;
//     double p_term_y = params_.kp_y * curr_error_.y;
//     double p_term_z = params_.kp_z * curr_error_.z;
//     double p_term_yaw = params_.kp_yaw * curr_error_.yaw;

//     double d_term_x = params_.kd_x * prev_error_.x;
//     double d_term_y = params_.kd_y * prev_error_.y;
//     double d_term_z = params_.kd_z * prev_error_.z;
//     double d_term_yaw = params_.kp_yaw * prev_error_.yaw;

//     prev_error_ = curr_error_;

//     vel_cmd_.linear.x = p_term_x + d_term_x;
//     vel_cmd_.linear.y = p_term_y + d_term_y;
//     vel_cmd_.linear.z = p_term_z + d_term_z;
//     vel_cmd_.angular.z = p_term_yaw + d_term_yaw; // todo
// }

// void PIDPositionController::enforce_dynamic_constraints()
// {
//     double vel_norm_horz = sqrt((vel_cmd_.linear.x * vel_cmd_.linear.x) + (vel_cmd_.linear.y * vel_cmd_.linear.y));

//     if (vel_norm_horz > constraints_.max_vel_horz_abs) {
//         vel_cmd_.linear.x = (vel_cmd_.linear.x / vel_norm_horz) * constraints_.max_vel_horz_abs;
//         vel_cmd_.linear.y = (vel_cmd_.linear.y / vel_norm_horz) * constraints_.max_vel_horz_abs;
//     }

//     if (std::fabs(vel_cmd_.linear.z) > constraints_.max_vel_vert_abs) {
//         // todo just add a sgn funciton in common utils? return double to be safe.
//         // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
//         vel_cmd_.linear.z = (vel_cmd_.linear.z / std::fabs(vel_cmd_.linear.z)) * constraints_.max_vel_vert_abs;
//     }
//     // todo yaw limits
//     if (std::fabs(vel_cmd_.linear.z) > constraints_.max_yaw_rate_degree) {
//         // todo just add a sgn funciton in common utils? return double to be safe.
//         // template <typename T> double sgn(T val) { return (T(0) < val) - (val < T(0)); }
//         vel_cmd_.linear.z = (vel_cmd_.linear.z / std::fabs(vel_cmd_.linear.z)) * constraints_.max_yaw_rate_degree;
//     }
// }

// // todo - replace with a airsim_move_client_.moveByVelocityAsync(
// void PIDPositionController::publish_control_cmd()
// {
//     airsim_vel_cmd_world_frame_pub_.publish(vel_cmd_);
// }

// bool PIDPositionController::get_velocity(geometry_msgs::Twist& vel) const
// {
//   if(got_goal_once_) {
//     vel = vel_cmd_;
//     return true;
//   }
//   return false;
// }

// msr::airlib::YawMode PIDPositionController::getYawMode() const {
//   msr::airlib::YawMode yaw_mode;
//   yaw_mode.is_rate = true;
//   yaw_mode.yaw_or_rate = math_utils::rad2deg(vel_cmd_.angular.z);
//   return yaw_mode;
// }

AirsimSimulator::AirsimSimulator(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      is_connected_(false),
      is_running_(false),
      is_shutdown_(false),
      followingGoal(false),
      only_move_in_yaw_direction(true),
      goal_yaw(0),
      has_goal_(false),
      pid_controller_(nh, nh_private),
      odometry_drift_simulator_(
          OdometryDriftSimulator::Config::fromRosParams(nh_private)) {
  // configure
  readParamsFromRos();

  // airsim
  bool success = setupAirsim();
  if (success) {
    LOG(INFO) << "Connected to the Airsim Server.";
    is_connected_ = true;
  } else {
    std::raise(SIGINT);
    return;
  }

  // setup everything
  initializeSimulationFrame();
  setupROS();
  LOG(INFO) << "setup ROS";
  startSimTimer();
  LOG(INFO) << "setting up timer.";
  // Startup the vehicle simulation via callback
  startup_timer_ = nh_private_.createTimer(
      ros::Duration(0.1), &AirsimSimulator::startupCallback, this);
  LOG(INFO) << "Setup timer.";
}

bool AirsimSimulator::readParamsFromRos() {
  AirsimSimulator::Config defaults;

  // params
  nh_.param("/use_sim_time", use_sim_time_, false);
  nh_private_.param("state_refresh_rate", config_.state_refresh_rate,
                    defaults.state_refresh_rate);
  nh_private_.param("time_publisher_interval", config_.time_publisher_interval,
                    defaults.time_publisher_interval);
  nh_private_.param("simulator_frame_name", config_.simulator_frame_name,
                    defaults.simulator_frame_name);
  nh_private_.param("vehicle_name", config_.vehicle_name,
                    defaults.vehicle_name);
  nh_private_.param("velocity", config_.velocity, defaults.velocity);
  nh_private_.param("publish_sensor_transforms",
                    config_.publish_sensor_transforms,
                    defaults.publish_sensor_transforms);

  // Verify params valid
  if (config_.state_refresh_rate <= 0.0) {
    config_.state_refresh_rate = defaults.state_refresh_rate;
    LOG(WARNING) << "Param 'state_refresh_rate' expected > 0.0, set to '"
                 << defaults.state_refresh_rate << "' (default).";
  }
  if (config_.time_publisher_interval < 0) {
    config_.time_publisher_interval = defaults.time_publisher_interval;
    LOG(WARNING) << "Param 'time_publisher_interval' expected >= 0, set to '"
                 << defaults.time_publisher_interval << "' (default).";
  }
  if (config_.velocity <= 0.0) {
    config_.velocity = defaults.velocity;
    LOG(WARNING) << "Param 'velocity' expected > 0.0, set to '"
                 << defaults.velocity << "' (default).";
  }

  // setup sensors
  std::vector<std::string> keys;
  nh_private_.getParamNames(keys);
  std::string sensor_ns = "sensors/";
  std::string full_ns = nh_private_.getNamespace() + "/" + sensor_ns;
  std::string sensor_name;
  std::vector<std::string> sensors;
  size_t pos;
  for (auto const& key : keys) {
    if ((pos = key.find(full_ns)) != std::string::npos) {
      sensor_name = key;
      sensor_name.erase(0, pos + full_ns.length());
      pos = sensor_name.find('/');
      if (pos != std::string::npos) {
        sensor_name = sensor_name.substr(0, pos);
      }
      if (std::find(sensors.begin(), sensors.end(), sensor_name) ==
          sensors.end()) {
        sensors.push_back(sensor_name);
      }
    }
  }
  for (auto const& name : sensors) {
    // currently pass all settings via params, maybe could add some smart
    // identification here
    if (!nh_private_.hasParam(sensor_ns + name + "/sensor_type")) {
      LOG(WARNING) << "Sensor '" << name
                   << "' has no sensor_type and will be ignored!";
      continue;
    }
    Config::Sensor* sensor_cfg;
    std::string sensor_type;
    nh_private_.getParam(sensor_ns + name + "/sensor_type", sensor_type);

    if (sensor_type == Config::Sensor::TYPE_CAMERA) {
      auto* cfg = new Config::Camera();
      Config::Camera cam_defaults;
      nh_private_.param(sensor_ns + name + "/pixels_as_float",
                        cfg->pixels_as_float, cfg->pixels_as_float);
      // cam types default to visual (Scene) camera, but make sure if something
      // else is intended a warning is thrown
      std::string read_img_type_default = "Param is not string";
      if (!nh_private_.hasParam(sensor_ns + name + "/image_type")) {
        read_img_type_default = cam_defaults.image_type_str;
      }
      nh_private_.param(sensor_ns + name + "/image_type", cfg->image_type_str,
                        read_img_type_default);
      if (cfg->image_type_str == "Scene") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::Scene;
      } else if (cfg->image_type_str == "DepthPerspective") {
        cfg->image_type =
            msr::airlib::ImageCaptureBase::ImageType::DepthPerspective;
      } else if (cfg->image_type_str == "DepthPlanar") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::DepthPlanar;
      } else if (cfg->image_type_str == "DepthVis") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::DepthVis;
      } else if (cfg->image_type_str == "DisparityNormalized") {
        cfg->image_type =
            msr::airlib::ImageCaptureBase::ImageType::DisparityNormalized;
      } else if (cfg->image_type_str == "SurfaceNormals") {
        cfg->image_type =
            msr::airlib::ImageCaptureBase::ImageType::SurfaceNormals;
      } else if (cfg->image_type_str == "Segmentation") {
        cfg->image_type =
            msr::airlib::ImageCaptureBase::ImageType::Segmentation;
      } else if (cfg->image_type_str == "Infrared") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::Infrared;
      } else {
        LOG(WARNING) << "Unrecognized ImageType '" << cfg->image_type_str
                     << "' for camera '" << name << "', set to '"
                     << cam_defaults.image_type_str << "' (default).";
        cfg->image_type = cam_defaults.image_type;
        cfg->image_type_str = cam_defaults.image_type_str;
      }
      cfg->camera_info = airsim_state_client_.simGetCameraInfo(name);
      sensor_cfg = (Config::Sensor*)cfg;
    } else if (sensor_type == Config::Sensor::TYPE_LIDAR) {
      sensor_cfg = new Config::Sensor();
    } else if (sensor_type == Config::Sensor::TYPE_IMU) {
      sensor_cfg = new Config::Sensor();
    } else {
      LOG(WARNING) << "Unknown sensor_type '" << sensor_type << "' for sensor '"
                   << name << "', sensor will be ignored!";
      continue;
    }

    // general settings
    sensor_cfg->name = name;
    sensor_cfg->sensor_type = sensor_type;
    nh_private_.param(sensor_ns + name + "/output_topic",
                      sensor_cfg->output_topic,
                      config_.vehicle_name + "/" + name);
    nh_private_.param(sensor_ns + name + "/frame_name", sensor_cfg->frame_name,
                      config_.vehicle_name + "/" + name);
    nh_private_.param(sensor_ns + name + "/force_separate_timer",
                      sensor_cfg->force_separate_timer,
                      sensor_cfg->force_separate_timer);
    double rate;
    nh_private_.param(sensor_ns + name + "/rate", rate, sensor_cfg->rate);
    if (rate <= 0) {
      LOG(WARNING) << "Param 'rate' for sensor '" << name
                   << "' expected > 0.0, set to '" << sensor_cfg->rate
                   << "' (default).";
    } else {
      sensor_cfg->rate = rate;
    }
    readTransformFromRos(sensor_ns + name + "/T_B_S",
                         &(sensor_cfg->translation), &(sensor_cfg->rotation));
    config_.sensors.push_back(std::unique_ptr<Config::Sensor>(sensor_cfg));
  }
  return true;
}

bool AirsimSimulator::setupAirsim() {
  // This is implemented explicitly to avoid Airsim printing and make it clearer
  // for us what is going wrong
  int timeout = 0;
  while (airsim_state_client_.getConnectionState() !=
             RpcLibClientBase::ConnectionState::Connected &&
         ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    timeout++;
    if (timeout > 10) {
      // connection state will remain RpcLibClientBase::ConnectionState::Initial
      // if the unreal game was not running when creating the client (in the
      // constructor)
      LOG(FATAL)
          << "Unable to connect to the Airsim Server (timeout after 5s). "
             "Is a UE4 game with enabled Airsim plugin running?";
      return false;
    }
  }

  // check versions
  bool versions_matching =
      true;  // Check both in one run to spare running into the issue twice
  int server_ver = 0;
  try {
    server_ver = airsim_state_client_.getServerVersion();
  } catch (rpc::rpc_error& e) {
    LOG(FATAL) << "Could not get server version from AirSim Plugin: "
               << e.get_error().as<std::string>();
    return false;
  }
  int client_ver = airsim_state_client_.getClientVersion();
  int server_min_ver = airsim_state_client_.getMinRequiredServerVersion();
  int client_min_ver = airsim_state_client_.getMinRequiredClientVersion();
  if (client_ver < client_min_ver) {
    LOG(FATAL) << "Airsim Client version is too old (is: " << client_ver
               << ", min: " << client_min_ver
               << "). Update and rebuild the Airsim library.";
    versions_matching = false;
  }
  if (server_ver < server_min_ver) {
    LOG(FATAL) << "Airsim Server version is too old (is: " << server_ver
               << ", min: " << server_min_ver
               << "). Update and rebuild the Airsim UE4 Plugin.";
    versions_matching = false;
  }
  return versions_matching;
}

bool AirsimSimulator::setupROS() {
  // General
  sim_state_timer_ =
      nh_.createTimer(ros::Duration(1.0 / config_.state_refresh_rate),
                      &AirsimSimulator::simStateCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(
      config_.vehicle_name + "/ground_truth/odometry", 5);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(
      config_.vehicle_name + "/ground_truth/pose", 5);
  collision_pub_ =
      nh_.advertise<std_msgs::Bool>(config_.vehicle_name + "/collision", 1);
  sim_is_ready_pub_ = nh_.advertise<std_msgs::Bool>("simulation_is_ready", 1);
  if (use_sim_time_) {
    time_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 50);
  }

  // control interfaces
  command_pose_sub_ =
      nh_.subscribe(config_.vehicle_name + "/command/pose", 10,
                    &AirsimSimulator::commandPoseCallback, this);

  command_trajectory_sub_ =
      nh_.subscribe(config_.vehicle_name + "/command/trajectory", 10,
                    &AirsimSimulator::commandTrajectorycallback, this);

  // sensors
  for (size_t i = 0; i < config_.sensors.size(); ++i) {
    // Find or allocate the sensor timer
    SensorTimer* timer = nullptr;
    if (!config_.sensors[i]->force_separate_timer) {
      for (const auto& t : sensor_timers_) {
        if (!t->isPrivate() && t->getRate() == config_.sensors[i]->rate) {
          timer = t.get();
          break;
        }
      }
    }
    if (timer == nullptr) {
      sensor_timers_.push_back(std::make_unique<SensorTimer>(
          nh_, config_.sensors[i]->rate,
          config_.sensors[i]->force_separate_timer, config_.vehicle_name,
          this));
      timer = sensor_timers_.back().get();
    }
    timer->addSensor(*this, i);

    // Save camera params (e.g. FOV) as they are needed to generate pointcloud
    if (config_.sensors[i]->sensor_type == Config::Sensor::TYPE_CAMERA) {
      auto camera = (Config::Camera*)config_.sensors[i].get();
      // This assumes the camera exists, which should always be the case with
      // the auto-generated-config.
      camera->camera_info = airsim_move_client_.simGetCameraInfo(
              camera->name, config_.vehicle_name);
      // TODO(Schmluk): Might want to also publish the camera info or convert
      // to intrinsics etc
     }

    if (!config_.publish_sensor_transforms) {
      // Broadcast all sensor mounting transforms via static tf.
      geometry_msgs::TransformStamped static_transformStamped;
      Eigen::Quaterniond rotation = config_.sensors[i]->rotation;
      if (config_.sensors[i]->sensor_type == Config::Sensor::TYPE_CAMERA) {
        // Camera frames are x right, y down, z depth
        rotation = Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5) * rotation;
      }
      static_transformStamped.header.stamp = ros::Time::now();
      static_transformStamped.header.frame_id = config_.vehicle_name;
      static_transformStamped.child_frame_id = config_.sensors[i]->frame_name;
      static_transformStamped.transform.translation.x =
          config_.sensors[i]->translation.x();
      static_transformStamped.transform.translation.y =
          config_.sensors[i]->translation.y();
      static_transformStamped.transform.translation.z =
          config_.sensors[i]->translation.z();
      static_transformStamped.transform.rotation.x = rotation.x();
      static_transformStamped.transform.rotation.y = rotation.y();
      static_transformStamped.transform.rotation.z = rotation.z();
      static_transformStamped.transform.rotation.w = rotation.w();
      static_tf_broadcaster_.sendTransform(static_transformStamped);
    }
  }

  // Simulator processors (find names and let them create themselves)
  std::vector<std::string> keys;
  nh_private_.getParamNames(keys);
  std::string proc_ns = "processors/";
  std::string full_ns = nh_private_.getNamespace() + "/" + proc_ns;
  std::string proc_name;
  std::vector<std::string> processors;
  size_t pos;
  for (auto const& key : keys) {
    if ((pos = key.find(full_ns)) != std::string::npos) {
      proc_name = key;
      proc_name.erase(0, pos + full_ns.length());
      pos = proc_name.find('/');
      if (pos != std::string::npos) {
        proc_name = proc_name.substr(0, pos);
      }
      if (std::find(processors.begin(), processors.end(), proc_name) ==
          processors.end()) {
        processors.push_back(proc_name);
      }
    }
  }
  for (auto const& name : processors) {
    if (!nh_private_.hasParam(full_ns + name + "/processor_type")) {
      LOG(ERROR) << "Sensor processor '" << name
                 << "' does not name a 'processor_type' and will be ignored.";
      continue;
    }
    std::string type;
    nh_private_.getParam(full_ns + name + "/processor_type", type);
    processors_.push_back(simulator_processor::ProcessorFactory::createFromRos(
        name, type, nh_, full_ns + name + "/", this));
  }
  return true;
}

bool AirsimSimulator::initializeSimulationFrame() {
  if (is_shutdown_) {
    return false;
  }
  // For frame conventions see coords/frames section in the readme/doc
  msr::airlib::Pose pose =
      airsim_state_client_.simGetVehiclePose(config_.vehicle_name);
  Eigen::Quaterniond ori(pose.orientation.w(), pose.orientation.x(),
                         pose.orientation.y(), pose.orientation.z());
  Eigen::Vector3d euler = ori.toRotationMatrix().eulerAngles(
      2, 1, 0);  // yaw-pitch-roll in airsim coords
  double yaw = euler[0];
  /**
   * NOTE(schmluk): make the coordinate rotation snap to full 45 degree
   * rotations, wrong initializations would induce large errors for the
   * evaluation of the constructed map.
   */
  const double kSnappingAnglesDegrees = 45;
  const double kSnappingRangeDegrees = 10;
  double snapping_angle = kSnappingAnglesDegrees / 180.0 * M_PI;
  double diff = fmod(yaw, snapping_angle);
  if (diff > snapping_angle / 2.0) {
    diff -= snapping_angle;
  }
  if (abs(diff) < kSnappingRangeDegrees / 180.0 * M_PI) {
    yaw -= diff;
  }
  frame_converter_.setupFromYaw(yaw);
  return true;
}

// Callback for trajectories published by active plner
void AirsimSimulator::commandTrajectorycallback(
    const trajectory_msgs::MultiDOFJointTrajectory trajectory) {
  if (!is_running_) {
    return;
  }
  for (const auto& point : trajectory.points) {
    for (auto pose : point.transforms) {
      std::cout << "adding pose to goal" << pose.rotation.y << " "
                << pose.rotation.w << std::endl;

      OdometryDriftSimulator::Transformation T_drift_command;
      tf::transformMsgToKindr(pose, &T_drift_command);
      const OdometryDriftSimulator::Transformation T_gt_command =
          odometry_drift_simulator_.convertDriftedToGroundTruthPose(T_drift_command);

      geometry_msgs::Transform transform;
      tf::transformKindrToMsg(T_gt_command, &transform);
      frame_converter_.rosToAirsim(&transform);

      auto* pt = new Waypoint;
      pt->pose = transform;
      pt->isGoal = false;
      points.push(pt);
    }
  }
  // Add goal point for yaw tracking as last point
  auto* lastPose = points.back();
  auto* pt = new Waypoint;
  pt->pose.translation.x = lastPose->pose.translation.x;
  pt->pose.translation.y = lastPose->pose.translation.y;
  pt->pose.translation.z = lastPose->pose.translation.z;

  pt->pose.rotation.y = lastPose->pose.rotation.y;
  pt->pose.rotation.x = lastPose->pose.rotation.x;
  pt->pose.rotation.z = lastPose->pose.rotation.z;
  pt->pose.rotation.w = lastPose->pose.rotation.w;
  pt->isGoal = true;
  points.push(pt);

  std::cout << "adding goal point" << pt->pose.rotation.y << " "
            << pt->pose.rotation.w << std::endl;
}

void AirsimSimulator::commandPoseCallback(const geometry_msgs::Pose& msg) {
  if (!is_running_) {
    return;
  }

  // Input pose is in drifting odom frame, we therefore
  // first convert it back into Unreal GT frame
  OdometryDriftSimulator::Transformation T_drift_command;
  tf::poseMsgToKindr(msg, &T_drift_command);
  const OdometryDriftSimulator::Transformation T_gt_command =
      odometry_drift_simulator_.convertDriftedToGroundTruthPose(
          T_drift_command);
  OdometryDriftSimulator::Transformation::Position t_gt_current_position =
      odometry_drift_simulator_.getGroundTruthPose().getPosition();

  // Use position + yaw as setpoint
  auto command_pos = T_gt_command.getPosition();
  auto command_ori = T_gt_command.getEigenQuaternion();
  frame_converter_.rosToAirsim(&command_ori);
  double yaw = tf2::getYaw(
      tf2::Quaternion(command_ori.x(), command_ori.y(), command_ori.z(),
                      command_ori.w()));  // Eigen's eulerAngles apparently
  // messes up some wrap arounds or direcions and gets the wrong yaws in some
  // cases
  //config_.velocity =0.2f;
  yaw = yaw / M_PI * 180.0;
  constexpr double kMinMovingDistance = 0.1;  // m
  airsim_move_client_.cancelLastTask();
  if ((command_pos - t_gt_current_position).norm() >= kMinMovingDistance) {
    frame_converter_.rosToAirsim(&command_pos);
    auto yaw_mode = msr::airlib::YawMode(false, yaw);
    float look_ahead=-1, adaptive_lookahead=0;
    airsim_move_client_.moveToPositionAsync(
        command_pos.x(), command_pos.y(), command_pos.z(), config_.velocity,
        3600, config_.drive_train_type, yaw_mode, look_ahead,adaptive_lookahead, config_.vehicle_name);
        std::cout <<"Moving to target position with "<<look_ahead << " lookahead and "<<adaptive_lookahead <<" as adaptive lookahead and velocity "<<config_.velocity<<std::endl;
  } else {
    // This second command catches the case if the total distance is too small,
    // where the moveToPosition command returns without satisfying the yaw. If
    // this is always run then apparently sometimes the move command is
    // overwritten.
    airsim_move_client_.rotateToYawAsync(yaw, 3600, 5, config_.vehicle_name);
    std::cout <<"No moving, only rotating"<<std::endl;
  }
}

void AirsimSimulator::startupCallback(const ros::TimerEvent&) {
  // Startup the drone, this should set the MAV hovering at 'PlayerStart' in
  // unreal
  startup_timer_.stop();
  airsim_move_client_.enableApiControl(
      true);  // Also disables user control, which is good
  airsim_move_client_.armDisarm(true);
  airsim_move_client_.takeoffAsync(2)->waitOnLastTask();
  airsim_move_client_.moveToPositionAsync(0, 0, 0, 5)->waitOnLastTask();
  is_running_ = true;
  std_msgs::Bool msg;
  msg.data = true;
  sim_is_ready_pub_.publish(msg);
  odometry_drift_simulator_.start();
  LOG(INFO) << "Airsim simulation is ready!";
}

bool AirsimSimulator::startSimTimer() {
  /***
   * NOTE(schmluk): Because, although airsim and ros both use the system clock
   * as default for time-stamping according to their docs, these are drifting
   * clocks! If use_sim_time is set in the launch file, we just use airsim time
   * as ros time and publish it at a fixed wall-time frequency (see param
   * time_publisher_interval).
   */
  if (use_sim_time_) {
    std::thread([this]() {
      while (is_connected_) {
        auto next = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(config_.time_publisher_interval);
        readSimTimeCallback();
        if (next > std::chrono::steady_clock::now()) {
          std::this_thread::sleep_until(next);
        }
      }
    })
        .detach();
  }
  return true;
}

void AirsimSimulator::readSimTimeCallback() {
  /**
   * TODO(schmluk): make this nice.
   * This is currently a work-around as getting only the time stamp is not yet
   * exposed. However, this call does not run on the game thread afaik and was
   * not measured to slow down other tasks. Although this queries sim time via
   * RPC call, it can run at ~4000 Hz so delay should be <1 ms.
   */
  uint64_t ts =
      airsim_time_client_.getMultirotorState(config_.vehicle_name).timestamp;
  rosgraph_msgs::Clock msg;
  msg.clock.fromNSec(ts);
  time_pub_.publish(msg);
}

ros::Time AirsimSimulator::getTimeStamp(msr::airlib::TTimePoint airsim_stamp) {
  if (use_sim_time_ && airsim_stamp > 0) {
    ros::Time t;
    t.fromNSec(airsim_stamp);
    return t;
  } else {
    return ros::Time::now();
  }
}

void AirsimSimulator::trackWayPoints() {
  // if there are waypoints that need to be followed
  // send desired velocity commands to PID Ccontroller
  // to follow the waypoints
  if (!points.empty()) {
    // just found a new waypoint. After
    // setting it as target

    // get current position
    geometry_msgs::Pose pose;
    tf::poseKindrToMsg(odometry_drift_simulator_.getSimulatedPose(), &pose);
    frame_converter_.rosToAirsim(&pose);
    auto current_position = pose.position;

    // updation current position for pid controller
    double current_yaw = math_utils::get_yaw_from_quat_msg(pose.orientation);
    pid_controller_.set_yaw_position(current_position.x, current_position.y,
                                       current_position.z, current_yaw);

    if (!followingGoal) {
      // Get next point to track
      current_goal = points.front();
      auto target_position = current_goal->pose.translation;

      double yaw;

      if (only_move_in_yaw_direction && !current_goal->isGoal) {
        // We are currently tracking waypoints and not final goalpoints
        // If only move in yaw direction, set yaw of waypoints (except goal
        // point) to moving direction
        auto delta_x = target_position.x - current_position.x;
        auto delta_y = target_position.y - current_position.y;
        if (abs(delta_x) + abs(delta_y) <= 0.1) {
          // If we overshoot over the goalpoint, the robot will rotate 180 deg
          // and drive back for a really small amount. Check if overshoot
          // occured (delta really small) and don't turn around in this case
          yaw = goal_yaw;
        } else {
          yaw = atan2(delta_y, delta_x);
        }

        std::cout << "forcing yaw in view direction" << std::endl;
      } else {
        // Rotate to goal yaw
        tf::Quaternion q(
            current_goal->pose.rotation.x, current_goal->pose.rotation.y,
            current_goal->pose.rotation.z, current_goal->pose.rotation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw);
      }
      goal_yaw = yaw;

      // goal_received=true
      
      if (pid_controller_.set_target(target_position.x, target_position.y,
                                     target_position.z, goal_yaw)) {
        followingGoal = true;
      }

    } else {  // already following a goal
      bool goal_reached = pid_controller_.is_goal_reached();
      
      if (goal_reached) {
        if (current_goal->isGoal) {
          // Reached Goal
          std::cout << "reached a goal point" << std::endl;

        } else {
          std::cout << "reached a way point" << std::endl;
        }

        // move to next way point
        auto* off = points.front();
        points.pop();
        delete off;
        followingGoal = false;
      }
    }

    pid_controller_.tick();

    geometry_msgs::Twist vel_cmd;
    if (pid_controller_.get_velocity(vel_cmd)) {
      double vel_cmd_duration = 1.0 / config_.state_refresh_rate;
      // move the drone
      airsim_move_client_.moveByVelocityAsync(
          vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z, vel_cmd_duration,
          msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
          pid_controller_.getYawMode(), config_.vehicle_name);
    }
  }
}

void AirsimSimulator::simStateCallback(const ros::TimerEvent&) {
  if (is_shutdown_) {
    return;
  }
  if (airsim_state_client_.getConnectionState() !=
      RpcLibClientBase::ConnectionState::Connected) {
    LOG(FATAL) << "Airsim client was disconnected!";
    is_connected_ = false;
    is_running_ = false;
    raise(SIGINT);
    return;
  }
  /***
   * Note(schmluk): getMultiRotorState returns the *estimated* current state.
   * According to their docs and current implementation, this is not implemented
   * and resturns the *ground truth* state. However, this might change in the
   * future. Alternavtively, we can force ground truth via
   * msr::airlib::Kinematics::State state =
   * airsim_state_client_.simGetGroundTruthKinematics(config_.vehicle_name); But
   * that comes without a timestamp.
   */
  msr::airlib::MultirotorState state =
      airsim_state_client_.getMultirotorState(config_.vehicle_name);
  ros::Time stamp = getTimeStamp(state.timestamp);

  // convert airsim pose to ROS
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = stamp;
  transformStamped.header.frame_id = config_.simulator_frame_name;
  transformStamped.child_frame_id = config_.vehicle_name;
  tf::vectorEigenToMsg(state.kinematics_estimated.pose.position.cast<double>(),
                       transformStamped.transform.translation);
  tf::quaternionEigenToMsg(
      state.kinematics_estimated.pose.orientation.cast<double>(),
      transformStamped.transform.rotation);
  frame_converter_.airsimToRos(&transformStamped.transform);

  // simulate odometry drift
  odometry_drift_simulator_.tick(transformStamped);

  // publish TFs, odom msgs and pose msgs
  odometry_drift_simulator_.publishTfs();
  if (odom_pub_.getNumSubscribers() > 0) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = config_.simulator_frame_name;
    odom_msg.child_frame_id = config_.vehicle_name;

    tf::poseKindrToMsg(odometry_drift_simulator_.getSimulatedPose(),
                       &odom_msg.pose.pose);

    odom_msg.twist.twist.linear.x = state.kinematics_estimated.twist.linear.x();
    odom_msg.twist.twist.linear.y = state.kinematics_estimated.twist.linear.y();
    odom_msg.twist.twist.linear.z = state.kinematics_estimated.twist.linear.z();
    odom_msg.twist.twist.angular.x =
        state.kinematics_estimated.twist.angular.x();
    odom_msg.twist.twist.angular.y =
        state.kinematics_estimated.twist.angular.y();
    odom_msg.twist.twist.angular.z =
        state.kinematics_estimated.twist.angular.z();
    // TODO(schmluk): verify that these twist conversions work as intended
    frame_converter_.airsimToRos(&odom_msg.twist.twist.linear);
    frame_converter_.airsimToRos(&odom_msg.twist.twist.angular);

    odom_pub_.publish(odom_msg);
  }
  if (pose_pub_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = config_.simulator_frame_name;
    tf::poseKindrToMsg(odometry_drift_simulator_.getSimulatedPose(),
                       &pose_msg.pose);
    pose_pub_.publish(pose_msg);
  }

  // collision (the CollisionInfo in the state does not get updated for whatever
  // reason)
  if (airsim_state_client_.simGetCollisionInfo(config_.vehicle_name)
          .has_collided) {
    LOG(WARNING) << "Collision detected for '" << config_.vehicle_name << "'!";
    std_msgs::Bool msg;
    msg.data = true;
    collision_pub_.publish(msg);
  }
  // track trajectory
  trackWayPoints();
}



bool AirsimSimulator::readTransformFromRos(const std::string& topic,
                                           Eigen::Vector3d* translation,
                                           Eigen::Quaterniond* rotation) {
  // This is implemented separately to catch all exceptions when parsing xmlrpc
  // defaults: Unit transformation
  *translation = Eigen::Vector3d();
  *rotation = Eigen::Quaterniond(1, 0, 0, 0);
  if (!nh_private_.hasParam(topic)) {
    return false;
  }
  XmlRpc::XmlRpcValue matrix;
  nh_private_.getParam(topic, matrix);
  if (matrix.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    LOG(WARNING) << "Transformation '" << topic << "' expected as 4x4 array.";
    return false;
  } else if (matrix.size() != 4) {
    LOG(WARNING) << "Transformation '" << topic << "' expected as 4x4 array.";
    return false;
  }
  Eigen::Matrix3d rot_mat;
  Eigen::Vector3d trans;
  double val;
  for (size_t i = 0; i < 3; ++i) {
    XmlRpc::XmlRpcValue row = matrix[i];
    if (row.getType() != XmlRpc::XmlRpcValue::TypeArray) {
      LOG(WARNING) << "Transformation '" << topic << "' expected as 4x4 array.";
      return false;
    } else if (row.size() != 4) {
      LOG(WARNING) << "Transformation '" << topic << "' expected as 4x4 array.";
      return false;
    }
    for (size_t j = 0; j < 4; ++j) {
      try {
        val = row[j];
      } catch (...) {
        try {
          int ival = row[j];
          val = static_cast<double>(ival);
        } catch (...) {
          LOG(WARNING) << "Unable to convert all entries of transformation '"
                       << topic << "' to double.";
          return false;
        }
      }
      if (j < 3) {
        rot_mat(i, j) = val;
      } else {
        trans[i] = val;
      }
    }
  }
  *translation = trans;
  *rotation = rot_mat;
  return true;
}

void AirsimSimulator::onShutdown() {
  is_shutdown_ = true;
  for (const auto& timer : sensor_timers_) {
    timer->signalShutdown();
  }
  if (is_connected_) {
    LOG(INFO) << "Shutting down: resetting airsim server.";
    airsim_state_client_.reset();
    airsim_state_client_.enableApiControl(false);
  }
}

}  // namespace unreal_airsim
