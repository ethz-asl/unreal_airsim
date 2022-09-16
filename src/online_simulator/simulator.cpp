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
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include <glog/logging.h>

#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <thread>

namespace unreal_airsim {

AirsimSimulator::AirsimSimulator(const ros::NodeHandle& nh,
                                 const ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      is_connected_(false),
      is_running_(false),
      is_shutdown_(false),
      followingGoal(false),
      only_move_in_yaw_direction(false),
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
nh_private_.param("reset_timer",
                    config_.reset_timer,
                    defaults.reset_timer);
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

  command_pose_no_odom_sub_ =
      nh_.subscribe(config_.vehicle_name + "/command/pose_no_odom", 1,
                    &AirsimSimulator::commandPoseNoOdomCallback, this);

  command_trajectory_sub_ =
      nh_.subscribe(config_.vehicle_name + "/command/trajectory", 10,
                    &AirsimSimulator::commandTrajectorycallback, this);

  // // sensors
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
      frame_converter_.rosToAirsim(&pose);

      auto* pt = new Waypoint;
      pt->pose = pose;
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
  std::cout << "Pose Orientation In ROS Frame: " << command_ori.x() << " "
            << command_ori.y() << " " << command_ori.z() << " "
            << command_ori.w() << std::endl;
  frame_converter_.rosToAirsim(&command_ori);
  std::cout << "Pose Orientation In Airsim Frame: " << command_ori.x() << " "
            << command_ori.y() << " " << command_ori.z() << " "
            << command_ori.w() << std::endl;
  double yaw = tf2::getYaw(
      tf2::Quaternion(command_ori.x(), command_ori.y(), command_ori.z(),
                      command_ori.w()));  // Eigen's eulerAngles apparently
  // messes up some wrap arounds or direcions and gets the wrong yaws in some
  // cases
  // config_.velocity =0.2f;
  yaw = yaw / M_PI * 180.0;
  constexpr double kMinMovingDistance = 0.1;  // m
  airsim_move_client_.cancelLastTask();
  if ((command_pos - t_gt_current_position).norm() >= kMinMovingDistance) {
    frame_converter_.rosToAirsim(&command_pos);
    auto yaw_mode = msr::airlib::YawMode(false, yaw);
    float look_ahead = -1, adaptive_lookahead = 0;
    airsim_move_client_.moveToPositionAsync(
        command_pos.x(), command_pos.y(), command_pos.z(), config_.velocity,
        3600, config_.drive_train_type, yaw_mode, look_ahead,
        adaptive_lookahead, config_.vehicle_name);
    std::cout << "Moving to target position with " << look_ahead
              << " lookahead and " << adaptive_lookahead
              << " as adaptive lookahead and velocity " << config_.velocity
              << std::endl;
  } else {
    // This second command catches the case if the total distance is too small,
    // where the moveToPosition command returns without satisfying the yaw. If
    // this is always run then apparently sometimes the move command is
    // overwritten.
    airsim_move_client_.rotateToYawAsync(yaw, 3600, 5, config_.vehicle_name);
    std::cout << "No moving, only rotating" << std::endl;
  }
}

void AirsimSimulator::commandPoseNoOdomCallback(const geometry_msgs::Pose& msg) {
  if (!is_running_) {
    std::cout << "Not running." << std::endl;
    return;
  }

  // // Input pose is in drifting odom frame, we therefore
  // // first convert it back into Unreal GT frame
  OdometryDriftSimulator::Transformation T_drift_command;
  tf::poseMsgToKindr(msg, &T_drift_command);
  const OdometryDriftSimulator::Transformation T_gt_command =
      odometry_drift_simulator_.convertDriftedToGroundTruthPose(
          T_drift_command);
  OdometryDriftSimulator::Transformation::Position t_gt_current_position =
      odometry_drift_simulator_.getGroundTruthPose().getPosition();

  // Use position + yaw as setpoint
  auto command_pos = msg.position;
  auto command_ori = msg.orientation;
  std::cout << "Pose Position In ROS Frame: " << command_pos.x << " "
            << command_pos.y << " " << command_pos.z << std::endl;
  std::cout << "Pose Orientation In ROS Frame: " << command_ori.x << " "
            << command_ori.y << " " << command_ori.z << " "
            << command_ori.w << std::endl;
  frame_converter_.rosToAirsim(&command_pos);
  frame_converter_.rosToAirsim(&command_ori);
  std::cout << "Pose Position In Airsim Frame: " << command_pos.x << " "
            << command_pos.y << " " << command_pos.z << std::endl;
  std::cout << "Pose Orientation In Airsim Frame: " << command_ori.x << " "
            << command_ori.y << " " << command_ori.z << " "
            << command_ori.w << std::endl;
            
  // double yaw = tf2::getYaw(
  //     tf2::Quaternion(command_ori.x, command_ori.y, command_ori.z,
  //                     command_ori.w));  // Eigen's eulerAngles apparently
  // // messes up some wrap arounds or direcions and gets the wrong yaws in some
  // // cases

  Eigen::Vector3f command_pos_eigen(command_pos.x, command_pos.y, command_pos.z);
  Eigen::Quaternion command_ori_eigen(float(command_ori.w), 
                                      float(command_ori.x), 
                                      float(command_ori.y), 
                                      float(command_ori.z));

  airsim_move_client_.cancelLastTask();
  airsim_move_client_.simSetVehiclePose(Pose(command_pos_eigen, command_ori_eigen),
                                        true,
                                        "airsim_drone");

  // Trigger image collection
  sensor_timers_.front()->getImages();
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
    LOG(INFO) << "Using sim time: AirSim time will be published as ROS time by "
                 "the simulator.";
    std::thread([this]() {
      while (is_connected_ && !is_shutdown_) {
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
    double current_yaw = math_utils::getYawFromQuaternionMsg(pose.orientation);
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
          vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.linear.z,
          vel_cmd_duration, msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
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
  sim_state_timer_ = ros::Timer();
  if (is_connected_) {
    if (config_.reset_timer > 0.f) {
    LOG(INFO) << "Shutting down: resetting airsim server in "<< config_.reset_timer<<" seconds.";
    sleep(config_.reset_timer);
    }
    LOG(INFO) << "Shutting down: Resetting airsim server.";
    airsim_state_client_.reset();
    airsim_state_client_.enableApiControl(false);
  }
}

}  // namespace unreal_airsim
