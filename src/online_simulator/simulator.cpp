#include "unreal_airsim/online_simulator/simulator.h"
#include "unreal_airsim/online_simulator/sensor_timer.h"

STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include <glog/logging.h>

#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

namespace unreal_airsim {

AirsimSimulator::AirsimSimulator(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private) :
    nh_(nh), nh_private_(nh_private), is_connected_(false), is_running_(false), is_shutdown_(false) {

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

  // Startup via callback
  startup_timer_ = nh_private_.createTimer(ros::Duration(0.1),
                                           &AirsimSimulator::startupCallback, this);
}

bool AirsimSimulator::readParamsFromRos() {
  AirsimSimulator::Config defaults;

  // params
  nh_private_.param("state_refresh_rate", config_.state_refresh_rate, defaults.state_refresh_rate);
  nh_private_.param("simulator_frame_name", config_.simulator_frame_name, defaults.simulator_frame_name);
  nh_private_.param("vehicle_name", config_.vehicle_name, defaults.vehicle_name);
  nh_private_.param("velocity", config_.velocity, defaults.velocity);

  // Verify params valid
  if (config_.state_refresh_rate <= 0.0) {
    config_.state_refresh_rate = defaults.state_refresh_rate;
    LOG(WARNING) << "Param 'state_refresh_rate' expected > 0.0, set to '" << defaults.state_refresh_rate
                 << "' (default).";
  }
  if (config_.velocity <= 0.0) {
    config_.velocity = defaults.velocity;
    LOG(WARNING) << "Param 'velocity' expected > 0.0, set to '" << defaults.velocity << "' (default).";
  }

  // setup sensors
  std::vector<std::string> keys;
  nh_private_.getParamNames(keys);
  std::string sensor_ns = "sensors/";
  std::string full_ns = nh_private_.getNamespace() + "/" + sensor_ns;
  std::string sensor_name;
  std::vector<std::string> sensors;
  size_t pos;
  for (auto const &key: keys) {
    if ((pos = key.find(full_ns)) != std::string::npos) {
      sensor_name = key;
      sensor_name.erase(0, pos + full_ns.length());
      pos = sensor_name.find('/');
      if (pos != std::string::npos) {
        sensor_name = sensor_name.substr(0, pos);
      }
      if (std::find(sensors.begin(), sensors.end(), sensor_name) == sensors.end()) {
        sensors.push_back(sensor_name);
      }
    }
  }
  for (auto const &name: sensors) {
    // currently pass all settings via params, maybe could add some smart identification here
    if (!nh_private_.hasParam(sensor_ns + name + "/sensor_type")) {
      LOG(WARNING) << "Sensor '" << name << "' has no sensor_type and will be ignored!";
      continue;
    }
    Config::Sensor *sensor_cfg;
    std::string sensor_type;
    nh_private_.getParam(sensor_ns + name + "/sensor_type", sensor_type);

    if (sensor_type == Config::Sensor::TYPE_CAMERA) {
      auto *cfg = new Config::Camera();
      Config::Camera cam_defaults;
      nh_private_.param(sensor_ns + name + "/pixels_as_float", cfg->pixels_as_float, cfg->pixels_as_float);
      // cam types default to visual (Scene) camera, but make sure if something else is intended a warning is thrown
      std::string read_img_type_default = "Param is not string";
      if (!nh_private_.hasParam(sensor_ns + name + "/ImageType")) {
        read_img_type_default = cam_defaults.image_type_str;
      }
      nh_private_.param(sensor_ns + name + "/ImageType", cfg->image_type_str, read_img_type_default);
      if (cfg->image_type_str == "Scene") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::Scene;
      } else if (cfg->image_type_str == "DepthPerspective") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::DepthPerspective;
      } else if (cfg->image_type_str == "DepthPlanner") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::DepthPlanner;
      } else if (cfg->image_type_str == "DepthVis") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::DepthVis;
      } else if (cfg->image_type_str == "DisparityNormalized") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::DisparityNormalized;
      } else if (cfg->image_type_str == "SurfaceNormals") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::SurfaceNormals;
      } else if (cfg->image_type_str == "Segmentation") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::Segmentation;
      } else if (cfg->image_type_str == "Infrared") {
        cfg->image_type = msr::airlib::ImageCaptureBase::ImageType::Infrared;
      } else {
        LOG(WARNING) << "Unrecognized ImageType '" << cfg->image_type_str << "' for camera '" << name
                     << "', set to '" << cam_defaults.image_type_str << "' (default).";
        cfg->image_type = cam_defaults.image_type;
        cfg->image_type_str = cam_defaults.image_type_str;
      }
      sensor_cfg = (Config::Sensor *) cfg;
    } else if (sensor_type == Config::Sensor::TYPE_LIDAR) {
      sensor_cfg = new Config::Sensor();
    } else if (sensor_type == Config::Sensor::TYPE_IMU) {
      sensor_cfg = new Config::Sensor();
    } else {
      LOG(WARNING) << "Unknown sensor_type '" << sensor_type << "' for sensor '" << name
                   << "', sensor will be ignored!";
      continue;
    }

    // general settings
    sensor_cfg->name = name;
    sensor_cfg->sensor_type = sensor_type;
    nh_private_.param(sensor_ns + name + "/output_topic", sensor_cfg->output_topic, config_.vehicle_name + "/" + name);
    nh_private_.param(sensor_ns + name + "/frame_name", sensor_cfg->frame_name, config_.vehicle_name + "_" + name);
    nh_private_.param(sensor_ns + name + "/force_separate_timer",
                      sensor_cfg->force_separate_timer,
                      sensor_cfg->force_separate_timer);
    double rate;
    nh_private_.param(sensor_ns + name + "/rate", rate, sensor_cfg->rate);
    if (rate <= 0) {
      LOG(WARNING) << "Param 'rate' for sensor '" << name << "' expected > 0.0, set to '" << sensor_cfg->rate
                   << "' (default).";
    } else {
      sensor_cfg->rate = rate;
    }
    readTransformFromRos(sensor_ns + name + "/T_B_S", &(sensor_cfg->translation), &(sensor_cfg->rotation));
    config_.sensors.push_back(std::unique_ptr<Config::Sensor>(sensor_cfg));
  }
  return true;
}

bool AirsimSimulator::setupAirsim() {
  // This is implemented explicitly to avoid Airsim printing and make it clearer for us what is going wrong
  int timeout = 0;
  while (airsim_state_client_.getConnectionState() != RpcLibClientBase::ConnectionState::Connected && ros::ok()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    timeout++;
    if (timeout > 10) {
      // connection state will remain RpcLibClientBase::ConnectionState::Initial if the unreal game was not running when
      // creating the client (in the constructor)
      LOG(FATAL) << "Unable to connect to the Airsim Server (timeout after 5s). "
                    "Is a UE4 game with enabled Airsim plugin running?";
      return false;
    }
  }

  // check versions
  bool versions_matching = true;  // Check both in one run to spare running into the issue twice
  int server_ver = 0;
  try {
    server_ver = airsim_state_client_.getServerVersion();
  }
  catch (rpc::rpc_error &e) {
    auto msg = e.get_error().as<std::string>();
    std::cout << "Lukas' RPC catch:" << std::endl << msg << std::endl;
  }
  int client_ver = airsim_state_client_.getClientVersion();
  int server_min_ver = airsim_state_client_.getMinRequiredServerVersion();
  int client_min_ver = airsim_state_client_.getMinRequiredClientVersion();
  if (client_ver < client_min_ver) {
    LOG(FATAL) << "Airsim Client version is too old (is: " << client_ver << ", min: " << client_min_ver
               << "). Update and rebuild the Airsim library.";
    versions_matching = false;
  }
  if (server_ver < server_min_ver) {
    LOG(FATAL) << "Airsim Server version is too old (is: " << server_ver << ", min: " << server_min_ver
               << "). Update and rebuild the Airsim UE4 Plugin.";
    versions_matching = false;
  }
  return versions_matching;
}

bool AirsimSimulator::setupROS() {
  // General
  // Publish state (timestamps??)
  sim_state_timer_ = nh_.createTimer(ros::Duration(1.0 / config_.state_refresh_rate),
                                     &AirsimSimulator::simStateCallback, this);
  odom_pub_ = nh_.advertise<nav_msgs::Odometry>(config_.vehicle_name + "/ground_truth/odometry", 5);
  pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(config_.vehicle_name + "/ground_truth/pose", 5);
  collision_pub_ = nh_.advertise<std_msgs::Bool>(config_.vehicle_name + "/collision", 1);
  sim_is_ready_pub_ = nh_.advertise<std_msgs::Bool>("simulation_is_ready", 1);

  // control interfaces
  command_pose_sub_ = nh_.subscribe(config_.vehicle_name + "/command/pose", 5, &AirsimSimulator::commandPoseCallback, this);

  // sensors
  for (auto const &sensor: config_.sensors) {
    // Find or allocate the sensor timer
    SensorTimer *timer = nullptr;
    if (!sensor->force_separate_timer) {
      for (const auto &t : sensor_timers_) {
        if (!t->isPrivate() && t->getRate() == sensor->rate) {
          timer = t.get();
          break;
        }
      }
    }
    if (timer == nullptr) {
      sensor_timers_.push_back(std::make_unique<SensorTimer>(nh_,
                                                             sensor->rate,
                                                             sensor->force_separate_timer,
                                                             config_.vehicle_name,
                                                             frame_converter_));
      timer = sensor_timers_.back().get();
    }
    config_.sensor_to_add = sensor.get();
    timer->addSensor(this);
    // Sensor transform broadcast
    geometry_msgs::TransformStamped static_transformStamped;
    Eigen::Quaterniond rotation = sensor->rotation;
    if (sensor->sensor_type == Config::Sensor::TYPE_CAMERA){
      // Camera frames are x right, y down, z depth
      rotation = Eigen::Quaterniond(0.5,-0.5, 0.5, -0.5) * rotation;
    }
    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = config_.vehicle_name;
    static_transformStamped.child_frame_id = sensor->frame_name;
    static_transformStamped.transform.translation.x = sensor->translation.x();
    static_transformStamped.transform.translation.y =  sensor->translation.y();
    static_transformStamped.transform.translation.z = sensor->translation.z();
    static_transformStamped.transform.rotation.x =  rotation.x();
    static_transformStamped.transform.rotation.y = rotation.y();
    static_transformStamped.transform.rotation.z = rotation.z();
    static_transformStamped.transform.rotation.w = rotation.w();
    static_tf_broadcaster_.sendTransform(static_transformStamped);
  }
  return true;
}

bool AirsimSimulator::initializeSimulationFrame() {
  if (is_shutdown_) { return false; }
  // For frame conventions see coords/frames section in the readme/doc
  msr::airlib::Pose pose = airsim_state_client_.simGetVehiclePose(config_.vehicle_name);
  Eigen::Quaterniond ori(pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
  Eigen::Vector3d euler = ori.toRotationMatrix().eulerAngles(2, 1, 0);  // yaw-pitch-roll in airsim coords
  double yaw = euler[0];
  /**
   * NOTE(schmluk): make the coordinate rotation snap to full 90 degree rotations, wrong initialization (due to physics
   * during drone spawn etc.) would induce large errors for the evaluation of the constructed map.
   */
  double diff = fmod(yaw, M_PI / 2.0);
  if (diff > M_PI / 4.0) { diff -= M_PI / 2.0; }
  if (abs(diff) < 10.0 / 180.0 * M_PI) { yaw -= diff; }
  frame_converter_.setupFromYaw(yaw);
  return true;
}

void AirsimSimulator::commandPoseCallback(const geometry_msgs::Pose& msg){
  if (!is_running_) { return; }
  // Input pose is in simulator (odom) frame, use position + yaw as setpoint
  auto pos = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  auto ori = Eigen::Quaterniond(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
  Eigen::Vector3d euler = ori.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX
  auto yaw_mode = msr::airlib::YawMode(false, -euler[0] / M_PI * 180.0);
  frame_converter_.rosToAirsim(&pos);
  airsim_move_client_.moveToPositionAsync(pos.x(), pos.y(), pos.z(), config_.velocity, 3600,
      config_.drive_train_type, yaw_mode, -1, 1, config_.vehicle_name);
}

void AirsimSimulator::startupCallback(const ros::TimerEvent &) {
  // Startup the drone, this should set the MAV hovering at 'PlayerStart' in unreal
  airsim_move_client_.enableApiControl(true);  // Also disables user control, which is good
  airsim_move_client_.armDisarm(true);
  airsim_move_client_.takeoffAsync(2)->waitOnLastTask();
  airsim_move_client_.moveToPositionAsync(0, 0, 0, 5)->waitOnLastTask();
  is_running_ = true;
  std_msgs::Bool msg;
  msg.data = true;
  sim_is_ready_pub_.publish(msg);
  LOG(INFO) << "Airsim simulation is ready!";
  startup_timer_.stop();
}

void AirsimSimulator::simStateCallback(const ros::TimerEvent &) {
  if (is_shutdown_) { return; }
  if (airsim_state_client_.getConnectionState() != RpcLibClientBase::ConnectionState::Connected) {
    LOG(FATAL) << "Airsim client was disconnected!";
    is_connected_ = false;
    is_running_ = false;
    raise(SIGINT);
    return;
  }
  msr::airlib::Kinematics::State state = airsim_state_client_.simGetGroundTruthKinematics(config_.vehicle_name);

  // body frame
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = config_.simulator_frame_name;
  transformStamped.child_frame_id = config_.vehicle_name;
  transformStamped.transform.translation.x = state.pose.position[0];
  transformStamped.transform.translation.y = state.pose.position[1];
  transformStamped.transform.translation.z = state.pose.position[2];
  transformStamped.transform.rotation.x = state.pose.orientation.x();
  transformStamped.transform.rotation.y = state.pose.orientation.y();
  transformStamped.transform.rotation.z = state.pose.orientation.z();
  transformStamped.transform.rotation.w = state.pose.orientation.w();
  frame_converter_.airsimToRos(&(transformStamped.transform));
  tf_broadcaster_.sendTransform(transformStamped);

  // odom
  if (odom_pub_.getNumSubscribers() > 0) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = config_.simulator_frame_name;
    odom_msg.child_frame_id = config_.vehicle_name;
    odom_msg.pose.pose.position.x = state.pose.position[0];
    odom_msg.pose.pose.position.y = state.pose.position[1];
    odom_msg.pose.pose.position.z = state.pose.position[2];
    odom_msg.pose.pose.orientation.x = state.pose.orientation.x();
    odom_msg.pose.pose.orientation.y = state.pose.orientation.y();
    odom_msg.pose.pose.orientation.z = state.pose.orientation.z();
    odom_msg.pose.pose.orientation.w = state.pose.orientation.w();
    odom_msg.twist.twist.linear.x = state.twist.linear.x();
    odom_msg.twist.twist.linear.y = state.twist.linear.y();
    odom_msg.twist.twist.linear.z = state.twist.linear.z();
    odom_msg.twist.twist.angular.x = state.twist.angular.x();
    odom_msg.twist.twist.angular.y = state.twist.angular.y();
    odom_msg.twist.twist.angular.z = state.twist.angular.z();
    frame_converter_.airsimToRos(&(odom_msg.pose.pose));
    // TODO(schmluk): verify that these twist conversions work as intended
    frame_converter_.airsimToRos(&(odom_msg.twist.twist.linear));
    frame_converter_.airsimToRos(&(odom_msg.twist.twist.angular));
    odom_pub_.publish(odom_msg);

    if (pose_pub_.getNumSubscribers() > 0) {
      geometry_msgs::PoseStamped pose_msg;
      pose_msg.header.stamp = ros::Time::now();
      pose_msg.header.frame_id = config_.simulator_frame_name;
      pose_msg.pose = odom_msg.pose.pose;
      pose_pub_.publish(pose_msg);
    }
  } else if (pose_pub_.getNumSubscribers() > 0) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = config_.simulator_frame_name;
    pose_msg.pose.position.x = state.pose.position[0];
    pose_msg.pose.position.y = state.pose.position[1];
    pose_msg.pose.position.z = state.pose.position[2];
    pose_msg.pose.orientation.x = state.pose.orientation.x();
    pose_msg.pose.orientation.y = state.pose.orientation.y();
    pose_msg.pose.orientation.z = state.pose.orientation.z();
    pose_msg.pose.orientation.w = state.pose.orientation.w();
    frame_converter_.airsimToRos(&(pose_msg.pose));
    pose_pub_.publish(pose_msg);
  }

  // collision (the CollisionInfo in the state does not get updated for whatever reason)
  if (airsim_state_client_.simGetCollisionInfo(config_.vehicle_name).has_collided) {
    LOG(WARNING) << "Collision detected for '" << config_.vehicle_name << "'!";
    std_msgs::Bool msg;
    msg.data = true;
    collision_pub_.publish(msg);
  }
}

bool AirsimSimulator::readTransformFromRos(const std::string &topic,
                                           Eigen::Vector3d *translation,
                                           Eigen::Quaterniond *rotation) {
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
          val = (double)ival;
        } catch (...) {
          LOG(WARNING) << "Unable to convert all entries of transformation '" << topic << "' to double.";
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
  LOG(INFO) << "Shutting down: resetting airsim server.";
  is_shutdown_ = true;
  for (const auto &timer : sensor_timers_){
    timer->signalShutdown();
  }
  if (is_connected_) {
    airsim_state_client_.reset();
    airsim_state_client_.enableApiControl(false);
  }
}

} // namespcae unreal_airsim
