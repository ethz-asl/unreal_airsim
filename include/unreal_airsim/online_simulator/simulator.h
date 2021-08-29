#ifndef UNREAL_AIRSIM_ONLINE_SIMULATOR_SIMULATOR_H_
#define UNREAL_AIRSIM_ONLINE_SIMULATOR_SIMULATOR_H_

#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

// AirSim
#include <common/CommonStructs.hpp>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>

#include "unreal_airsim/frame_converter.h"
#include "unreal_airsim/online_simulator/sensor_timer.h"
#include "unreal_airsim/simulator_processing/processor_base.h"

#include "unreal_airsim/simulator_processing/odometry_drift_simulator/odometry_drift_simulator.h"

namespace unreal_airsim {

  typedef struct Waypoint {
  geometry_msgs::Transform_<std::allocator<void>> pose;
  bool isGoal{};
} Waypoint;

class PIDParams
{
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
        : kp_x(0.5), kp_y(0.5), kp_z(0.10), kp_yaw(0.4), kd_x(0.3), kd_y(0.3), kd_z(0.2), kd_yaw(0.1), reached_thresh_xyz(0.02), reached_yaw_degrees(0.1)
    {
    }

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

// todo should be a common representation
struct XYZYaw
{
    double x;
    double y;
    double z;
    double yaw;
};

// todo should be a common representation
class DynamicConstraints
{
public:
    double max_vel_horz_abs; // meters/sec
    double max_vel_vert_abs;
    double max_yaw_rate_degree;

    DynamicConstraints()
        : max_vel_horz_abs(1.0), max_vel_vert_abs(0.5), max_yaw_rate_degree(10.0)
    {
    }

    bool load_from_rosparams(const ros::NodeHandle& nh);
};

namespace math_utils {

  template <typename T>
inline T rad2deg(const T radians)
{
    return (radians / M_PI) * 180.0;
}

template <typename T>
inline T wrap_to_pi(T radians)
{
    int m = (int)(radians / (2 * M_PI));
    radians = radians - m * 2 * M_PI;
    if (radians > M_PI)
        radians -= 2.0 * M_PI;
    else if (radians < -M_PI)
        radians += 2.0 * M_PI;
    return radians;
}

template <typename T>
inline void wrap_to_pi_inplace(T& a)
{
    a = wrap_to_pi(a);
}

template <class T>
inline T angular_dist(T from, T to)
{
    wrap_to_pi_inplace(from);
    wrap_to_pi_inplace(to);
    T d = to - from;
    if (d > M_PI)
        d -= 2. * M_PI;
    else if (d < -M_PI)
        d += 2. * M_PI;
    return d;
}
}


/***
 * This class implements a simulation interface with airsim.
 * Current application case is for a single Multirotor Vehicle.
 */
class AirsimSimulator {
 public:
  /***
   * Available settings for the simulation and their defaults.
   */
  struct Config {
    // general settings
    double state_refresh_rate = 100;  // hz
    int time_publisher_interval =
        2;  // ms, this is the interval (in wall-time) in which the airsim time
            // is published as sim_time, i.e. 500 Hz. This only happens if
            // use_sim_time=true during launch.
    std::string simulator_frame_name = "odom";

    // vehicle (the multirotor)
    std::string vehicle_name =
        "airsim_drone";  // Currently assumes a single drone, multi-vehicle sim
                         // could be setup
    double velocity = 1.0;  // m/s, for high level movement commands
    msr::airlib::DrivetrainType drive_train_type =
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom;  // this is currently
                                                          // fixed

    // sensors
    bool publish_sensor_transforms = true;  // publish transforms when receiving
    // sensor measurements to guarantee correct tfs.
    // TODO(schmluk): This is mostly a time syncing problem, maybe easiest to
    //  publish the body pose based on these readings.
    struct Sensor {
      inline static const std::string TYPE_CAMERA = "Camera";
      inline static const std::string TYPE_LIDAR = "Lidar";
      inline static const std::string TYPE_IMU = "Imu";
      std::string name = "";
      std::string sensor_type = "";
      std::string output_topic;  // defaults to vehicle_name/sensor_name
      std::string frame_name;    // defaults to vehicle_name/sensor_name
      double rate = 10.0;        // Hz
      bool force_separate_timer =
          false;  // By default all sensors of identical rate are synced into 1
                  // timer, but that can slow down overall performance for a
                  // specific sensor
      Eigen::Vector3d translation;  // T_B_S, default is unit transform
      Eigen::Quaterniond rotation;
    };
    struct Camera : Sensor {
      std::string image_type_str = "Scene";
      bool pixels_as_float = false;
      msr::airlib::ImageCaptureBase::ImageType image_type =
          msr::airlib::ImageCaptureBase::ImageType::Scene;
      msr::airlib::CameraInfo camera_info;  // The info is read from UE4
    };
    std::vector<std::unique_ptr<Sensor>> sensors;
  };

  AirsimSimulator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  virtual ~AirsimSimulator() = default;

  // ROS callbacks
  void simStateCallback(const ros::TimerEvent&);
  void startupCallback(const ros::TimerEvent&);
  void onShutdown();  // called by the sigint handler

  // Control
  /**
   * NOTE(Schmluk): Airsim also exposes a number of other control interfaces,
   * mostly low level. If needed, some of these could also be included here, but
   * set pose should do for most purposes.
   */
  void commandPoseCallback(const geometry_msgs::Pose& msg);

  // added from trajectory caller node
  void commandTrajectorycallback(const trajectory_msgs::MultiDOFJointTrajectory trajectory);

  // Acessors
  const Config& getConfig() const { return config_; }
  const FrameConverter& getFrameConverter() const { return frame_converter_; }
  ros::Time getTimeStamp(msr::airlib::TTimePoint airsim_stamp);
  OdometryDriftSimulator* getOdometryDriftSimulator() {
    return &odometry_drift_simulator_;
  }

 protected:
  // ROS
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Timer sim_state_timer_;
  ros::Timer startup_timer_;
  ros::Publisher odom_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher collision_pub_;
  ros::Publisher sim_is_ready_pub_;
  ros::Publisher time_pub_;
  ros::Subscriber command_pose_sub_;
  ros::Subscriber command_trajectory_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

  // Odometry simulator
  OdometryDriftSimulator odometry_drift_simulator_;
  std::queue<Waypoint *> points;
  Waypoint *current_goal;
  // Whether robot is idle or currently tracking a goal
  bool followingGoal;
  bool only_move_in_yaw_direction;
  // Current yaw of the goal position
  double goal_yaw;

  // Read sim time from AirSim
  std::thread timer_thread_;

  bool reached_goal_;
  bool has_goal_;
  bool got_goal_once_;

  DynamicConstraints constraints_;
  PIDParams params_;
  XYZYaw target_position_;
  XYZYaw curr_position_;
  XYZYaw prev_error_;
  XYZYaw curr_error_;

  // components
  std::vector<std::unique_ptr<SensorTimer>>
      sensor_timers_;  // These manage the actual sensor reading/publishing
  std::vector<std::unique_ptr<simulator_processor::ProcessorBase>>
      processors_;  // Various post-processing

  // Airsim clients (These can be blocking and thus slowing down tasks if only
  // one is used)
  msr::airlib::MultirotorRpcLibClient airsim_state_client_;
  msr::airlib::MultirotorRpcLibClient airsim_move_client_;
  msr::airlib::MultirotorRpcLibClient airsim_time_client_;

  // tools
  Config config_;
  FrameConverter frame_converter_;  // the world-to-airsim transformation

  // variables
  bool is_connected_;  // whether the airsim client is connected
  bool is_running_;   // whether the simulator setup successfully and is working
  bool is_shutdown_;  // After setting is shutdown no more airsim requests are
                      // allowed.
  bool use_sim_time_;  // Publish ros time based on the airsim clock

  // setup methods
  bool setupAirsim();  // Connect to Airsim and verify
  bool setupROS();
  bool readParamsFromRos();
  bool initializeSimulationFrame();
  bool startSimTimer();

  // methods
  void readSimTimeCallback();

  // helper methods
  bool readTransformFromRos(const std::string& topic,
                            Eigen::Vector3d* translation,
                            Eigen::Quaterniond* rotation);
};

}  // namespace unreal_airsim

#endif  // UNREAL_AIRSIM_ONLINE_SIMULATOR_SIMULATOR_H_
