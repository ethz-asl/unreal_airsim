#include "unreal_airsim/online_simulator/sensor_timer.h"

#include <string>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <glog/logging.h>
#include <minkindr_conversions/kindr_msg.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include "unreal_airsim/online_simulator/simulator.h"

namespace unreal_airsim {

SensorTimer::SensorTimer(const ros::NodeHandle& nh, double rate,
                         bool is_private, const std::string& vehicle_name,
                         AirsimSimulator* parent)
    : nh_(nh),
      is_private_(is_private),
      rate_(rate),
      vehicle_name_(vehicle_name),
      is_shutdown_(false),
      parent_(parent) {
  timer_ = nh_.createTimer(ros::Duration(1.0 / rate),
                           &SensorTimer::timerCallback, this);
}

double SensorTimer::getRate() const { return rate_; }
bool SensorTimer::isPrivate() const { return is_private_; }

void SensorTimer::signalShutdown() { is_shutdown_ = true; }

void SensorTimer::timerCallback(const ros::TimerEvent&) {
  processCameras();
  processLidars();
  processImus();
}

void SensorTimer::addSensor(const AirsimSimulator& simulator,
                            int sensor_index) {
  AirsimSimulator::Config::Sensor* sensor =
      simulator.getConfig().sensors[sensor_index].get();
  if (sensor->sensor_type == AirsimSimulator::Config::Sensor::TYPE_CAMERA) {
    auto camera_cfg = (AirsimSimulator::Config::Camera*)sensor;
    auto camera = Camera();
    camera.pub = nh_.advertise<sensor_msgs::Image>(camera_cfg->output_topic, 5);
    camera.frame_name = camera_cfg->frame_name;
    camera.request.camera_name = camera_cfg->name;
    camera.request.compress = false;
    camera.request.image_type = camera_cfg->image_type;
    camera.request.pixels_as_float = camera_cfg->pixels_as_float;
    camera.T_S_B = camera_cfg->T_B_S.inverse();
    cameras_.push_back(camera);
  } else if (sensor->sensor_type ==
             AirsimSimulator::Config::Sensor::TYPE_LIDAR) {
    auto lidar = Lidar();
    lidar.pub =
        nh_.advertise<sensor_msgs::PointCloud2>(sensor->output_topic, 5);
    lidar.name = sensor->name;
    lidar.frame_name = sensor->frame_name;
    lidar.T_S_B = sensor->T_B_S.inverse();
    lidars_.push_back(lidar);
  } else if (sensor->sensor_type == AirsimSimulator::Config::Sensor::TYPE_IMU) {
    auto imu = Imu();
    imu.pub = nh_.advertise<sensor_msgs::Imu>(sensor->output_topic, 5);
    imu.name = sensor->name;
    imu.frame_name = sensor->frame_name;
    imus_.push_back(imu);
  }
}

void SensorTimer::processCameras() {
  if (is_shutdown_ || cameras_.empty()) {
    return;
  }
  if (camera_requests_.size() != cameras_.size()) {
    camera_requests_.clear();
    camera_requests_.resize(cameras_.size());
    for (const auto& camera : cameras_) {
      camera_requests_.emplace_back(camera.request);
    }
  }

  // get images from unreal.
  std::vector<msr::airlib::ImageCaptureBase::ImageResponse> responses =
      airsim_client_.simGetImages(camera_requests_, vehicle_name_);
  ros::Time timestamp =
      parent_->getTimeStamp(responses[0].time_stamp);  // these are synchronized
  bool sent_body_transform = false;

  // process responses
  for (size_t i = 0; i < responses.size(); ++i) {
    if (cameras_[i].pub.getNumSubscribers() > 0) {
      sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
      if (responses[i].pixels_as_float) {
        // Encode float images
        cv::Mat cv_image =
            cv::Mat(responses[i].height, responses[i].width, CV_32FC1);
        memcpy(cv_image.data, responses[i].image_data_float.data(),
               responses[i].image_data_float.size() * sizeof(float));
        cv_bridge::CvImage(std_msgs::Header(), "32FC1", cv_image)
            .toImageMsg(*msg);
      } else {
        msg->height = responses[i].height;
        msg->width = responses[i].width;
        msg->is_bigendian = 0;
        if (responses[i].image_type ==
            msr::airlib::ImageCaptureBase::ImageType::Infrared) {
          // IR images are published as 1C mono images.
          msg->step = responses[i].width;
          msg->encoding = "mono8";
          msg->data.resize(responses[i].image_data_uint8.size() / 3);
          for (size_t j = 0; j < msg->data.size(); ++j) {
            msg->data[j] = responses[i].image_data_uint8[j * 3];
          }
        } else {
          // all others are 3C RGB images.
          msg->step = responses[i].width * 3;
          msg->encoding = "bgr8";
          msg->data = responses[i].image_data_uint8;
        }
      }
      msg->header.stamp = timestamp;
      msg->header.frame_id = cameras_[i].frame_name;

      // Publish the pose s.t. it will match with this sensor reading.
      if (!sent_body_transform) {
        sent_body_transform = true;

        // Get the pose.
        auto rotation = Eigen::Quaterniond(responses[i].camera_orientation.w(),
                                           responses[i].camera_orientation.x(),
                                           responses[i].camera_orientation.y(),
                                           responses[i].camera_orientation.z());
        auto position = Eigen::Vector3d(responses[i].camera_position.x(),
                                        responses[i].camera_position.y(),
                                        responses[i].camera_position.z());
        parent_->getFrameConverter().airsimToRos(&rotation);
        parent_->getFrameConverter().airsimToRos(&position);

        // Camera frames are x right, y down, z depth
        // TODO(schmluk) does this still hold?
        // rotation = rotation * Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5);

        kindr::minimal::QuatTransformationTemplate<double> T_W_S(position,
                                                                 rotation);
        kindr::minimal::QuatTransformationTemplate<double> T_W_B =
            T_W_S * cameras_[i].T_S_B;

        // send pose update to simulator.
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = timestamp;
        transformStamped.header.frame_id =
            parent_->getConfig().simulator_frame_name;
        transformStamped.child_frame_id = cameras_[i].frame_name;
        tf::transformKindrToMsg(T_W_B, &(transformStamped.transform));
        parent_->publishState(transformStamped);
      }
      cameras_[i].pub.publish(msg);
    }
    }
}

void SensorTimer::processLidars() {
  if (is_shutdown_ || lidars_.empty()) {
    return;
  }
  for (const Lidar& lidar : lidars_) {
    if (lidar.pub.getNumSubscribers() == 0) {
      continue;
    }
    msr::airlib::LidarData lidar_data =
        airsim_client_.getLidarData(lidar.name, vehicle_name_);
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = lidar.frame_name;
    msg->header.stamp = parent_->getTimeStamp(lidar_data.time_stamp);
    msg->height = 1;
    msg->width = lidar_data.point_cloud.size() / 3;
    msg->fields.resize(3);
    msg->fields[0].name = "x";
    msg->fields[1].name = "y";
    msg->fields[2].name = "z";
    int offset = 0;
    for (size_t d = 0; d < msg->fields.size(); ++d, offset += 4) {
      msg->fields[d].offset = offset;
      msg->fields[d].datatype = sensor_msgs::PointField::FLOAT32;
      msg->fields[d].count = 1;
    }
    msg->point_step = offset;
    msg->is_bigendian = false;
    msg->row_step = msg->point_step * msg->width;
    msg->is_dense = false;
    std::vector<float> data_std = lidar_data.point_cloud;
    for (size_t j = 0; j < data_std.size(); j += 3) {
      // points are in sensor-Frame but with airsim axis
      data_std[j + 1] *= -1.0;
      data_std[j + 2] *= -1.0;
    }
    auto bytes = reinterpret_cast<const unsigned char*>(&data_std[0]);
    vector<unsigned char> lidar_msg_data(
        bytes, bytes + sizeof(float) * data_std.size());
    msg->data = std::move(lidar_msg_data);

    // Publish the corresponding pose time synced.
    // Get the pose.
    auto rotation = Eigen::Quaterniond(
        lidar_data.pose.orientation.w(), lidar_data.pose.orientation.x(),
        lidar_data.pose.orientation.y(), lidar_data.pose.orientation.z());
    auto position = Eigen::Vector3d(lidar_data.pose.position[0],
                                    lidar_data.pose.position[1],
                                    lidar_data.pose.position[2]);
    parent_->getFrameConverter().airsimToRos(&rotation);
    parent_->getFrameConverter().airsimToRos(&position);

    kindr::minimal::QuatTransformationTemplate<double> T_W_S(position,
                                                             rotation);
    kindr::minimal::QuatTransformationTemplate<double> T_W_B =
        T_W_S * lidar.T_S_B;

    // send pose update to simulator.
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp =
        parent_->getTimeStamp(lidar_data.time_stamp);
    transformStamped.header.frame_id =
        parent_->getConfig().simulator_frame_name;
    transformStamped.child_frame_id = lidar.frame_name;
    tf::transformKindrToMsg(T_W_B, &(transformStamped.transform));
    parent_->publishState(transformStamped);

    // Publish pointcloud
    lidar.pub.publish(msg);
  }
}

void SensorTimer::processImus() {
  if (is_shutdown_ || imus_.empty()) {
    return;
  }
  for (const Imu& imu : imus_) {
    msr::airlib::ImuBase::Output imu_data =
        airsim_client_.getImuData(imu.name, vehicle_name_);

    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    // orientation
    msg->header.frame_id = imu.frame_name;
    msg->header.stamp = parent_->getTimeStamp(imu_data.time_stamp);
    msg->orientation.x = imu_data.orientation.x();
    msg->orientation.y = imu_data.orientation.y();
    msg->orientation.z = imu_data.orientation.z();
    msg->orientation.w = imu_data.orientation.w();
    parent_->getFrameConverter().airsimToRos(
        &(msg->orientation));  // transform to simulation frame
    // Rates (these should also be in sensor-frame but with airsim axis,
    // TODO(schmluk): Test this
    msg->angular_velocity.x = imu_data.angular_velocity.x();
    msg->angular_velocity.y = -imu_data.angular_velocity.y();
    msg->angular_velocity.z = -imu_data.angular_velocity.z();
    msg->linear_acceleration.x = imu_data.linear_acceleration.x();
    msg->linear_acceleration.y = -imu_data.linear_acceleration.y();
    msg->linear_acceleration.z = -imu_data.linear_acceleration.z();

    // TODO(schmluk): covariances?
    // imu_msg.orientation_covariance = ;
    // imu_msg.angular_velocity_covariance = ;
    // imu_msg.linear_acceleration_covariance = ;

    imu.pub.publish(msg);
  }
}

}  // namespace unreal_airsim
