#include "unreal_airsim/online_simulator/sensor_timer.h"
#include "unreal_airsim/online_simulator/simulator.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

#include <glog/logging.h>

namespace unreal_airsim {

SensorTimer::SensorTimer(const ros::NodeHandle &nh,
                         double rate,
                         bool is_private,
                         const std::string &vehicle_name,
                         AirsimSimulator *parent) :
    nh_(nh),
    is_private_(is_private),
    rate_(rate),
    vehicle_name_(vehicle_name),
    is_shutdown_(false),
    parent_(parent) {
  timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &SensorTimer::timerCallback, this);
}

double SensorTimer::getRate() const {
  return rate_;
}
bool SensorTimer::isPrivate() const {
  return is_private_;
}

void SensorTimer::signalShutdown() {
  is_shutdown_ = true;
}

void SensorTimer::timerCallback(const ros::TimerEvent &) {
  processCameras();
  processLidars();
  processImus();
}

void SensorTimer::addSensor(const AirsimSimulator &simulator, int sensor_index) {
  AirsimSimulator::Config::Sensor *sensor = simulator.getConfig().sensors[sensor_index].get();
  if (sensor->sensor_type == AirsimSimulator::Config::Sensor::TYPE_CAMERA) {
    auto camera = (AirsimSimulator::Config::Camera *) sensor;
    camera_pubs_.push_back(nh_.advertise<sensor_msgs::Image>(camera->output_topic, 5));
    camera_frame_names_.push_back(camera->frame_name);
    msr::airlib::ImageCaptureBase::ImageRequest request;
    request.camera_name = camera->name;
    request.compress = false;
    request.image_type = camera->image_type;
    request.pixels_as_float = camera->pixels_as_float;
    image_requests_.push_back(request);
  } else if (sensor->sensor_type == AirsimSimulator::Config::Sensor::TYPE_LIDAR) {
    lidar_pubs_.push_back(nh_.advertise<sensor_msgs::PointCloud2>(sensor->output_topic, 5));
    lidar_names_.push_back(sensor->name);
    lidar_frame_names_.push_back(sensor->frame_name);
  } else if (sensor->sensor_type == AirsimSimulator::Config::Sensor::TYPE_IMU) {
    imu_pubs_.push_back(nh_.advertise<sensor_msgs::Imu>(sensor->output_topic, 5));
    imu_names_.push_back(sensor->name);
    imu_frame_names_.push_back(sensor->frame_name);
  }
}

void SensorTimer::processCameras() {
  if (is_shutdown_) { return; }
  if (!image_requests_.empty()) {
    std::vector<msr::airlib::ImageCaptureBase::ImageResponse>
        responses = airsim_client_.simGetImages(image_requests_, vehicle_name_);
    ros::Time timestamp = parent_->getTimeStamp(responses[0].time_stamp); // these are synchronized
    for (size_t i = 0; i < responses.size(); ++i) {
      if (camera_pubs_[i].getNumSubscribers() > 0) {
        sensor_msgs::ImagePtr msg(new sensor_msgs::Image);
        if (responses[i].pixels_as_float) {
          cv::Mat cv_image = cv::Mat(responses[i].height, responses[i].width, CV_32FC1);
          memcpy(cv_image.data,
                 responses[i].image_data_float.data(),
                 responses[i].image_data_float.size() * sizeof(float));
          cv_bridge::CvImage(std_msgs::Header(), "32FC1", cv_image).toImageMsg(*msg);
        } else {
          msg->height = responses[i].height;
          msg->width = responses[i].width;
          msg->is_bigendian = 0;
          msg->step = responses[i].width * 3;
          msg->encoding = "bgr8";
          msg->data = responses[i].image_data_uint8;
        }
        msg->header.stamp = timestamp;
        msg->header.frame_id = camera_frame_names_[i];

        // Ground truth transform
        if (parent_->getConfig().publish_sensor_transforms) {
          auto rotation = Eigen::Quaterniond(responses[i].camera_orientation.w(),
                                             responses[i].camera_orientation.x(),
                                             responses[i].camera_orientation.y(),
                                             responses[i].camera_orientation.z());
          parent_->getFrameConverter().airsimToRos(&(rotation));
          // Camera frames are x right, y down, z depth
          rotation = rotation * Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5);
          geometry_msgs::TransformStamped transformStamped;
          transformStamped.header.stamp = timestamp;
          transformStamped.header.frame_id = parent_->getConfig().simulator_frame_name;
          transformStamped.child_frame_id = camera_frame_names_[i];
          transformStamped.transform.translation.x = responses[i].camera_position[0];
          transformStamped.transform.translation.y = responses[i].camera_position[1];
          transformStamped.transform.translation.z = responses[i].camera_position[2];
          transformStamped.transform.rotation.x = rotation.x();
          transformStamped.transform.rotation.y = rotation.y();
          transformStamped.transform.rotation.z = rotation.z();
          transformStamped.transform.rotation.w = rotation.w();
          parent_->getFrameConverter().airsimToRos(&(transformStamped.transform.translation));
          tf_broadcaster_.sendTransform(transformStamped);
        }

        camera_pubs_[i].publish(msg);
      }
    }
  }
}

void SensorTimer::processLidars() {
  if (is_shutdown_) { return; }
  for (size_t i = 0; i < lidar_names_.size(); ++i) {
    msr::airlib::LidarData lidar_data = airsim_client_.getLidarData(lidar_names_[i], vehicle_name_);
    sensor_msgs::PointCloud2Ptr msg(new sensor_msgs::PointCloud2);
    msg->header.frame_id = lidar_frame_names_[i];
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
    auto bytes = reinterpret_cast<const unsigned char *>(&data_std[0]);
    vector<unsigned char> lidar_msg_data(bytes, bytes + sizeof(float) * data_std.size());
    msg->data = std::move(lidar_msg_data);

    // Ground truth transform
    if (parent_->getConfig().publish_sensor_transforms) {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = msg->header.stamp;
      transformStamped.header.frame_id = lidar_frame_names_[i];
      transformStamped.child_frame_id = parent_->getConfig().simulator_frame_name;
      transformStamped.transform.translation.x = lidar_data.pose.position[0];
      transformStamped.transform.translation.y = lidar_data.pose.position[1];
      transformStamped.transform.translation.z = lidar_data.pose.position[2];
      transformStamped.transform.rotation.x = lidar_data.pose.orientation.x();
      transformStamped.transform.rotation.y = lidar_data.pose.orientation.y();
      transformStamped.transform.rotation.z = lidar_data.pose.orientation.z();
      transformStamped.transform.rotation.w = lidar_data.pose.orientation.w();
      parent_->getFrameConverter().airsimToRos(&(transformStamped.transform));
      tf_broadcaster_.sendTransform(transformStamped);
    }

    lidar_pubs_[i].publish(msg);
  }
}

void SensorTimer::processImus() {
  if (is_shutdown_) { return; }
  for (size_t i = 0; i < imu_names_.size(); ++i) {
    msr::airlib::ImuBase::Output imu_data = airsim_client_.getImuData(imu_names_[i], vehicle_name_);

    sensor_msgs::ImuPtr msg(new sensor_msgs::Imu);
    // orientation
    msg->header.frame_id = imu_frame_names_[i];
    msg->header.stamp = parent_->getTimeStamp(imu_data.time_stamp);
    msg->orientation.x = imu_data.orientation.x();
    msg->orientation.y = imu_data.orientation.y();
    msg->orientation.z = imu_data.orientation.z();
    msg->orientation.w = imu_data.orientation.w();
    parent_->getFrameConverter().airsimToRos(&(msg->orientation));    // transform to simulation frame
    // Rates (these should also be in sensor-frame but with airsim axis, TODO(schmluk): Test this
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

    imu_pubs_[i].publish(msg);
  }
}

} // namespcae unreal_airsim
