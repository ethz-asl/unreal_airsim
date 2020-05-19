#include "unreal_airsim/simulator_processing/depth_to_pointcloud.h"
#include "unreal_airsim/online_simulator/simulator.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>

namespace unreal_airsim::simulator_processor {

ProcessorFactory::Registration<DepthToPointcloud>DepthToPointcloud::registration_("DepthToPointcloud");

bool DepthToPointcloud::setupFromRos(const ros::NodeHandle &nh, const std::string &ns) {
  nh_ = nh;
  use_color_ = false;
  is_setup_ = false;

  // get params
  std::string depth_camera_name, color_camera_name, output_topic;
  nh.param(ns + "max_queue_length", max_queue_length_, 10);
  if (!nh.hasParam(ns + "depth_camera_name")) {
    LOG(FATAL) << "DepthToPointcloud requires the 'depth_camera_name' param to be set!";
    return false;
  }
  nh.getParam(ns + "depth_camera_name", depth_camera_name);
  nh.param(ns + "max_depth", max_depth_, 1e6f);
  nh.param(ns + "output_topic", output_topic, parent_->getConfig().vehicle_name + "/" + name_);
  if (nh.hasParam(ns + "color_camera_name")) {
    nh.getParam(ns + "color_camera_name", color_camera_name);
    use_color_ = true;
  }

  // Find source cameras
  std::string depth_topic, color_topic;
  for (const auto &sensor : parent_->getConfig().sensors) {
    if (sensor->name == depth_camera_name) {
      depth_topic = sensor->output_topic;
      auto camera = (unreal_airsim::AirsimSimulator::Config::Camera *) sensor.get();
      fov_ = camera->camera_info.fov;
    }
    if (sensor->name == color_camera_name) {
      color_topic = sensor->output_topic;
    }
  }
  if (depth_topic.empty()) {
    LOG(FATAL) << "Could not find a Camera with name '" << depth_camera_name << "'!";
    return false;
  }
  if (color_topic.empty() && use_color_) {
    LOG(FATAL) << "Could not find a Camera with name '" << color_camera_name << "'!";
    return false;
  }

  // ros
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 5);
  depth_sub_ = nh_.subscribe(depth_topic, max_queue_length_, &DepthToPointcloud::depthIamgeCallback, this);
  if (use_color_) {
    color_sub_ = nh_.subscribe(color_topic, max_queue_length_, &DepthToPointcloud::colorIamgeCallback, this);
  }
  return true;
}

void DepthToPointcloud::depthIamgeCallback(const sensor_msgs::ImagePtr &msg) {
  // use the first depth image to initialize intrinsics
  if (!is_setup_) {
    vx_ = msg->width / 2;
    vy_ = msg->height / 2;
    focal_length_ = (float) (msg->width) / (2.0 * std::tan(fov_ * M_PI / 360.0));
    is_setup_ = true;
  }
  if (!use_color_) {
    publishPointcloud(msg, nullptr);
  } else {
    // check whether there is a color image with matching timestamp
    auto it = std::find_if(color_queue_.begin(),
                           color_queue_.end(),
                           [msg](const sensor_msgs::ImagePtr &s) { return s->header.stamp == msg->header.stamp; });
    if (it != color_queue_.end()) {
      publishPointcloud(msg, *it);
      color_queue_.erase(it);
    } else {
      // store depth img in queue
      depth_queue_.push_back(msg);
      if (depth_queue_.size() > max_queue_length_) {
        depth_queue_.pop_front();
      }
    }
  }
}

void DepthToPointcloud::colorIamgeCallback(const sensor_msgs::ImagePtr &msg) {
  // check whether there is a depth image with matching timestamp
  auto it = std::find_if(depth_queue_.begin(),
                         depth_queue_.end(),
                         [msg](const sensor_msgs::ImagePtr &s) { return s->header.stamp == msg->header.stamp; });
  if (it != depth_queue_.end()) {
    publishPointcloud(*it, msg);
    depth_queue_.erase(it);
  } else {
    // store color img in queue
    color_queue_.push_back(msg);
    if (color_queue_.size() > max_queue_length_) {
      color_queue_.pop_front();
    }
  }
}

void DepthToPointcloud::publishPointcloud(const sensor_msgs::ImagePtr &depth_ptr,
                                          const sensor_msgs::ImagePtr &color_ptr) {
  /**
   * NOTE(schmluk): This method assumes that depth and color image are from the same simulated camera, i.e. are
   * perfectly aligned and have identical settings (resolution, intrinsics, extrinsics)
   */
   if (pub_.getNumSubscribers() == 0){
     return;
   }
  cv_bridge::CvImageConstPtr depth_img, color_img;
  depth_img = cv_bridge::toCvShare(depth_ptr, depth_ptr->encoding);
  if (use_color_) {
    color_img = cv_bridge::toCvShare(color_ptr, color_ptr->encoding);
  }

  // figure out number of points
  int numpoints = depth_img->image.rows * depth_img->image.cols;

  // declare message and sizes
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = depth_ptr->header.frame_id;
  cloud.header.stamp = depth_ptr->header.stamp;
  cloud.width = numpoints;
  cloud.height = 1;
  cloud.is_bigendian = false;
  cloud.is_dense = false; // there may be invalid points

  // fields setup
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  if (use_color_) {
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  } else {
    modifier.setPointCloud2FieldsByString(1, "xyz");
  }
  modifier.resize(numpoints);

  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");

  // Apparently there's no default constructor for the iterator, if no color is used these will be ignored.
  sensor_msgs::PointCloud2Iterator<uint8_t> out_rgb(cloud, "x");
  if (use_color_) {
    out_rgb = sensor_msgs::PointCloud2Iterator<uint8_t>(cloud, "rgb");
  }
  for (int y = 0; y < depth_img->image.rows; y++) {
    for (int x = 0; x < depth_img->image.cols; x++) {
      float z = depth_img->image.at<float>(y, x);   // This assumes we use planar depth camera (ImageType::DepthPlanner)
      if (z > max_depth_) {
        continue;
      }
      out_x[0] = ((float)x - vx_) * z / focal_length_;
      out_x[1] = ((float)y - vy_) * z / focal_length_;
      out_x[2] = z;
      ++out_x;

      if (use_color_) {
        cv::Vec3b color = color_img->image.at<cv::Vec3b>(y, x);
        out_rgb[0] = color[0];
        out_rgb[1] = color[1];
        out_rgb[2] = color[2];
        ++out_rgb;
      }
    }
  }
  pub_.publish(cloud);
}
} // namespcae unreal_airsim::simulator_processor