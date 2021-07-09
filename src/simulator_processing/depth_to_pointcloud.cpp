#include "unreal_airsim/simulator_processing/depth_to_pointcloud.h"

#include <cmath>
#include <deque>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "unreal_airsim/online_simulator/simulator.h"

namespace unreal_airsim::simulator_processor {

ProcessorFactory::Registration<DepthToPointcloud>
    DepthToPointcloud::registration_("DepthToPointcloud");

bool DepthToPointcloud::setupFromRos(const ros::NodeHandle& nh,
                                     const std::string& ns) {
  nh_ = nh;
  use_color_ = false;
  is_setup_ = false;

  // Get params.
  std::string depth_camera_name, color_camera_name, segmentation_camera_name,
      output_topic, depth_topic, color_topic, segmentation_topic;
  nh.param(ns + "max_queue_length", max_queue_length_, 10);
  nh.param(ns + "max_depth", max_depth_, 1e6f);
  nh.param(ns + "max_ray_length", max_ray_length_, 1e6f);
  nh.param(ns + "output_topic", output_topic,
           parent_->getConfig().vehicle_name + "/" + name_);
  if (nh.hasParam(ns + "depth_camera_name")) {
    nh.getParam(ns + "depth_camera_name", depth_camera_name);
  } else if (nh.hasParam(ns + "depth_topic")) {
    nh.getParam(ns + "depth_camera_topic", depth_topic);
  } else {
    LOG(FATAL) << "DepthToPointcloud requires the 'depth_camera_name' or "
                  "'depth_camera_topic' param to be set!";
    return false;
  }
  if (nh.hasParam(ns + "color_camera_name")) {
    nh.getParam(ns + "color_camera_name", color_camera_name);
    use_color_ = true;
  } else if (nh.hasParam(ns + "color_camera_topic")) {
    nh.getParam(ns + "color_camera_topic", color_topic);
    use_color_ = true;
  }
  if (nh.hasParam(ns + "segmentation_camera_name")) {
    nh.getParam(ns + "segmentation_camera_name", segmentation_camera_name);
    use_segmentation_ = true;
  } else if (nh.hasParam(ns + "segmentation_camera_topic")) {
    nh.getParam(ns + "segmentation_camera_topic", segmentation_topic);
    use_segmentation_ = true;
  }

  // Find source cameras.
  for (const auto& sensor : parent_->getConfig().sensors) {
    if (sensor->name == depth_camera_name) {
      depth_topic = sensor->output_topic;
      auto camera =
          (unreal_airsim::AirsimSimulator::Config::Camera*)sensor.get();
      fov_ = camera->camera_info.fov;
    }
    if (sensor->name == color_camera_name) {
      color_topic = sensor->output_topic;
    }
    if (sensor->name == segmentation_camera_name) {
      segmentation_topic = sensor->output_topic;
    }
  }
  if (depth_topic.empty()) {
    LOG(FATAL) << "Could not find a Camera with name '" << depth_camera_name
               << "'!";
    return false;
  }
  if (color_topic.empty() && use_color_) {
    LOG(FATAL) << "Could not find a Camera with name '" << color_camera_name
               << "'!";
    return false;
  }
  if (segmentation_topic.empty() && use_segmentation_) {
    LOG(FATAL) << "Could not find a Camera with name '"
               << segmentation_camera_name << "'!";
    return false;
  }

  // Ros.
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 5);
  depth_sub_ = nh_.subscribe(depth_topic, max_queue_length_,
                             &DepthToPointcloud::depthImageCallback, this);
  if (use_color_) {
    color_sub_ = nh_.subscribe(color_topic, max_queue_length_,
                               &DepthToPointcloud::colorImageCallback, this);
  }
  if (use_segmentation_) {
    segmentation_sub_ =
        nh_.subscribe(segmentation_topic, max_queue_length_,
                      &DepthToPointcloud::segmentationImageCallback, this);
  }
  return true;
}

void DepthToPointcloud::depthImageCallback(const sensor_msgs::ImagePtr& msg) {
  // Use the first depth image to initialize intrinsics.
  if (!is_setup_) {
    vx_ = msg->width / 2;
    vy_ = msg->height / 2;
    focal_length_ =
        static_cast<float>(msg->width) / (2.0 * std::tan(fov_ * M_PI / 360.0));
    is_setup_ = true;
  }

  // Store depth img in queue.
  depth_queue_.push_back(msg);
  if (depth_queue_.size() > max_queue_length_) {
    depth_queue_.pop_front();
  }
  findMatchingMessagesToPublish(msg);
}

void DepthToPointcloud::colorImageCallback(const sensor_msgs::ImagePtr& msg) {
  // Store color img in queue.
  color_queue_.push_back(msg);
  if (color_queue_.size() > max_queue_length_) {
    color_queue_.pop_front();
  }
  findMatchingMessagesToPublish(msg);
}

void DepthToPointcloud::segmentationImageCallback(
    const sensor_msgs::ImagePtr& msg) {
  // Store segmentation img in queue.
  segmentation_queue_.push_back(msg);
  if (segmentation_queue_.size() > max_queue_length_) {
    segmentation_queue_.pop_front();
  }
  findMatchingMessagesToPublish(msg);
}

void DepthToPointcloud::findMatchingMessagesToPublish(
    const sensor_msgs::ImagePtr& reference_msg) {
  std::deque<sensor_msgs::ImagePtr>::iterator depth_it, color_it,
      segmentation_it;
  // If a modality is not used we count it as found.
  bool found_color = true;
  bool found_segmentation = true;

  depth_it =
      std::find_if(depth_queue_.begin(), depth_queue_.end(),
                   [reference_msg](const sensor_msgs::ImagePtr& s) {
                     return s->header.stamp == reference_msg->header.stamp;
                   });
  if (depth_it == depth_queue_.end()) {
    // Depth image is always necessary.
    return;
  }

  if (use_color_) {
    // Check whether there is a color image with matching timestamp.
    color_it =
        std::find_if(color_queue_.begin(), color_queue_.end(),
                     [reference_msg](const sensor_msgs::ImagePtr& s) {
                       return s->header.stamp == reference_msg->header.stamp;
                     });
    if (color_it == color_queue_.end()) {
      found_color = false;
    }
  }

  if (use_segmentation_) {
    // Check whether there is a segmentation image with matching timestamp.
    segmentation_it =
        std::find_if(segmentation_queue_.begin(), segmentation_queue_.end(),
                     [reference_msg](const sensor_msgs::ImagePtr& s) {
                       return s->header.stamp == reference_msg->header.stamp;
                     });
    if (segmentation_it == segmentation_queue_.end()) {
      found_segmentation = false;
    }
  }

  if (found_color && found_segmentation) {
    publishPointcloud(*depth_it, use_color_ ? *color_it : nullptr,
                      use_segmentation_ ? *segmentation_it : nullptr);
    depth_queue_.erase(depth_it);
    if (use_color_) {
      color_queue_.erase(color_it);
    }
    if (use_segmentation_) {
      segmentation_queue_.erase(segmentation_it);
    }
  }
}

void DepthToPointcloud::publishPointcloud(
    const sensor_msgs::ImagePtr& depth_ptr,
    const sensor_msgs::ImagePtr& color_ptr,
    const sensor_msgs::ImagePtr& segmentation_ptr) {
  /**
   * NOTE(schmluk): This method assumes that all images are from the same
   * simulated camera, i.e. are perfectly aligned and have identical settings
   * (resolution, intrinsics, extrinsics). This assumes we use planar depth
   * camera (ImageType::DepthPlanar) with image data as floats. We further
   * assume that segmentation is created using infrared (ImageType::Infrared),
   * i.e. the labels are provided in the first channel of the segmentation
   * image.
   */
  if (pub_.getNumSubscribers() == 0) {
    return;
  }
  cv_bridge::CvImageConstPtr depth_img, color_img, segmentation_img;
  depth_img = cv_bridge::toCvShare(depth_ptr, depth_ptr->encoding);
  if (use_color_) {
    color_img = cv_bridge::toCvShare(color_ptr, color_ptr->encoding);
  }
  if (use_segmentation_) {
    segmentation_img =
        cv_bridge::toCvShare(segmentation_ptr, segmentation_ptr->encoding);
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
  cloud.is_dense = false;  // It is safer to assume there may be invalid points
                           // since I did not see proof otherwise

  // fields setup
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  if (use_color_) {
    if (use_segmentation_) {
      modifier.setPointCloud2Fields(5, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "rgb", 1, sensor_msgs::PointField::FLOAT32,
                                    "id", 1, sensor_msgs::PointField::UINT8);
    } else {
      modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "rgb", 1, sensor_msgs::PointField::FLOAT32);
    }
  } else {
    if (use_segmentation_) {
      modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32,
                                    "id", 1, sensor_msgs::PointField::UINT8);
    } else {
      modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                    "y", 1, sensor_msgs::PointField::FLOAT32,
                                    "z", 1, sensor_msgs::PointField::FLOAT32);
    }
  }
  modifier.resize(numpoints);

  sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> out_rgb;
  std::unique_ptr<sensor_msgs::PointCloud2Iterator<uint8_t>> out_seg;
  if (use_color_) {
    out_rgb = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(
        cloud, "rgb");
  }
  if (use_segmentation_) {
    out_seg = std::make_unique<sensor_msgs::PointCloud2Iterator<uint8_t>>(cloud,
                                                                          "id");
  }
  for (int v = 0; v < depth_img->image.rows; v++) {
    for (int u = 0; u < depth_img->image.cols; u++) {
      float z = depth_img->image.at<float>(v, u);
      if (z > max_depth_) {
        continue;
      }
      float x = (static_cast<float>(u) - vx_) * z / focal_length_;
      float y = (static_cast<float>(v) - vy_) * z / focal_length_;
      if (max_ray_length_ > 0.0) {
        float dist_square = x * x + y * y + z * z;
        if (dist_square > max_ray_length_ * max_ray_length_) {
          continue;
        }
      }
      out_x[0] = x;
      out_y[0] = y;
      out_z[0] = z;
      ++out_x;
      ++out_y;
      ++out_z;

      if (use_color_) {
        cv::Vec3b color = color_img->image.at<cv::Vec3b>(v, u);
        (*out_rgb)[0] = color[0];
        (*out_rgb)[1] = color[1];
        (*out_rgb)[2] = color[2];
        ++(*out_rgb);
      }

      if (use_segmentation_) {
        (*out_seg)[0] = segmentation_img->image.at<uchar>(v, u);
        ++(*out_seg);
      }
    }
  }
  pub_.publish(cloud);
}

}  // namespace unreal_airsim::simulator_processor
