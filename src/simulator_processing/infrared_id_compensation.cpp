#include "unreal_airsim/simulator_processing/infrared_id_compensation.h"

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>

#include "unreal_airsim/online_simulator/simulator.h"

namespace unreal_airsim::simulator_processor {

ProcessorFactory::Registration<InfraredIdCompensation>
    InfraredIdCompensation::registration_("InfraredIdCompensation");

bool InfraredIdCompensation::setupFromRos(const ros::NodeHandle& nh,
                                          const std::string& ns) {
  nh_ = nh;
  if (!nh_.hasParam(ns + "input_topic")) {
    LOG(ERROR)
        << "InfraredIdCompensation requires the 'input_topic' to be set.";
    return false;
  }
  if (!nh_.hasParam(ns + "output_topic")) {
    LOG(ERROR)
        << "InfraredIdCompensation requires the 'output_topic' to be set.";
    return false;
  }
  std::string input_topic, output_topic;
  nh_.getParam(ns + "input_topic", input_topic);
  nh_.getParam(ns + "output_topic", output_topic);
  pub_ = nh_.advertise<sensor_msgs::Image>(output_topic, 10);
  sub_ = nh_.subscribe(input_topic, 10, &InfraredIdCompensation::imageCallback,
                       this);
  return true;
}

void InfraredIdCompensation::imageCallback(const sensor_msgs::ImagePtr& msg) {
  cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, msg->encoding);
  for (int v = 0; v < img->image.rows; v++) {
    for (int u = 0; u < img->image.cols; u++) {
      img->image.at<uchar>(v, u) =
          infrared_compensation_[img->image.at<uchar>(v, u)];
    }
  }
  pub_.publish(img->toImageMsg());
}

}  // namespace unreal_airsim::simulator_processor
