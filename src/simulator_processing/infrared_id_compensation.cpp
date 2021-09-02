#include "unreal_airsim/simulator_processing/infrared_id_compensation.h"

#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "3rd_party/csv.h"
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
  if (nh_.hasParam(ns + "correction_file")) {
    // Allow reading the corrections from file
    std::string correction_file;
    nh_.getParam(ns + "correction_file", correction_file);
    io::CSVReader<2> in(correction_file);
    in.read_header(io::ignore_extra_column, "MeshID", "InfraRedID");
    int mesh, ir;
    std::fill_n(infrared_compensation_, 256, 255);
    int previous = -1;
    while (in.read_row(mesh, ir)) {
      if (ir > previous) {
        infrared_compensation_[ir] = mesh;
        previous = ir;
      }
    }
    LOG(INFO) << "Read infrared corrections from file '" << correction_file
              << "'.";
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
  try {
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(msg, msg->encoding);

    for (int v = 0; v < img->image.rows; v++) {
      for (int u = 0; u < img->image.cols; u++) {
        img->image.at<uchar>(v, u) =
            infrared_compensation_[img->image.at<uchar>(v, u)];
      }
    }
    pub_.publish(img->toImageMsg());
  } catch (const cv_bridge::Exception& e) {
    std::cerr << "Could not convert image to CvImage\n";
    std::cout << e.what();
  }
}

}  // namespace unreal_airsim::simulator_processor
