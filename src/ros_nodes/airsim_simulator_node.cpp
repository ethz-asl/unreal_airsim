#include "unreal_airsim/online_simulator/simulator.h"

#include <glog/logging.h>
#include <csignal>
#include <memory>


// Lets the simulator shutdown in a controlled fashion
std::unique_ptr<unreal_airsim::AirsimSimulator> the_simulator;
void sigintHandler(int sig) {
  if(the_simulator){
    the_simulator->onShutdown();
  }
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "airsim_simulator", ros::init_options::NoSigintHandler);

  // Setup logging
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();
  google::ParseCommandLineFlags(&argc, &argv, false);

  // Run node
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");
  signal(SIGINT, sigintHandler);
  the_simulator = std::make_unique<unreal_airsim::AirsimSimulator>(nh, nh_private);

  int n_threads;
  nh_private.param("n_threads", n_threads, 0);  // 0 defaults to #physical cores
  ros::MultiThreadedSpinner spinner(n_threads);
  spinner.spin();
  return 0;
}