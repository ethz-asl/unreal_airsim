#include "unreal_airsim/online_simulator/simulator.h"

#include <glog/logging.h>
#include <csignal>


// A little ugly but lets the simulator shutdown in a controlled fashion
unreal_airsim::AirsimSimulator* the_simulator = nullptr;
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
  the_simulator = new unreal_airsim::AirsimSimulator(nh, nh_private); // After this the process is killed, no memory leak

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();
  return 0;
}