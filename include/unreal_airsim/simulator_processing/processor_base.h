#ifndef UNREAL_AIRSIM_SIMULATOR_PROCESSING_PROCESSOR_BASE_H_
#define UNREAL_AIRSIM_SIMULATOR_PROCESSING_PROCESSOR_BASE_H_

#include "unreal_airsim/simulator_processing/processor_factory.h"

#include <ros/ros.h>

#include <string>

namespace unreal_airsim{
class AirsimSimulator;
namespace simulator_processor {
  /***
  * Base class for all simulator_processors.
  */
  class ProcessorBase {
   public:
    ProcessorBase() = default;
    virtual ~ProcessorBase() = default;

    virtual bool setupFromRos(const ros::NodeHandle &nh, const std::string &ns) = 0;  // params can be retrieved as ns+param_name

   protected:
    // these fields are set by the factory
    friend ProcessorFactory;
    AirsimSimulator* parent_;
    std::string name_;
  };

} // namespcae simulator_processor
} // namespcae unreal_airsim

#endif // UNREAL_AIRSIM_SIMULATOR_PROCESSING_PROCESSOR_BASE_H_
