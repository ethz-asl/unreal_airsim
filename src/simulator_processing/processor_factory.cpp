#include "unreal_airsim/simulator_processing/processor_factory.h"
#include "unreal_airsim/simulator_processing/processor_base.h"

namespace unreal_airsim::simulator_processor {

ProcessorFactory::TypeMap &ProcessorFactory::getRegistryMap() {
  static ProcessorFactory::TypeMap type_map_;
  return type_map_;
}

std::unique_ptr<ProcessorBase> ProcessorFactory::createFromRos(const std::string &name,
                                                               const std::string &type,
                                                               const ros::NodeHandle &nh,
                                                               const std::string &ns,
                                                               AirsimSimulator *parent) {
  if (getRegistryMap().find(type) == getRegistryMap().end()) {
    const std::string typeList =
        getRegistryMap().empty() ? std::string() : std::accumulate(
            std::next(getRegistryMap().begin()), getRegistryMap().end(),
            getRegistryMap().begin()->first,
            [](std::string &a, std::pair<std::string, FactoryMethod> b) {
              return a + ", " + b.first;
            });
    LOG(ERROR) << "Unknown type '" << type << "' for simulator_processor '" << name << "'. Available types are: "
               << typeList << ".";
    return nullptr;
  }
  ProcessorBase *processor = getRegistryMap().at(type)();
  processor->parent_ = parent;
  processor->name_ = name;
  if (!processor->setupFromRos(nh, ns)) {
    LOG(ERROR) << "Setup for simulator_processor '" << name << "' failed, it will be ignored.";
    return nullptr;
  }
  return std::unique_ptr<ProcessorBase>(processor);
}

} // namespcae unreal_airsim::simulator_processor
