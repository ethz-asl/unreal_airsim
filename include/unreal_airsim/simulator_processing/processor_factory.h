#ifndef UNREAL_AIRSIM_SIMULATOR_PROCESSOR_PROCESSOR_FACTORY_H_
#define UNREAL_AIRSIM_SIMULATOR_PROCESSOR_PROCESSOR_FACTORY_H_

#include <functional>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <utility>

#include <glog/logging.h>
#include <ros/node_handle.h>

namespace unreal_airsim {
class AirsimSimulator;

namespace simulator_processor {
class ProcessorBase;

/***
 * Static Factory with a singleton registration map. Processors can register
 * themselves with a name using the static Registration struct, such that they
 * can then be created via the name at runtime.
 */
class ProcessorFactory {
 public:
  typedef std::function<ProcessorBase*()> FactoryMethod;
  virtual ~ProcessorFactory() = default;

  // Allow processors to register themselves to the factory using a static
  // Registration struct
  template <class ProcessorType>
  struct Registration {
    explicit Registration(const std::string& name) {
      if (getRegistryMap().find(name) == getRegistryMap().end()) {
        getRegistryMap().insert(std::pair<std::string, FactoryMethod>(
            name, []() { return new ProcessorType(); }));
      } else {
        LOG(FATAL)
            << "Cannot register already existing simulator_processor name '"
            << name << "'!";
      }
    }
  };

  static std::unique_ptr<ProcessorBase> createFromRos(const std::string& name,
                                                      const std::string& type,
                                                      const ros::NodeHandle& nh,
                                                      const std::string& ns,
                                                      AirsimSimulator* parent);

 private:
  ProcessorFactory() = default;
  typedef std::unordered_map<std::string, FactoryMethod> TypeMap;
  static TypeMap& getRegistryMap();
};

}  // namespace simulator_processor

}  // namespace unreal_airsim

#endif  // UNREAL_AIRSIM_SIMULATOR_PROCESSOR_PROCESSOR_FACTORY_H_
