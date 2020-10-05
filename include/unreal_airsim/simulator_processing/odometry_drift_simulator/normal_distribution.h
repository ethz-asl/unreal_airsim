#ifndef UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_
#define UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_

#include <ros/ros.h>
#include <random>
#include <string>

#include <glog/logging.h>

namespace unreal_airsim {
class NormalDistribution {
 public:
  struct Config {
    // Initialize from ROS params
    static Config fromRosParams(const ros::NodeHandle& nh);

    // Distribution parameters
    double mean = 0.0;
    double stddev = 0.0;
    double truncate_at_n_stddevs = 10;

    // Validity queries and assertions
    bool isValid(const std::string& error_msg_prefix = "") const;
    const Config& checkValid() const {
      CHECK(isValid());
      return *this;
    }

    // Write config values to stream, e.g. for logging
    friend std::ostream& operator<<(std::ostream& os, const Config& config);
  };

  explicit NormalDistribution(double mean = 0.0, double stddev = 1.0,
                              double truncation_radius = 1e6)
      : NormalDistribution(Config{mean, stddev, truncation_radius}) {}
  explicit NormalDistribution(Config config)
      : config_(config.checkValid()),
        truncation_radius_(config.stddev * config.truncate_at_n_stddevs) {}

  double getMean() const { return config_.mean; }
  double getStddev() const { return config_.stddev; }
  double getTruncationRadius() const { return config_.truncate_at_n_stddevs; }

  // Return a sample from the normal distribution N(mean_, stddev_)
  double operator()() {
    // Random engine
    // TODO(victorr): Add proper random seed handling (and option to provide it)
    // NOTE: The noise generator is static to ensure that all instances draw
    //       subsequent (different) numbers from the same pseudo random
    //       sequence. If the generator is instance specific, there's a risk
    //       that multiple instances use generators with the same seed and
    //       output the same sequence.
    static std::mt19937 noise_generator_;

    // Draw a sample from the standard normal N(0,1), scale it using the change
    // of variables formula. If it falls outside the truncation radius, redraw.
    double sample =
        normal_distribution_(noise_generator_) * config_.stddev + config_.mean;
    if (std::abs(sample - config_.mean) <= truncation_radius_) {
      return sample;
    } else {
      return operator()();
    };
  }

 private:
  const Config config_;
  const double truncation_radius_;

  // Standard normal distribution
  std::normal_distribution<double> normal_distribution_;
};
}  // namespace unreal_airsim

#endif  // UNREAL_AIRSIM_SIMULATOR_PROCESSING_ODOMETRY_DRIFT_SIMULATOR_NORMAL_DISTRIBUTION_H_
