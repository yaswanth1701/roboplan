#include <roboplan/core/twist_utils.hpp>

#include <cmath>

namespace roboplan {

Eigen::Matrix3d eulerRateToAngularVelocityMatrix(const Eigen::Vector3d& rate) {
  const double pitch = rate(1);
  const double yaw = rate(2);

  Eigen::Matrix3d e;
  e << std::cos(yaw) * std::cos(pitch), -std::sin(yaw), 0.0,  
      std::sin(yaw) * std::cos(pitch), std::cos(yaw), 0.0, 
      -std::sin(pitch), 0.0, 1.0;
  return e;
}

}  // namespace roboplan
