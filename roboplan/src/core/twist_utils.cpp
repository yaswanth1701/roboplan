#include <roboplan/core/twist_utils.hpp>

#include <cmath>

namespace roboplan {

Eigen::Matrix3d eulerRateToAngularVelocityMatrix(const Eigen::Vector3d& rate) {
  // Note: `rate` here carries the current Euler ANGLES [roll, pitch, yaw]; the matrix is evaluated
  // at the angles (pitch, yaw) and then multiplied by an Euler-rate vector by the caller.
  const double pitch = rate(1);
  const double yaw = rate(2);

  Eigen::Matrix3d e;
  e << std::cos(yaw) * std::cos(pitch), -std::sin(yaw), 0.0,  //
      std::sin(yaw) * std::cos(pitch), std::cos(yaw), 0.0,    //
      -std::sin(pitch), 0.0, 1.0;
  return e;
}

}  // namespace roboplan
