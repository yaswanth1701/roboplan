#pragma once

#include <Eigen/Dense>

namespace roboplan {

/// @brief Builds the matrix mapping extrinsic XYZ Euler-angle rates to an angular velocity.
/// @details For an orientation parametrized as R = Rz(yaw) * Ry(pitch) * Rx(roll) (extrinsic XYZ,
/// i.e. fixed-axis roll-pitch-yaw), the angular velocity in the fixed/base frame is
///   omega = E(pitch, yaw) * [roll_rate, pitch_rate, yaw_rate]^T.
/// E depends only on pitch and yaw, and is singular at pitch = +/- pi/2 (gimbal lock), where roll
/// and yaw rates become indistinguishable.
/// @param euler [roll (X), pitch (Y), yaw (Z)] in radians (extrinsic XYZ convention).
/// @return The 3x3 matrix E mapping the corresponding Euler rates to angular velocity (base frame).
Eigen::Matrix3d eulerRateToAngularVelocityMatrix(const Eigen::Vector3d& rate);

}  // namespace roboplan
