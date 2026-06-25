#pragma once

#include <utility>

#include <Eigen/Dense>

namespace roboplan {

/// @brief Computes the position (meters) and orientation (radians) error between two
/// SE(3) transforms expressed in the same frame.
/// @param a The first transform.
/// @param b The second transform.
/// @return A pair of {position error, orientation error}.
std::pair<double, double> poseError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b);

/// @brief Interpolates between two SE(3) transforms: linear in position, SLERP in orientation.
/// @param start The transform at fraction 0.
/// @param end The transform at fraction 1.
/// @param fraction The interpolation coefficient, between 0 and 1.
/// @return The interpolated transform.
Eigen::Matrix4d interpolatePose(const Eigen::Matrix4d& start, const Eigen::Matrix4d& end,
                                double fraction);

/// @brief Computes the transform of `a` expressed in the frame of `b`: T_rel = b^{-1} * a.
/// @details Uses the rigid-transform inverse (R_b^T, -R_b^T p_b), so both inputs must be valid
/// homogeneous transforms (orthonormal rotation block, bottom row [0 0 0 1]).
/// @param a The transform to express in b's frame.
/// @param b The reference frame.
/// @return The 4x4 relative transform b^{-1} * a.
Eigen::Matrix4d relativeTransform(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b);

/// @brief Extracts extrinsic XYZ Euler angles (fixed-axis roll-pitch-yaw) from a rotation matrix.
/// @details The returned angles reconstruct the rotation as R = Rz(yaw) * Ry(pitch) * Rx(roll)
/// (extrinsic XYZ is identical to intrinsic ZYX). Uses atan2 so the result is continuous around
/// zero. Singular at pitch = +/- pi/2 (gimbal lock).
/// @param rotation A 3x3 rotation matrix.
/// @return [roll (X), pitch (Y), yaw (Z)] in radians.
Eigen::Vector3d rotationToExtrinsicEuler(const Eigen::Matrix3d& rotation);

}  // namespace roboplan
