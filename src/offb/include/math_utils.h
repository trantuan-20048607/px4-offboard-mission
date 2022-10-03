#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Eigen>

#include "simd.h"

// 从欧拉角 (roll, pitch, yaw) 创建四元数
inline Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d &e) {
  return {Eigen::AngleAxisd(e.z(), Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(e.y(), Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(e.x(), Eigen::Vector3d::UnitX())};
}

// 将四元数转换至欧拉角 (roll, pitch, yaw)
inline Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond &q) {
  double q_arr[4] = {q.w(), q.x(), q.y(), q.z()};
  return {
      static_cast<double>(simd::atan2_f(
          2.f * static_cast<float>(q_arr[3] * q_arr[2] + q_arr[0] * q_arr[1]),
          1.f - 2.f * static_cast<float>(q_arr[1] * q_arr[1] +
                                         q_arr[2] * q_arr[2]))),
      asin(2.0 * (q_arr[2] * q_arr[0] - q_arr[3] * q_arr[1])),
      static_cast<double>(simd::atan2_f(
          2.f * static_cast<float>(q_arr[3] * q_arr[0] + q_arr[1] * q_arr[2]),
          1.f - 2.f * static_cast<float>(q_arr[2] * q_arr[2] +
                                         q_arr[3] * q_arr[3])))};
}

#endif
