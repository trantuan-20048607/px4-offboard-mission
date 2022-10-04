#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <Eigen/Eigen>

#if defined(__x86_64__)
#include <emmintrin.h>
#endif
#if defined(__aarch64__)
#include "simd/neon.h"
#endif
#if defined(__x86_64__) | defined(__aarch64__)
#define USE_SSE2
#include "simd/sse.h"
#else
#include <cmath>
#endif

namespace math_utils {

namespace simd {

inline void sin_cos_4f(const float x[4], float s[4], float c[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, s_v4sf, c_v4sf;
  sincos_ps(x_v4sf, &s_v4sf, &c_v4sf);
  auto s_ptr = (float *)&s_v4sf, c_ptr = (float *)&c_v4sf;
  s[0] = *s_ptr++;
  c[0] = *c_ptr++;
  s[1] = *s_ptr++;
  c[1] = *c_ptr++;
  s[2] = *s_ptr++;
  c[2] = *c_ptr++;
  s[3] = *s_ptr;
  c[3] = *c_ptr;
#else
  for (auto i = 0; i < 4; i++) {
    s[i] = sinf(x[i]);
    c[i] = cosf(x[i]);
  }
#endif
}

inline void sin_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, s_v4sf = sin_ps(x_v4sf);
  auto s_ptr = (float *)&s_v4sf;
  x[0] = *s_ptr++;
  x[1] = *s_ptr++;
  x[2] = *s_ptr++;
  x[3] = *s_ptr;
#else
  for (auto i = 0; i < 4; i++) x[i] = sinf(x[i]);
#endif
}

inline float sin_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x}, s_v4sf = sin_ps(x_v4sf);
  return *(float *)&s_v4sf;
#else
  return sinf(x);
#endif
}

inline void cos_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, c_v4sf = cos_ps(x_v4sf);
  auto c_ptr = (float *)&c_v4sf;
  x[0] = *c_ptr++;
  x[1] = *c_ptr++;
  x[2] = *c_ptr++;
  x[3] = *c_ptr;
#else
  for (auto i = 0; i < 4; i++) x[i] = cosf(x[i]);
#endif
}

inline float cos_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x}, c_v4sf = cos_ps(x_v4sf);
  return *(float *)&c_v4sf;
#else
  return cosf(x);
#endif
}

inline void tan_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = tan_ps(x_v4sf);
  auto t_ptr = (float *)&t_v4sf;
  x[0] = *t_ptr++;
  x[1] = *t_ptr++;
  x[2] = *t_ptr++;
  x[3] = *t_ptr;
#else
  for (auto i = 0; i < 4; i++) x[i] = tanf(x[i]);
#endif
}

inline void cot_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = cot_ps(x_v4sf);
  auto t_ptr = (float *)&t_v4sf;
  x[0] = *t_ptr++;
  x[1] = *t_ptr++;
  x[2] = *t_ptr++;
  x[3] = *t_ptr;
#else
  for (auto i = 0; i < 4; i++) x[i] = 1.f / tanf(x[i]);
#endif
}

inline void atan_4f(float x[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, t_v4sf = atan_ps(x_v4sf);
  auto t_ptr = (float *)&t_v4sf;
  x[0] = *t_ptr++;
  x[1] = *t_ptr++;
  x[2] = *t_ptr++;
  x[3] = *t_ptr;
#else
  for (auto i = 0; i < 4; i++) x[i] = atanf(x[i]);
#endif
}

inline void atan2_4f(const float y[4], const float x[4], float res[4]) {
#if defined(__x86_64__) | defined(__aarch64__)
  v4sf x_v4sf = {x[0], x[1], x[2], x[3]}, y_v4sf = {y[0], y[1], y[2], y[3]};
  v4sf res_v4sf = atan2_ps(y_v4sf, x_v4sf);
  auto res_ptr = (float *)&res_v4sf;
  res[0] = *res_ptr++;
  res[1] = *res_ptr++;
  res[2] = *res_ptr++;
  res[3] = *res_ptr;
#else
  for (auto i = 0; i < 4; i++) res[i] = atan2f(y[i], x[i]);
#endif
}

inline float atan2_f(float y, float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  return atan2_ref(y, x);
#else
  return atan2f(y, x);
#endif
}

inline float sqrt_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  return sqrt_ps(x);
#else
  return sqrtf(x);
#endif
}

inline float rsqrt_f(float x) {
#if defined(__x86_64__) | defined(__aarch64__)
  return rsqrt_ps(x);
#else
  return 1.f / sqrtf(x);
#endif
}
}  // namespace simd

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

}  // namespace math_utils

#endif
