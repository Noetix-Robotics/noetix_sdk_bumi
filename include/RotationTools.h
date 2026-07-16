#pragma once

#include <chrono>
#include "Types.h"

/**
 * Compute the rotation matrix corresponding to euler angles zyx
 *
 * @param [in] eulerAnglesZyx
 * @return The corresponding rotation matrix
 */
template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 3> getRotationMatrixFromZyxEulerAngles(const Eigen::Matrix<SCALAR_T, 3, 1>& eulerAngles) {
  const SCALAR_T z = eulerAngles(0);
  const SCALAR_T y = eulerAngles(1);
  const SCALAR_T x = eulerAngles(2);

  const SCALAR_T c1 = cos(z);
  const SCALAR_T c2 = cos(y);
  const SCALAR_T c3 = cos(x);
  const SCALAR_T s1 = sin(z);
  const SCALAR_T s2 = sin(y);
  const SCALAR_T s3 = sin(x);

  const SCALAR_T s2s3 = s2 * s3;
  const SCALAR_T s2c3 = s2 * c3;

  // clang-format off
  Eigen::Matrix<SCALAR_T, 3, 3> rotationMatrix;
  rotationMatrix << c1 * c2,      c1 * s2s3 - s1 * c3,       c1 * s2c3 + s1 * s3,
                    s1 * c2,      s1 * s2s3 + c1 * c3,       s1 * s2c3 - c1 * s3,
                        -s2,                  c2 * s3,                   c2 * c3;
  // clang-format on
  return rotationMatrix;
}

template <typename T>
T square(T a) {
  return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> zyx;

  SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
  zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
  zyx(1) = std::asin(as);
  zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
  return zyx;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToXyz(const Eigen::Quaternion<SCALAR_T>& q) {
  Eigen::Matrix<SCALAR_T, 3, 1> xyz;

  // Roll (X-axis rotation)
    SCALAR_T sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    SCALAR_T cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    xyz(0) = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-axis rotation)
    SCALAR_T sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1)
        xyz(1) = std::copysign(M_PI / 2, sinp); // 使用copysign来处理极端情况
    else
        xyz(1) = std::asin(sinp);

    // Yaw (Z-axis rotation)
    SCALAR_T siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    SCALAR_T cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    xyz(2) = std::atan2(siny_cosp, cosy_cosp);

  return xyz;
}

template <typename SCALAR_T>
Eigen::Quaterniond matrix_to_quaternion_eigen(const Eigen::Matrix3d& r1) {
    Eigen::Quaterniond q(r1);
    q.normalize();
    return q;
}

template <typename SCALAR_T>
// 提取偏航分量的四元数函数
Eigen::Quaterniond yaw_quat(const Eigen::Quaterniond& q) {
  double w = q.w(), x = q.x(), y = q.y(), z = q.z();
  
  // 计算偏航角 (yaw)
  double yaw = std::atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  
  // 返回只包含偏航旋转的四元数
  return Eigen::Quaterniond(
      std::cos(yaw / 2),  // w
      0,                  // x
      0,                  // y
      std::sin(yaw / 2)   // z
  );
}

template <typename SCALAR_T>
Eigen::Quaterniond subtract_frame_transforms(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2, const Eigen::Quaterniond& init_to_world) {

    Eigen::Quaterniond q1_norm = q1.normalized();
    Eigen::Quaterniond q2_norm = q2.normalized();
    Eigen::Quaterniond init_to_world_norm = init_to_world.normalized();

    Eigen::Quaterniond delta_real = init_to_world_norm * q2_norm;

    Eigen::Quaterniond delta_q = q1_norm.conjugate() * delta_real;
    return delta_q.normalized();
}

template <typename SCALAR_T>
Eigen::Matrix3d QuatToMat(const Eigen::Quaterniond& q) {
    return q.normalized().toRotationMatrix();
}
