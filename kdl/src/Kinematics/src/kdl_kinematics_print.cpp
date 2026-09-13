// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：kdl_kinematics_print.hpp 中声明的函数在这里落地。

#include "kdl_kinematics_print.hpp"

#include <cmath>
#include <ostream>
#include <string>

namespace kdl_kinematics
{
namespace
{

/**
 * @brief 把浮点噪声清理成漂亮的 0 / ±1，只用于打印展示。
 * @param value 原始数值。
 * @return 接近 0 或 ±1 时返回精确值，否则原样返回。
 *
 * @note URDF 里的 rpy 经矩阵运算后常带 1e-6 级别的残差，
 *       直接打印会干扰阅读，所以这里做一次"仅供显示"的取整。
 */
double tidy(double value)
{
  constexpr double kEps = 1e-9;
  if (std::abs(value) < kEps) {
    return 0.0;
  }
  if (std::abs(value - 1.0) < kEps) {
    return 1.0;
  }
  if (std::abs(value + 1.0) < kEps) {
    return -1.0;
  }
  return value;
}

}  // namespace

void printFrame(const KDL::Frame & frame, std::ostream & os)
{
  // RPY：绕固定轴 X-Y-Z 依次转 roll、pitch、yaw，最直观的姿态表示。
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  frame.M.GetRPY(roll, pitch, yaw);

  os << "[Frame]\n";
  os << "  position = [" << tidy(frame.p.x()) << ", " << tidy(frame.p.y()) << ", "
     << tidy(frame.p.z()) << "]\n";
  os << "  rpy      = [" << tidy(roll) << ", " << tidy(pitch) << ", " << tidy(yaw) << "]\n";

  // KDL::Frame / KDL::Rotation 都没有 operator<<，用 M(i, j) 逐元素取出。
  os << "  rotation =\n";
  for (int i = 0; i < 3; ++i) {
    os << "    [";
    for (int j = 0; j < 3; ++j) {
      os << tidy(frame.M(i, j)) << (j < 2 ? ", " : "");
    }
    os << "]\n";
  }
}

void printJntArray(const KDL::JntArray & q, std::ostream & os)
{
  os << "[JntArray] " << q.rows() << " joints\n  q = [";
  for (unsigned int i = 0; i < q.rows(); ++i) {
    os << q(i) << (i + 1 < q.rows() ? ", " : "");
  }
  os << "]\n";
}

void printJacobian(const KDL::Jacobian & jacobian, std::ostream & os)
{
  os << "[Jacobian] " << jacobian.rows() << " x " << jacobian.columns() << "\n";
  for (unsigned int i = 0; i < jacobian.rows(); ++i) {
    os << "  [";
    for (unsigned int j = 0; j < jacobian.columns(); ++j) {
      os << jacobian(i, j) << (j + 1 < jacobian.columns() ? ", " : "");
    }
    os << "]\n";
  }
  os << "  (前 3 行 = 线速度 v，后 3 行 = 角速度 ω；基座坐标系，参考点在末端)\n";
}

void printIkResult(const IkResult & result, std::ostream & os)
{
  os << "[IkResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  printJntArray(result.q, os);
}

void printTwist(const KDL::Twist & twist, std::ostream & os)
{
  // 借用速度模块的 splitTwist()：加速度同样是 6 维 Twist，用法完全一样。
  const CartesianVector cartesian = splitTwist(twist);

  os << "[Twist]\n";
  os << "  linear  = [" << tidy(cartesian.linear.x()) << ", "
     << tidy(cartesian.linear.y()) << ", " << tidy(cartesian.linear.z()) << "]\n";
  os << "  angular = [" << tidy(cartesian.angular.x()) << ", "
     << tidy(cartesian.angular.y()) << ", " << tidy(cartesian.angular.z()) << "]\n";
}

void printVelocityResult(const VelocityResult & result, std::ostream & os)
{
  os << "[VelocityResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  if (result.singular()) {
    os << "  (奇异警告：伪逆退化，关节速度可能不可信)\n";
  }
  os << "  sigma_min = " << result.sigma_min << "  (越接近 0 越接近奇异位形)\n";
  printJntArray(result.qdot, os);
}

void printAccelerationResult(const AccelerationResult & result, std::ostream & os)
{
  os << "[AccelerationResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  if (result.singular()) {
    os << "  (奇异警告：伪逆退化，关节加速度可能不可信)\n";
  }
  os << "  sigma_min = " << result.sigma_min << "  (越接近 0 越接近奇异位形)\n";
  printJntArray(result.qddot, os);
}

}  // namespace kdl_kinematics
