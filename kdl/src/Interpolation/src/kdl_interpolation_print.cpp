// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：kdl_interpolation_print.hpp 中声明的函数在这里落地。

#include "kdl_interpolation_print.hpp"

#include <cmath>
#include <ostream>
#include <string>

namespace kdl_interpolation
{
namespace
{

/**
 * @brief 把浮点噪声清理成漂亮的 0，只用于打印展示。
 * @param value 原始数值。
 * @return 接近 0 时返回精确的 0，否则原样返回。
 *
 * @note 路点处的速度、加速度是解线性方程组得来的，本来应该是 0 的地方
 *       常带 1e-17 级别的残差，直接打印会干扰阅读。
 */
double tidy(double value)
{
  constexpr double kEps = 1e-12;
  return std::abs(value) < kEps ? 0.0 : value;
}

}  // namespace

void printJointVector(const KDL::JntArray & values, std::ostream & os)
{
  os << "[";
  for (unsigned int i = 0; i < values.rows(); ++i) {
    os << values(i) << (i + 1 < values.rows() ? ", " : "");
  }
  os << "]\n";
}

void printQuinticCoefficients(const QuinticCoefficients & coefficients, std::ostream & os)
{
  os << "[QuinticCoefficients] T = " << coefficients.duration
     << " s,  q(τ) = Σ c_k·τ^k（对 τ = t/T 的系数）\n";
  for (unsigned int k = 0; k < 6; ++k) {
    os << "  c" << k << " = " << tidy(coefficients.c[k]) << "\n";
  }
}

void printJointLimits(const JointLimits & limits, std::ostream & os)
{
  os << "[JointLimits]\n";

  const auto print_bound = [&](const KDL::JntArray & bound, const char * name, const char * unit) {
    os << "  " << name << " [";
    if (bound.rows() == 0) {
      os << "not checked]\n";
      return;
    }
    for (unsigned int i = 0; i < bound.rows(); ++i) {
      os << bound(i) << (i + 1 < bound.rows() ? ", " : "");
    }
    os << "] " << unit << "\n";
  };

  print_bound(limits.max_velocity, "max_velocity    ", "rad/s");
  print_bound(limits.max_acceleration, "max_acceleration", "rad/s²");
  print_bound(limits.max_jerk, "max_jerk        ", "rad/s³");
}

void printTrajectorySummary(const QuinticTrajectory & trajectory, std::ostream & os)
{
  if (!trajectory.valid()) {
    os << "[QuinticTrajectory] 无效（尚未成功构建）\n";
    return;
  }

  os << "[QuinticTrajectory] " << trajectory.joints() << " 关节, " << trajectory.segmentCount()
     << " 段, 总时长 " << trajectory.duration() << " s\n";
  os << "  各段时长 [s]: [";
  for (unsigned int k = 0; k < trajectory.segmentCount(); ++k) {
    os << trajectory.segmentDuration(k) << (k + 1 < trajectory.segmentCount() ? ", " : "");
  }
  os << "]\n";
}

void printKnotTable(const QuinticTrajectory & trajectory, std::ostream & os)
{
  if (!trajectory.valid()) {
    os << "[KnotTable] 无效（尚未成功构建）\n";
    return;
  }

  os << "[KnotTable] 路点处的解（速度/加速度为相邻两段共用，故连续性自动成立）\n";
  const std::vector<double> & times = trajectory.knotTimes();
  for (std::size_t i = 0; i < times.size(); ++i) {
    os << "  #" << i << " (t = " << times[i] << " s)\n";
    os << "    q     (rad)    ";
    printJointVector(trajectory.waypoints()[i], os);
    os << "    qdot  (rad/s)  ";
    printJointVector(trajectory.knotVelocities()[i], os);
    os << "    qddot (rad/s²) ";
    printJointVector(trajectory.knotAccelerations()[i], os);
  }
}

void printTrajectorySample(
  double t, const KDL::JntArray & q, const KDL::JntArray & qdot, const KDL::JntArray & qddot,
  std::ostream & os)
{
  os << "  t = " << t << " s\n";
  os << "    q     (rad)    ";
  printJointVector(q, os);
  os << "    qdot  (rad/s)  ";
  printJointVector(qdot, os);
  os << "    qddot (rad/s²) ";
  printJointVector(qddot, os);
}

void printTrajectoryResult(const TrajectoryResult & result, std::ostream & os)
{
  if (result.success) {
    os << "[TrajectoryResult] success（轨迹可行）\n";
    return;
  }
  os << "[TrajectoryResult] FAILED: " << result.message << "\n";
}

}  // namespace kdl_interpolation
