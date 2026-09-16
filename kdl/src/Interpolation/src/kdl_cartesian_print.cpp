// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：kdl_cartesian_print.hpp 中声明的函数在这里落地。

#include "kdl_cartesian_print.hpp"

#include <ostream>
#include <string>

namespace kdl_interpolation
{

void printCartesianPose(const KDL::Frame & pose, std::ostream & os)
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  pose.M.GetRPY(roll, pitch, yaw);

  double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
  pose.M.GetQuaternion(qx, qy, qz, qw);

  os << "[CartesianPose]\n";
  os << "  p    (m)   = [" << pose.p.x() << ", " << pose.p.y() << ", " << pose.p.z() << "]\n";
  os << "  RPY  (rad) = [" << roll << ", " << pitch << ", " << yaw << "]\n";
  os << "  quat (x,y,z,w) = [" << qx << ", " << qy << ", " << qz << ", " << qw << "]\n";
}

void printCartesianState(const CartesianState & state, std::ostream & os)
{
  printCartesianPose(state.pose, os);
  os << "  v     (m/s)     = [" << state.linear_velocity.x() << ", " << state.linear_velocity.y()
     << ", " << state.linear_velocity.z() << "]\n";
  os << "  omega (rad/s)   = [" << state.angular_velocity.x() << ", " << state.angular_velocity.y()
     << ", " << state.angular_velocity.z() << "]\n";
  os << "  a     (m/s²)    = [" << state.linear_acceleration.x() << ", "
     << state.linear_acceleration.y() << ", " << state.linear_acceleration.z() << "]\n";
  os << "  alpha (rad/s²)  = [" << state.angular_acceleration.x() << ", "
     << state.angular_acceleration.y() << ", " << state.angular_acceleration.z() << "]\n";
}

void printCartesianSample(double t, const CartesianState & state, std::ostream & os)
{
  os << "  t = " << t << " s\n";
  printCartesianState(state, os);
}

void printCartesianLimits(const CartesianLimits & limits, std::ostream & os)
{
  os << "[CartesianLimits]\n";

  const auto print_bound = [&](double bound, const char * name, const char * unit) {
    os << "  " << name << " [";
    if (bound <= 0.0) {
      os << "not checked]\n";
      return;
    }
    os << bound << "] " << unit << "\n";
  };

  print_bound(limits.max_linear_velocity, "max |v|    ", "m/s");
  print_bound(limits.max_linear_acceleration, "max |a|    ", "m/s²");
  print_bound(limits.max_angular_velocity, "max |omega|", "rad/s");
  print_bound(limits.max_angular_acceleration, "max |alpha|", "rad/s²");
}

void printCartesianSegment(
  const std::array<QuinticCoefficients, 3> & position, const SlerpSegment & orientation,
  std::ostream & os)
{
  os << "[CartesianSegment] T = " << orientation.duration
     << " s（位置系数是对 τ = t/T 的，不是对 t 的）\n";

  static const char * kAxisName[3] = {"x", "y", "z"};
  for (unsigned int i = 0; i < 3; ++i) {
    os << "  位置 " << kAxisName[i] << "(τ) = Σ c_k·τ^k , c = [";
    for (unsigned int k = 0; k < 6; ++k) {
      os << position[i].c[k] << (k + 1 < 6 ? ", " : "");
    }
    os << "]\n";
  }

  os << "  姿态 R(τ) = Rot(axis, θ·s(τ)) · R0（左乘：绕基座系固定轴转）\n";
  os << "    R0 的 RPY (rad) = [";
  {
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    orientation.start_rotation.GetRPY(roll, pitch, yaw);
    os << roll << ", " << pitch << ", " << yaw;
  }
  os << "]\n";
  os << "    转轴 axis (基座系, 单位向量) = [" << orientation.axis_base.x() << ", "
     << orientation.axis_base.y() << ", " << orientation.axis_base.z() << "]\n";
  os << "    转角 θ (rad) = " << orientation.angle;
  if (!orientation.rotating()) {
    os << "  （≈ 0：该段没有姿态变化，轴未定义，按约定取零向量）";
  }
  os << "\n";
}

void printCartesianTrajectorySummary(const CartesianTrajectory & trajectory, std::ostream & os)
{
  if (!trajectory.valid()) {
    os << "[CartesianTrajectory] 无效（尚未成功构建）\n";
    return;
  }

  os << "[CartesianTrajectory] " << trajectory.segmentCount() << " 段, 总时长 "
     << trajectory.duration() << " s\n";
  os << "  路点行为: "
     << (trajectory.waypointBehavior() == WaypointBehavior::kStop ?
           "kStop（位置在每个路点停住，与姿态一致）" :
           "kPassThrough（位置 C⁴ 路过路点不停，姿态仍在路点停住）")
     << "\n";
  os << "  各段时长 [s]: [";
  for (unsigned int k = 0; k < trajectory.segmentCount(); ++k) {
    os << trajectory.segmentDuration(k) << (k + 1 < trajectory.segmentCount() ? ", " : "");
  }
  os << "]\n";
}

void printCartesianOrientationTable(const CartesianTrajectory & trajectory, std::ostream & os)
{
  if (!trajectory.valid()) {
    os << "[CartesianOrientationTable] 无效（尚未成功构建）\n";
    return;
  }

  os << "[CartesianOrientationTable] 每段：起点 RPY / 转轴 / 转角 / 匀速角速度 θ/T\n";
  os << "  段  t0[s]   T[s]      起点 RPY[rad]              转轴(基座系)          θ[rad]   θ/T[rad/s]\n";

  for (unsigned int k = 0; k < trajectory.segmentCount(); ++k) {
    const SlerpSegment & segment = trajectory.orientationSegment(k);
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    segment.start_rotation.GetRPY(roll, pitch, yaw);
    const double rate = segment.duration > 0.0 ? segment.angle / segment.duration : 0.0;

    os << "  " << k << "  " << trajectory.knotTimes()[k] << "  " << segment.duration << "  ["
       << roll << ", " << pitch << ", " << yaw << "]  [" << segment.axis_base.x() << ", "
       << segment.axis_base.y() << ", " << segment.axis_base.z() << "]  " << segment.angle << "  "
       << rate << "\n";
  }

  os << "  说明：相邻两段的转轴一般不平行，而角速度的方向就是各自的转轴，\n"
        "        所以要让角速度在路点处连续，只能两侧都为 0——这就是姿态在\n"
        "        每个路点都必须停住的几何原因。\n";
}

void printCartesianResult(const CartesianResult & result, std::ostream & os)
{
  if (result.success) {
    os << "[CartesianResult] success（轨迹可行）\n";
    return;
  }
  os << "[CartesianResult] FAILED: " << result.message << "\n";
}

}  // namespace kdl_interpolation
