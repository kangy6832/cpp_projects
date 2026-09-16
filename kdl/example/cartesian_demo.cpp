// Copyright (c) 2026, kdl_interpolation authors.
// 教学示例：笛卡尔空间位姿插值 —— 位置五次多项式 + 姿态 slerp。
//
// 运行方式（先 colcon build，再 source install/setup.bash）：
//   ros2 run kdl_tools cartesian_demo
//
// 演示顺序：
//   1) 单段：起点位姿 → 目标位姿，看 x/y/z 的六个系数与 slerp 的 (轴, 角)，再采样
//   2) 姿态角参数为什么必须用五次 s(τ)、不能用线性的 τ（解析对比角速度剖面）
//   3) 姿态的解析峰值 15θ/(8T) 与 10θ/(√3 T²)，和采样峰值互相对照
//   4) slerp 的两个奇异点：Δθ ≈ 0 与 Δθ = π，确认都不出 NaN
//   5) 多段：kStop 与 kPassThrough 对比 —— 姿态在路点都停，位置则完全不同
//   6) 限位校验：同一批路点，时间压紧就判"不可行"
//   7) 导出 CSV，交给 plot_cartesian 画图
//
// 四个关键教学点：
//   a) 位置是三个标量，可以逐分量用五次多项式（走直线）；姿态不是标量，
//      逐元素插值出来的矩阵不再是旋转，只能用 slerp。
//   b) 每一段 slerp 都是"绕该段自己的固定轴转"，角速度方向就是那根轴。
//      相邻两段的轴一般不平行，所以多段时**内部路点的角速度只能是 0**。
//   c) 用五次时间标度 s(τ) 后，s'(0)=s'(1)=s''(0)=s''(1)=0 自动成立，
//      上面那条"必须为 0"不需要再解任何方程，位置与姿态也就同起同停。
//   d) 代价是峰值角速度被抬到 1.875·θ/T（匀速转只要 θ/T），这就是"平滑"
//      要付的账；面板 2 与 3 会把这笔账算清楚。
//
// 本示例不依赖 URDF / 机器人模型：笛卡尔插值是末端位姿层面的事，
// 给几个 KDL::Frame 就能跑（至于这些位姿关节能不能达到，那是 IK 的问题）。

#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <kdl/frames.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kdl_cartesian.hpp"
#include "kdl_cartesian_print.hpp"
#include "kdl_quintic.hpp"

namespace
{

/// 圆周率（不依赖 M_PI，它不是标准 C++ 的一部分）。
constexpr double kPi = 3.14159265358979323846;

/// CSV 采样点数（不含首尾则为 kCsvSteps，实际写 kCsvSteps + 1 行），与 quintic_demo 一致。
constexpr unsigned int kCsvSteps = 400;

/**
 * @brief 由位置与姿态拼一个位姿。
 * @param p 位置 x/y/z，单位 m。
 * @param R 姿态。
 * @return 位姿 T = (R, p)。
 */
KDL::Frame makeFrame(const std::array<double, 3> & p, const KDL::Rotation & R)
{
  return KDL::Frame(R, KDL::Vector(p[0], p[1], p[2]));
}

/**
 * @brief 保持位置不变、只换姿态。
 * @param base 参考位姿（只取它的位置 p）。
 * @param R    新姿态。
 * @return 位姿 T = (R, base.p)，单位 m。
 *
 * @note 奇异点那一节专门用它：转轴随便怎么转，末端都待在同一点上，
 *       这样观察到的现象就只与姿态有关，不会混进位置的变化。
 */
KDL::Frame withOrientation(const KDL::Frame & base, const KDL::Rotation & R)
{
  return KDL::Frame(R, base.p);
}

/**
 * @brief 两个姿态之间的"夹角"（等效旋转角）。
 * @param a 姿态 a。
 * @param b 姿态 b。
 * @return a⁻¹·b 的等效转角，单位 rad，∈ [0, π]。
 *
 * @note 直接借 KDL 的 GetRotAngle()：它按定义返回 [0, π] 内那个角，所以这里
 *       量出来的就是"最短弧"意义下的姿态差——正是 slerp 该走的那段弧。
 *       用它做校验，等于在检查"姿态是否真的落在两点之间的大圆上"。
 */
double rotationGap(const KDL::Rotation & a, const KDL::Rotation & b)
{
  KDL::Vector axis;
  return std::abs((a.Inverse() * b).GetRotAngle(axis, 1e-8));
}

/**
 * @brief 打印一个姿态的 RPY，单位 rad。
 * @param R 姿态。
 */
void printRpy(const KDL::Rotation & R)
{
  double roll = 0.0, pitch = 0.0, yaw = 0.0;
  R.GetRPY(roll, pitch, yaw);
  std::cout << "[" << roll << ", " << pitch << ", " << yaw << "]";
}

/**
 * @brief 把一条笛卡尔轨迹按固定步长采样写成 CSV。
 * @param trajectory [in] 待导出的轨迹，须 valid()。
 * @param path       [in] 输出文件路径。
 * @return true 表示写入成功。
 *
 * @note 姿态同时写四元数与 RPY 两套：RPY 好读（画图脚本用它），四元数
 *       好算（需要复核姿态的人用它），两者互相印证。
 * @note 列名约定被 src/Interpolation/scripts/plot_cartesian.py 依赖，改动要同步。
 */
bool writeCsv(const kdl_interpolation::CartesianTrajectory & trajectory, const std::string & path)
{
  std::ofstream csv(path);
  if (!csv) {
    return false;
  }

  static const char * kColumns[] = {
    "x",   "y",   "z",     "qx",  "qy",  "qz",  "qw",  "roll", "pitch", "yaw", "vx",
    "vy",  "vz",  "wx",    "wy",  "wz",  "ax",  "ay",  "az",   "alphax", "alphay", "alphaz"};

  csv << "t";
  for (const char * name : kColumns) {
    csv << "," << name;
  }
  csv << "\n";

  kdl_interpolation::CartesianState state;
  for (unsigned int i = 0; i <= kCsvSteps; ++i) {
    const double t =
      trajectory.duration() * static_cast<double>(i) / static_cast<double>(kCsvSteps);
    trajectory.sample(t, state);

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    state.pose.M.GetRPY(roll, pitch, yaw);
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
    state.pose.M.GetQuaternion(qx, qy, qz, qw);

    csv << t << "," << state.pose.p.x() << "," << state.pose.p.y() << "," << state.pose.p.z() << ","
        << qx << "," << qy << "," << qz << "," << qw << "," << roll << "," << pitch << "," << yaw
        << "," << state.linear_velocity.x() << "," << state.linear_velocity.y() << ","
        << state.linear_velocity.z() << "," << state.angular_velocity.x() << ","
        << state.angular_velocity.y() << "," << state.angular_velocity.z() << ","
        << state.linear_acceleration.x() << "," << state.linear_acceleration.y() << ","
        << state.linear_acceleration.z() << "," << state.angular_acceleration.x() << ","
        << state.angular_acceleration.y() << "," << state.angular_acceleration.z() << "\n";
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("cartesian_demo");
  std::cout << std::setprecision(6);

  // =========================================================================
  // 1) 单段：位置五次多项式 + 姿态 slerp
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 1) 单段：位置五次多项式 + 姿态 slerp\n"
            << "=============================================================\n";

  const KDL::Frame start = makeFrame({0.40, -0.20, 0.30}, KDL::Rotation::RPY(0.10, -0.20, 0.30));
  const KDL::Frame goal = makeFrame({0.60, 0.30, 0.50}, KDL::Rotation::RPY(0.50, 0.20, -0.60));
  const double single_duration = 1.0;  // s

  std::cout << "  起点位姿:\n";
  kdl_interpolation::printCartesianPose(start);
  std::cout << "  目标位姿:\n";
  kdl_interpolation::printCartesianPose(goal);

  // 六个边界条件全部由这两个位姿与"两端静止"约定决定，不需要调用者再填。
  std::array<kdl_interpolation::QuinticCoefficients, 3> position;
  kdl_interpolation::SlerpSegment orientation;
  kdl_interpolation::computeCartesianSegment(start, goal, single_duration, position, orientation);

  std::cout << "\n  单段数据（x/y/z 的六个系数 + 姿态的 (轴, 角)）:\n";
  kdl_interpolation::printCartesianSegment(position, orientation);

  std::cout << "\n  采样表（位置 m、姿态 RPY rad、|omega| rad/s）\n";
  std::cout << "      t[s]         x         y         z |      roll     pitch       yaw |"
               "      |v|   |omega|\n";
  for (unsigned int i = 0; i <= 10; ++i) {
    const double tau = static_cast<double>(i) / 10.0;
    const double t = tau * single_duration;

    kdl_interpolation::CartesianState state;
    kdl_interpolation::evaluateCartesianSegment(position, orientation, tau, state);

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    state.pose.M.GetRPY(roll, pitch, yaw);

    std::cout << std::fixed << std::setprecision(4) << std::setw(10) << t << std::setw(10)
              << state.pose.p.x() << std::setw(10) << state.pose.p.y() << std::setw(10)
              << state.pose.p.z() << " |" << std::setw(10) << roll << std::setw(10) << pitch
              << std::setw(10) << yaw << " |" << std::setw(10) << state.linear_velocity.Norm()
              << std::setw(10) << state.angular_velocity.Norm() << "\n"
              << std::defaultfloat << std::setprecision(6);
  }

  // 边界条件校验：把 τ = 0 与 τ = 1 代回去，看六个条件是否被精确满足。
  kdl_interpolation::CartesianState at_start, at_goal;
  kdl_interpolation::evaluateCartesianSegment(position, orientation, 0.0, at_start);
  kdl_interpolation::evaluateCartesianSegment(position, orientation, 1.0, at_goal);

  std::cout << "\n  边界条件校验（应≈0）:\n"
            << "    起点位置误差 |p − p_start| = " << (at_start.pose.p - start.p).Norm() << " m\n"
            << "    终点位置误差 |p − p_goal | = " << (at_goal.pose.p - goal.p).Norm() << " m\n"
            << "    起点姿态误差 = " << rotationGap(at_start.pose.M, start.M) << " rad\n"
            << "    终点姿态误差 = " << rotationGap(at_goal.pose.M, goal.M) << " rad\n"
            << "    起点/终点 |v|     = " << at_start.linear_velocity.Norm() << " / "
            << at_goal.linear_velocity.Norm() << " m/s\n"
            << "    起点/终点 |omega| = " << at_start.angular_velocity.Norm() << " / "
            << at_goal.angular_velocity.Norm() << " rad/s\n"
            << "  说明：位置由五次多项式保证两端速度为零；姿态由 s'(0)=s'(1)=0 保证\n"
            << "        两端角速度为零。两条曲线在同一起点停住、同一点停下。\n";

  // =========================================================================
  // 2) 角参数 s(τ)：为什么必须是五次，而不是线性的 τ
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 2) 角参数 s(τ) 为什么必须是五次（与线性 τ 的解析对比）\n"
            << "=============================================================\n";

  const double theta = orientation.angle;  // 本段转角，单位 rad
  const double duration = single_duration;  // 本段时长，单位 s

  std::cout << "  本段转角 theta = " << theta << " rad (" << theta * 180.0 / kPi << "°), T = "
            << duration << " s\n";
  std::cout << "  下表最后一列是 |omega| 除以 (theta/T)，即「相对匀速转」的倍数：\n\n";
  std::cout << "      tau    s(tau)   s'(tau)   |omega|/(theta/T) 五次 |  |omega|/(theta/T) 线性\n";

  for (unsigned int i = 0; i <= 4; ++i) {
    const double tau = static_cast<double>(i) / 4.0;
    double s = 0.0, ds = 0.0, dds = 0.0;
    kdl_interpolation::quinticSmoothStep(tau, s, ds, dds);

    // 线性角参数 s(τ) = τ 时 s' ≡ 1，角速度恒为 θ/T，比值自然是 1。
    std::cout << std::fixed << std::setprecision(4) << std::setw(9) << tau << std::setw(10) << s
              << std::setw(10) << ds << std::setw(24) << ds << std::setw(24) << 1.0 << "\n"
              << std::defaultfloat << std::setprecision(6);
  }

  std::cout << "\n  结论：线性角参数（最后一列）全程恒为 theta/T，包括 t = 0 与 t = T——\n"
            << "        也就是说姿态是「突跳」着从 0 变成 theta/T 起步、又「突跳」着停下的，\n"
            << "        角加速度在两端等于冲击；而五次 s(τ) 两端 s' = 0，与位置同步\n"
            << "        从 0 长起来、再回到 0。代价看下一节：峰值被抬高到 1.875 倍。\n";

  // =========================================================================
  // 3) 解析峰值 vs 采样峰值
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 3) 姿态/位置的解析峰值 vs 采样峰值\n"
            << "=============================================================\n";

  // 用多段入口构造一条只有一段的轨迹，正好也演示"单段是多段的特例"。
  const kdl_interpolation::CartesianResult single =
    kdl_interpolation::buildCartesianTrajectory({start, goal}, {single_duration});
  kdl_interpolation::printCartesianResult(single);
  if (!single.success) {
    RCLCPP_ERROR(node->get_logger(), "示例无法继续：%s", single.message.c_str());
    rclcpp::shutdown();
    return 1;
  }

  double linear_velocity_peak = 0.0;
  double linear_acceleration_peak = 0.0;
  double angular_velocity_peak = 0.0;
  double angular_acceleration_peak = 0.0;
  const unsigned int samples = 200;
  for (unsigned int i = 0; i <= samples; ++i) {
    const double t = single_duration * static_cast<double>(i) / static_cast<double>(samples);
    kdl_interpolation::CartesianState state;
    single.trajectory.sample(t, state);
    linear_velocity_peak = std::max(linear_velocity_peak, state.linear_velocity.Norm());
    linear_acceleration_peak = std::max(linear_acceleration_peak, state.linear_acceleration.Norm());
    angular_velocity_peak = std::max(angular_velocity_peak, state.angular_velocity.Norm());
    angular_acceleration_peak =
      std::max(angular_acceleration_peak, state.angular_acceleration.Norm());
  }

  const double displacement = (goal.p - start.p).Norm();  // Δp，单位 m
  const double linear_velocity_analytic = 15.0 * displacement / (8.0 * duration);
  const double linear_acceleration_analytic =
    10.0 * displacement / (std::sqrt(3.0) * duration * duration);
  const double angular_velocity_analytic = 15.0 * theta / (8.0 * duration);
  const double angular_acceleration_analytic =
    10.0 * theta / (std::sqrt(3.0) * duration * duration);

  std::cout << "  位移 Δp = " << displacement << " m，转角 theta = " << theta << " rad\n\n";
  std::cout << "  量            解析峰值            采样峰值(201 点)      相对误差\n";
  const auto print_peak_row = [](const char * name, double analytic, double sampled) {
    const double error = analytic > 0.0 ? std::abs(sampled - analytic) / analytic : 0.0;
    std::cout << std::fixed << std::setprecision(6) << "  " << std::setw(12) << name
              << std::setw(20) << analytic << std::setw(22) << sampled << std::setw(16) << error
              << "\n"
              << std::defaultfloat << std::setprecision(6);
  };
  print_peak_row("|v| [m/s]", linear_velocity_analytic, linear_velocity_peak);
  print_peak_row("|a| [m/s²]", linear_acceleration_analytic, linear_acceleration_peak);
  print_peak_row("|omega| [rad/s]", angular_velocity_analytic, angular_velocity_peak);
  print_peak_row("|alpha| [rad/s²]", angular_acceleration_analytic, angular_acceleration_peak);

  std::cout << "\n  解析式的来历（两端 v = a = 0 的单段，三个分量共用同一个 s(τ)）：\n"
            << "    p(τ) = p0 + Δp·s(τ)  ⇒  |v| = Δp·|s'|/T ≤ Δp·1.875/T  = 15Δp/(8T)\n"
            << "    姿态同理：      |omega| = theta·|s'|/T ≤ 15theta/(8T)\n"
            << "    两个二阶导的极值都在 τ = 1/2 ∓ √3/6 处取到 10/(√3)：\n"
            << "    |a| = Δp·|s''|/T² ≤ 10Δp/(√3 T²)，|alpha| ≤ 10theta/(√3 T²)\n"
            << "  采样峰值与解析峰值一致，说明轨迹本身没问题；反过来，解析式让\n"
            << "  「给定时间和位移，峰值到底是多少」在写代码之前就能算出来。\n";

  // =========================================================================
  // 4) slerp 的两个奇异点
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 4) slerp 的两个奇异点：Δtheta ≈ 0 与 Δtheta = π\n"
            << "=============================================================\n";
  std::cout << "  判定「无姿态变化」的阈值 kAxisEps = 1e-8（四元数矢量部分模长）\n\n";

  // ---- 4a. Δθ ≈ 1e-12 rad：低于阈值，应判为"不转"，且不能出 NaN ----
  const KDL::Frame tiny_goal = withOrientation(start, start.M * KDL::Rotation::RotZ(1e-12));
  std::array<kdl_interpolation::QuinticCoefficients, 3> tiny_position;
  kdl_interpolation::SlerpSegment tiny_orientation;
  kdl_interpolation::computeCartesianSegment(
    start, tiny_goal, single_duration, tiny_position, tiny_orientation);

  double tiny_pose_gap = 0.0;
  double tiny_omega = 0.0;
  bool tiny_finite = true;
  for (unsigned int i = 0; i <= 4; ++i) {
    const double tau = static_cast<double>(i) / 4.0;
    kdl_interpolation::CartesianState state;
    kdl_interpolation::evaluateCartesianSegment(tiny_position, tiny_orientation, tau, state);
    tiny_pose_gap = std::max(tiny_pose_gap, rotationGap(state.pose.M, start.M));
    tiny_omega = std::max(tiny_omega, state.angular_velocity.Norm());
    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
    state.pose.M.GetQuaternion(qx, qy, qz, qw);
    tiny_finite = tiny_finite && std::isfinite(qx) && std::isfinite(qy) && std::isfinite(qz) &&
                  std::isfinite(qw);
  }
  std::cout << "    4a) Δtheta = 1e-12 rad：解出 theta = " << tiny_orientation.angle
            << " rad，|axis| = " << tiny_orientation.axis_base.Norm()
            << "（应为 0，即判定为「无姿态变化」）\n";
  std::cout << "        采样 5 点：姿态与起点的最大夹角 = " << tiny_pose_gap
            << " rad，|omega|max = " << tiny_omega << "，四元数有限 = " << std::boolalpha
            << tiny_finite << std::noboolalpha << "（都应满足）\n";

  // ---- 4b. Δθ = π：w ≈ 0，但 |v| ≈ 1，公式依然稳定 ----
  const KDL::Frame half_turn_goal = withOrientation(start, start.M * KDL::Rotation::RotX(kPi));
  kdl_interpolation::SlerpSegment pi_orientation;
  kdl_interpolation::computeCartesianSegment(
    start, half_turn_goal, single_duration, tiny_position, pi_orientation);

  kdl_interpolation::CartesianState at_half, at_end;
  kdl_interpolation::evaluateCartesianSegment(tiny_position, pi_orientation, 0.5, at_half);
  kdl_interpolation::evaluateCartesianSegment(tiny_position, pi_orientation, 1.0, at_end);
  // 理论值：绕基座系的固定轴转过半程，左乘起点姿态（与模块内部的写法一致）。
  const KDL::Rotation expected_half =
    KDL::Rotation::Rot(pi_orientation.axis_base, pi_orientation.angle * 0.5) * start.M;

  std::cout << "\n    4b) Δtheta = π rad（绕 R0 的 X 轴转 180°）：\n"
            << "        解出 theta = " << pi_orientation.angle << " rad，转轴(基座系) = ["
            << pi_orientation.axis_base.x() << ", " << pi_orientation.axis_base.y() << ", "
            << pi_orientation.axis_base.z() << "]\n"
            << "        τ = 0.5 处与理论值 Rot(axis, theta/2) 的夹角 = "
            << rotationGap(at_half.pose.M, expected_half) << " rad（应为 0）\n"
            << "        τ = 1.0 处与目标姿态的夹角 = " << rotationGap(at_end.pose.M, half_turn_goal.M)
            << " rad（应为 0）\n"
            << "        τ = 0.5 处 |omega| = " << at_half.angular_velocity.Norm()
            << " rad/s（= 1.875·theta/T，是全程峰值）\n";
  std::cout << "  说明：θ→0 时轴没有定义，必须显式处理（否则要给 0 除 0）；\n"
            << "        θ→π 时 w→0 而 |v|→1，公式反而稳定，不唯一的只是轴的正负号，\n"
            << "        而 ±axis 描述的是同一个 180° 旋转，对结果毫无影响。\n";

  // =========================================================================
  // 5) 多段：kStop 与 kPassThrough 对比
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 5) 多段：kStop 与 kPassThrough 的差别（姿态两者都停）\n"
            << "=============================================================\n";

  std::vector<KDL::Frame> waypoints;
  waypoints.push_back(makeFrame({0.40, -0.20, 0.30}, KDL::Rotation::RPY(0.10, -0.20, 0.30)));
  waypoints.push_back(makeFrame({0.45, 0.10, 0.55}, KDL::Rotation::RPY(-0.30, 0.10, 0.90)));
  waypoints.push_back(makeFrame({0.30, 0.35, 0.40}, KDL::Rotation::RPY(0.40, 0.50, -0.20)));
  waypoints.push_back(makeFrame({0.20, 0.05, 0.25}, KDL::Rotation::RPY(0.00, -0.10, 0.60)));
  const std::vector<double> base_durations{0.6, 0.5, 0.8};  // s，逐段显式给出

  const kdl_interpolation::CartesianResult stop_result =
    kdl_interpolation::buildCartesianTrajectory(
      waypoints, base_durations, kdl_interpolation::WaypointBehavior::kStop);
  const kdl_interpolation::CartesianResult pass_result =
    kdl_interpolation::buildCartesianTrajectory(
      waypoints, base_durations, kdl_interpolation::WaypointBehavior::kPassThrough);

  std::cout << "  两种模式（无约束）:\n";
  kdl_interpolation::printCartesianResult(stop_result);
  kdl_interpolation::printCartesianResult(pass_result);
  if (!stop_result.success || !pass_result.success) {
    RCLCPP_ERROR(node->get_logger(), "多段构建失败，示例无法继续");
    rclcpp::shutdown();
    return 1;
  }

  kdl_interpolation::printCartesianTrajectorySummary(stop_result.trajectory);
  std::cout << "\n";
  kdl_interpolation::printCartesianTrajectorySummary(pass_result.trajectory);
  std::cout << "\n";
  kdl_interpolation::printCartesianOrientationTable(stop_result.trajectory);

  std::cout << "\n  路点处的速度对比：\n";
  std::cout << "  路点   t[s]    kStop |v| [m/s]   kPassThrough |v| [m/s]   |omega| [rad/s]\n";
  for (unsigned int k = 0; k < stop_result.trajectory.knotTimes().size(); ++k) {
    kdl_interpolation::CartesianState stop_state, pass_state;
    stop_result.trajectory.sample(stop_result.trajectory.knotTimes()[k], stop_state);
    pass_result.trajectory.sample(pass_result.trajectory.knotTimes()[k], pass_state);

    std::cout << std::fixed << std::setprecision(4) << std::setw(6) << k << std::setw(8)
              << stop_result.trajectory.knotTimes()[k] << std::setw(18)
              << stop_state.linear_velocity.Norm() << std::setw(24)
              << pass_state.linear_velocity.Norm() << std::setw(18)
              << stop_state.angular_velocity.Norm() << "\n"
              << std::defaultfloat << std::setprecision(6);
  }
  std::cout << "  说明：kStop 的位置在每个路点都是 0；kPassThrough 的位置路过路点\n"
            << "        不减速（C⁴ 连续，总时长却和 kStop 一样——省的是「不用停」这件事\n"
            << "        带来的平滑与峰值余量，而不是时间，因为时长是调用者给定的）。\n"
            << "        两种模式下 |omega| 都是 0：这不是巧合，而是 slerp 段的转轴\n"
            << "        各不相同带来的必然结果，见上面姿态表里的「转轴」一列。\n";

  // =========================================================================
  // 6) 限位校验：时间压紧就判"不可行"
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 6) 限位校验：不可行就如实报错，不做 clamp\n"
            << "=============================================================\n";

  kdl_interpolation::CartesianLimits limits;
  limits.max_linear_velocity = 0.6;          // m/s
  limits.max_linear_acceleration = 8.0;      // m/s²
  limits.max_angular_velocity = 4.0;         // rad/s
  limits.max_angular_acceleration = 15.0;    // rad/s²
  kdl_interpolation::printCartesianLimits(limits);

  for (double scale : {0.25, 0.5, 1.0, 2.0}) {
    std::vector<double> durations;
    for (double base : base_durations) {
      durations.push_back(base * scale);
    }

    std::cout << "\n  >>> 时间缩放 " << scale << " → 各段时长 [s]: [";
    for (std::size_t k = 0; k < durations.size(); ++k) {
      std::cout << durations[k] << (k + 1 < durations.size() ? ", " : "");
    }
    std::cout << "]\n";

    kdl_interpolation::printCartesianResult(
      kdl_interpolation::buildCartesianTrajectory(
        waypoints, durations, kdl_interpolation::WaypointBehavior::kPassThrough, limits));
  }

  std::cout << "\n  结论：路点不变、只把时间压紧，峰值 |v| ∝ 1/T、|a| ∝ 1/T²、\n"
            << "        |omega| ∝ 1/T、|alpha| ∝ 1/T²，很快就有某一项撞线；\n"
            << "        反过来把时间放大到 2 倍，同一条轨迹又变得可行了——这也是\n"
            << "        离线规划里「整体拉长时间」为什么是最常用的第一条兜底手段。\n"
            << "        工程上正确的做法就是这里的做法：如实返回不可行，让调用者决定\n"
            << "        放慢、改路点还是换方案，绝不在库里偷偷改慢。\n";

  // =========================================================================
  // 7) 导出 CSV
  // =========================================================================
  std::cout << "\n=============================================================\n"
            << " 7) 导出 CSV\n"
            << "=============================================================\n";

  const std::string path = std::filesystem::absolute("cartesian_trajectory.csv").string();
  if (writeCsv(pass_result.trajectory, path)) {
    std::cout << "  已写入: " << path << "\n"
              << "  列含义: t, x/y/z [m], qx/qy/qz/qw, roll/pitch/yaw [rad],\n"
              << "          vx/vy/vz [m/s], wx/wy/wz [rad/s], ax/ay/az [m/s²],\n"
              << "          alphax/alphay/alphaz [rad/s²]\n"
              << "  " << (kCsvSteps + 1) << " 行（t = 0 .. 总时长）等间隔采样。\n"
              << "  画图:   ros2 run kdl_tools plot_cartesian " << path << "\n";
  } else {
    std::cout << "  写入失败: " << path << "\n";
  }

  rclcpp::shutdown();
  return 0;
}
