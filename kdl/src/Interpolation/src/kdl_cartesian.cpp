// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：kdl_cartesian.hpp 中声明的函数在这里落地。
//
// 文件顺序与头文件一一对应：
//   一、单段：系数与求值   —— 位置三次标量插值 + 姿态一次 slerp
//   二、轨迹类成员函数     —— 采样与校验
//   三、构建入口           —— 输入校验 → 逐段组装 → 立即校验
//
// 本文件刻意写得很"薄"，原因有二：
//   1) 位置的数学完全复用 kdl_quintic，这里只负责把 x/y/z 当成三个独立标量；
//   2) 姿态的数学只有两步——解出轴角、绕固定轴转 θ·s(τ)，没有别的。

#include "kdl_cartesian.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <kdl/jntarray.hpp>
#include <rclcpp/logging.hpp>

namespace kdl_interpolation
{
namespace
{

/// 所有日志统一前缀，与 kdl_quintic 保持一致，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_interpolation";

/// 限位校验时每段的采样点数（含两端），与 kdl_quintic 的做法一致。
constexpr unsigned int kLimitCheckSamples = 200;

/**
 * @brief 判定"两个姿态几乎相同"的阈值：四元数矢量部分的模长小于它，就认为该段
 *        没有姿态变化。
 *
 * @note 取 1e-8 的理由：小角近似下 θ ≈ 2·|v|，1e-8 对应约 2e-8 rad（≈1e-6 度），
 *       远小于任何真实机构的姿态分辨率；而这个量级又远高于 double 的舍入误差
 *       （~1e-16），不会把噪声误判成"真的在转"。
 */
constexpr double kAxisEps = 1e-8;

/**
 * @brief 由相对旋转 R_rel 解出"最短弧"的转轴与转角。
 * @param relative [in]  相对旋转 R0⁻¹·R1。
 * @param axis     [out] 单位转轴，表达在 R0 坐标系中；无姿态变化时为零向量。
 * @param angle    [out] 转角 θ ∈ [0, π]，单位 rad。
 *
 * @note 三个步骤正好对应头文件第六节列出的两个奇异点：
 *       1) 归一化四元数，并把 w < 0 的那一支整体取反（q 与 −q 表示同一姿态），
 *          从而保证拿到的是最短弧、θ 落在 [0, π]；
 *       2) |v| < kAxisEps → 判定"无姿态变化"，轴取零向量，θ 取 0；
 *       3) 否则 θ = 2·atan2(|v|, w)、n̂ = v/|v|。用 atan2 而不是 acos，是因为
 *          θ → 0 时 acos 的导数趋于无穷，会把四元数的舍入误差放大成明显误差。
 *       注意 θ → π 时 w → 0 而 |v| → 1，公式依然稳定，所以 π 不是危险点。
 */
void relativeAxisAngle(const KDL::Rotation & relative, KDL::Vector & axis, double & angle)
{
  double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
  relative.GetQuaternion(x, y, z, w);

  const double norm = std::sqrt(x * x + y * y + z * z + w * w);
  if (norm <= 0.0) {
    // 理论上不会发生（GetQuaternion 给的是单位四元数），但零向量除不得，
    // 这里退化成"无旋转"比返回 NaN 安全得多。
    axis = KDL::Vector(0.0, 0.0, 0.0);
    angle = 0.0;
    return;
  }
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;

  // 最短弧：q 与 −q 是同一个姿态，统一取 w ≥ 0 的那一支。
  if (w < 0.0) {
    x = -x;
    y = -y;
    z = -z;
    w = -w;
  }

  const double vector_norm = std::sqrt(x * x + y * y + z * z);
  if (vector_norm < kAxisEps) {
    // 情形 1：θ → 0，转轴无定义 → 约定为"该段不转"。
    axis = KDL::Vector(0.0, 0.0, 0.0);
    angle = 0.0;
    return;
  }

  angle = 2.0 * std::atan2(vector_norm, w);
  axis = KDL::Vector(x / vector_norm, y / vector_norm, z / vector_norm);
}

/**
 * @brief 组装一段 slerp 数据。
 * @param start    [in]  起点姿态 R0。
 * @param goal     [in]  终点姿态 R1。
 * @param duration [in]  段时长 T，单位 s，须 > 0。
 * @param segment  [out] 结果。
 *
 * @note 关键的一步是把转轴从 R0 坐标系（相对旋转 R_rel 自己的转轴 n̂）搬到基座
 *       坐标系。靠的是旋转矩阵的相似变换性质 S·Rot(a,φ)·Sᵀ = Rot(S·a, φ)：
 *
 *           R0·Rot(n̂, θs)  =  Rot(R0·n̂, θs)·R0
 *
 *       也就是说"绕着 R0 坐标系里的 n̂ 转"与"绕着基座系里的 R0·n̂ 转、再接上 R0"
 *       是同一件事。存基座系里的那根轴（axis_base = R0·n̂）有两个好处：
 *       1) 角速度方向恒定：ω = (θ/T)·s'(τ)·axis_base 不必再乘 R(τ)；
 *       2) 多段拼接时"轴不同则无法连续"这个结论一目了然。
 */
void makeSlerpSegment(
  const KDL::Rotation & start, const KDL::Rotation & goal, double duration, SlerpSegment & segment)
{
  KDL::Vector axis_local;
  double angle = 0.0;
  relativeAxisAngle(start.Inverse() * goal, axis_local, angle);

  segment.start_rotation = start;
  segment.duration = duration;
  segment.angle = angle;
  segment.axis_base = start * axis_local;  // 零向量转过去仍是零向量
}

}  // namespace

// ---------------------------------------------------------------------------
// 一、单段：系数与求值
// ---------------------------------------------------------------------------

void quinticSmoothStep(double tau, double & s, double & ds, double & dds)
{
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;

  s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
  ds = 30.0 * tau2 - 60.0 * tau3 + 30.0 * tau4;  // 因式分解：30τ²(1−τ)²
  dds = 60.0 * tau - 180.0 * tau2 + 120.0 * tau3;
}

bool computeCartesianSegment(
  const KDL::Frame & start, const KDL::Frame & goal, double duration,
  std::array<QuinticCoefficients, 3> & position, SlerpSegment & orientation)
{
  if (duration <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "computeCartesianSegment: 段时长必须 > 0（当前 %.6f s）",
      duration);
    return false;
  }

  // ---- 位置：把 x/y/z 当成三个互不耦合的标量，各求一次五次多项式 ----
  // 两端 v = a = 0，所以每段的形状就是 p(τ) = p0 + (p1 − p0)·s(τ)，位置走直线。
  const double p0[3] = {start.p.x(), start.p.y(), start.p.z()};
  const double p1[3] = {goal.p.x(), goal.p.y(), goal.p.z()};
  for (unsigned int i = 0; i < 3; ++i) {
    computeQuinticCoefficients(p0[i], 0.0, 0.0, p1[i], 0.0, 0.0, duration, position[i]);
  }

  // ---- 姿态：解出轴角，存成"基座系下的固定轴 + 转角" ----
  makeSlerpSegment(start.M, goal.M, duration, orientation);
  return true;
}

bool evaluateCartesianSegment(
  const std::array<QuinticCoefficients, 3> & position, const SlerpSegment & orientation, double tau,
  CartesianState & state)
{
  if (orientation.duration <= 0.0) {
    return false;
  }

  // ---- 位置：三个分量各自求值（evaluateQuinticSegment 内部已做 /T 的换算）----
  double x = 0.0, vx = 0.0, ax = 0.0;
  double y = 0.0, vy = 0.0, ay = 0.0;
  double z = 0.0, vz = 0.0, az = 0.0;
  evaluateQuinticSegment(position[0], tau, x, vx, ax);
  evaluateQuinticSegment(position[1], tau, y, vy, ay);
  evaluateQuinticSegment(position[2], tau, z, vz, az);

  state.pose.p = KDL::Vector(x, y, z);
  state.linear_velocity = KDL::Vector(vx, vy, vz);
  state.linear_acceleration = KDL::Vector(ax, ay, az);

  // ---- 姿态：先算角参数 s(τ)，再绕固定轴转 θ·s(τ) ----
  double s = 0.0, ds = 0.0, dds = 0.0;
  quinticSmoothStep(tau, s, ds, dds);

  // ⚠ 乘法次序：转轴存在**基座系**里，所以是左乘——"先到起点姿态，再在基座系里
  // 绕固定轴转过去"，即 R(τ) = Rot(axis_base, θ·s(τ)) · R0。写成右乘（R0 · Rot(...)）
  // 的含义变成"绕 R0 自身坐标系里的轴转"，那段轴是另一个人（axis_local），
  // 结果会在终点处对不上目标姿态。两者的等价关系见 makeSlerpSegment() 的注释。
  // Rot() 内部会自行归一化转轴；轴为零向量且角度为 0 时返回单位旋转，
  // 所以"无姿态变化"的段不需要任何特判。
  state.pose.M =
    KDL::Rotation::Rot(orientation.axis_base, orientation.angle * s) * orientation.start_rotation;

  // 角速度、角加速度都沿着同一根固定轴，只是标量倍数不同：
  //   |ω| = θ·s'(τ)/T 、|α| = θ·s''(τ)/T²。
  const double inv_T = 1.0 / orientation.duration;
  state.angular_velocity = orientation.axis_base * (orientation.angle * ds * inv_T);
  state.angular_acceleration = orientation.axis_base * (orientation.angle * dds * inv_T * inv_T);
  return true;
}

// ---------------------------------------------------------------------------
// 二、轨迹类成员函数
// ---------------------------------------------------------------------------

double CartesianTrajectory::segmentDuration(unsigned int index) const
{
  if (index >= durations_.size()) {
    return 0.0;
  }
  return durations_[index];
}

const std::array<QuinticCoefficients, 3> & CartesianTrajectory::positionCoefficients(
  unsigned int segment) const
{
  // 越界时返回一个静态的空系数，避免调用者拿到悬垂引用（与 kdl_quintic 同款）。
  static const std::array<QuinticCoefficients, 3> kEmpty{};
  if (segment >= position_.size()) {
    return kEmpty;
  }
  return position_[segment];
}

const SlerpSegment & CartesianTrajectory::orientationSegment(unsigned int segment) const
{
  static const SlerpSegment kEmpty;
  if (segment >= orientation_.size()) {
    return kEmpty;
  }
  return orientation_[segment];
}

bool CartesianTrajectory::locateSegment(double t, unsigned int & index, double & tau) const
{
  if (!valid()) {
    return false;
  }

  // 越界一律 clamp 到端点：控制器偶尔会多算半个周期，这不该成为一次错误。
  const double total = duration();
  const double clamped = std::min(std::max(t, 0.0), total);

  // 逐段线性查找。段数在现实里只有个位数，二分反而更难读。
  unsigned int segment = 0;
  while (segment + 1 < segmentCount() && clamped > knot_times_[segment + 1]) {
    ++segment;
  }

  index = segment;
  tau = (clamped - knot_times_[segment]) / durations_[segment];
  return true;
}

bool CartesianTrajectory::sample(double t, CartesianState & state) const
{
  unsigned int index = 0;
  double tau = 0.0;
  if (!locateSegment(t, index, tau)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "sample: 轨迹尚未成功构建，无法采样");
    return false;
  }
  return evaluateCartesianSegment(position_[index], orientation_[index], tau, state);
}

bool CartesianTrajectory::validateLimits(
  const CartesianLimits & limits, std::string & message) const
{
  message.clear();
  if (!valid()) {
    message = "轨迹为空，无法校验";
    return false;
  }

  const bool check_linear_velocity = limits.check_linear_velocity();
  const bool check_linear_acceleration = limits.check_linear_acceleration();
  const bool check_angular_velocity = limits.check_angular_velocity();
  const bool check_angular_acceleration = limits.check_angular_acceleration();
  if (!check_linear_velocity && !check_linear_acceleration && !check_angular_velocity &&
      !check_angular_acceleration) {
    return true;  // 一项都不用查，直接放行（默认构造就是这个情形）
  }

  for (unsigned int k = 0; k < segmentCount(); ++k) {
    // 先在本段内采样取四个峰值，再统一与上限比较：
    // 这样报出来的是"这一段真正的峰值"，而不是"第一个撞线的采样点"。
    double linear_velocity_peak = 0.0;
    double linear_acceleration_peak = 0.0;
    double angular_velocity_peak = 0.0;
    double angular_acceleration_peak = 0.0;

    for (unsigned int i = 0; i <= kLimitCheckSamples; ++i) {
      const double tau = static_cast<double>(i) / static_cast<double>(kLimitCheckSamples);
      CartesianState state;
      evaluateCartesianSegment(position_[k], orientation_[k], tau, state);

      linear_velocity_peak = std::max(linear_velocity_peak, state.linear_velocity.Norm());
      linear_acceleration_peak =
        std::max(linear_acceleration_peak, state.linear_acceleration.Norm());
      angular_velocity_peak = std::max(angular_velocity_peak, state.angular_velocity.Norm());
      angular_acceleration_peak =
        std::max(angular_acceleration_peak, state.angular_acceleration.Norm());
    }

    const auto exceeded = [&](double peak, double bound, const char * name, const char * unit) {
      if (bound <= 0.0 || peak <= bound) {
        return false;
      }
      message = "第 " + std::to_string(k) + " 段的" + name + "峰值 " + std::to_string(peak) + " " +
                unit + " 超过上限 " + std::to_string(bound) + " " + unit + "（采样近似值）";
      return true;
    };

    if (check_linear_velocity &&
        exceeded(linear_velocity_peak, limits.max_linear_velocity, "线速度", "m/s")) {
      return false;
    }
    if (check_linear_acceleration &&
        exceeded(linear_acceleration_peak, limits.max_linear_acceleration, "线加速度", "m/s²")) {
      return false;
    }
    if (check_angular_velocity &&
        exceeded(angular_velocity_peak, limits.max_angular_velocity, "角速度", "rad/s")) {
      return false;
    }
    if (check_angular_acceleration &&
        exceeded(angular_acceleration_peak, limits.max_angular_acceleration, "角加速度", "rad/s²")) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 三、构建入口
// ---------------------------------------------------------------------------

CartesianResult buildCartesianTrajectory(
  const std::vector<KDL::Frame> & waypoints, const std::vector<double> & durations,
  WaypointBehavior waypoint_behavior, const CartesianLimits & limits)
{
  CartesianResult result;

  // ---- 1. 输入校验：全部集中在前面，后面就可以假定数据是规整的 ----
  const unsigned int segments = static_cast<unsigned int>(durations.size());
  if (segments == 0) {
    result.message = "至少需要 1 段轨迹（即 2 个路点）";
    return result;
  }
  if (waypoints.size() != durations.size() + 1) {
    result.message = "路点数(" + std::to_string(waypoints.size()) + ")必须等于段数 + 1(" +
                     std::to_string(durations.size() + 1) + ")";
    return result;
  }
  for (std::size_t k = 0; k < durations.size(); ++k) {
    if (durations[k] <= 0.0) {
      result.message = "第 " + std::to_string(k) + " 段时长必须 > 0（当前 " +
                       std::to_string(durations[k]) + " s）";
      return result;
    }
  }

  // ---- 2. 落盘路点、时长、路点时刻 ----
  CartesianTrajectory & trajectory = result.trajectory;
  trajectory.waypoint_behavior_ = waypoint_behavior;
  trajectory.waypoints_ = waypoints;
  trajectory.durations_ = durations;
  trajectory.knot_times_.assign(durations.size() + 1, 0.0);
  for (unsigned int k = 0; k < segments; ++k) {
    trajectory.knot_times_[k + 1] = trajectory.knot_times_[k] + durations[k];
  }
  trajectory.position_.resize(segments);
  trajectory.orientation_.resize(segments);

  // ---- 3. 位置：两种模式分别怎么算 ----
  if (waypoint_behavior == WaypointBehavior::kStop) {
    // kStop：每段独立、两端 v = a = 0，段与段之间没有任何耦合。
    // 因此不需要（也不能）列全局方程——起点终点都是静止的，本来就没有
    // "要传下去"的速度可解。
    for (unsigned int k = 0; k < segments; ++k) {
      const KDL::Frame & start = waypoints[k];
      const KDL::Frame & goal = waypoints[k + 1];
      const double p0[3] = {start.p.x(), start.p.y(), start.p.z()};
      const double p1[3] = {goal.p.x(), goal.p.y(), goal.p.z()};
      for (unsigned int i = 0; i < 3; ++i) {
        computeQuinticCoefficients(
          p0[i], 0.0, 0.0, p1[i], 0.0, 0.0, durations[k], trajectory.position_[k][i]);
      }
    }
  } else {
    // kPassThrough：位置转调 kdl_quintic 的多段 C⁴ 求解器。
    // 做法是把 x/y/z 看成一条"3 自由度的链"——位置本来就没有关节概念，
    // 用 KDL::JntArray 只是为了复用现成的接口，与机器人无关。
    // 这里刻意**不传** JointLimits：位置限位属于笛卡尔限位（CartesianLimits）
    // 的范畴，留给本模块第 4 步统一判定，避免同一个超限被报两次。
    std::vector<KDL::JntArray> position_waypoints;
    position_waypoints.reserve(waypoints.size());
    for (const KDL::Frame & frame : waypoints) {
      KDL::JntArray p(3);
      p(0) = frame.p.x();
      p(1) = frame.p.y();
      p(2) = frame.p.z();
      position_waypoints.push_back(p);
    }

    const TrajectoryResult spline = buildQuinticTrajectory(position_waypoints, durations);
    if (!spline.success) {
      result.message = "位置的多段 C⁴ 拼接失败：" + spline.message;
      return result;
    }
    for (unsigned int k = 0; k < segments; ++k) {
      for (unsigned int i = 0; i < 3; ++i) {
        trajectory.position_[k][i] = spline.trajectory.segmentCoefficients(k, i);
      }
    }
  }

  // ---- 4. 姿态：每段一次 slerp 分解（两种模式完全相同）----
  // 注意这里**不能**调 computeCartesianSegment() 顺手把位置也算一遍，
  // 否则 kPassThrough 模式下刚拼好的系数就被覆盖了。
  for (unsigned int k = 0; k < segments; ++k) {
    makeSlerpSegment(
      waypoints[k].M, waypoints[k + 1].M, durations[k], trajectory.orientation_[k]);
  }

  // ---- 5. 立即校验一次，所以 success == true 就意味着"这条轨迹可行" ----
  std::string message;
  if (!trajectory.validateLimits(limits, message)) {
    result.message = message;
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace kdl_interpolation
