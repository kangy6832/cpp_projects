// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：kdl_quintic.hpp 中声明的函数在这里落地。
//
// 文件顺序与头文件一一对应：
//   一、单段：系数与求值      —— 纯代数，逐行可对照公式
//   二、轨迹类成员函数        —— 采样与校验
//   三、构建入口              —— 组装线性方程组（本文件唯一"需要动脑"的地方）
//
// 关于线性方程组的规模，可以先安心：未知量是"每个内部路点 × 2"，一个 7 个
// 路点的轨迹也只有 12 个未知量，所以直接用 Eigen 的稠密求解器即可，
// 完全没必要上带状/稀疏求解——那属于"为了省 12 个浮点数而引入一个算法"。

#include "kdl_quintic.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <Eigen/QR>
#include <rclcpp/logging.hpp>

namespace kdl_interpolation
{
namespace
{

/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_interpolation";

/// 速度、加速度校验时每段的采样点数（jerk 走解析路线，不需要采样）。
constexpr unsigned int kLimitCheckSamples = 200;

/**
 * @brief 一段五次多项式在两端处的 jerk / snap 关于 6 个边界量的梯度。
 *
 * @note 六个边界量的顺序固定为 (qL, vL, aL, qR, vR, aR)。
 *       多段拼接时必须知道"改动某个路点的 v 或 a 会怎样影响相邻两段的
 *       jerk / snap"，才能把连续性条件写成关于未知量的线性方程。
 */
struct DerivativeGradients
{
  std::array<double, 6> jerk_left{};   ///< τ = 0 处 q⃛ 对 6 个边界量的偏导
  std::array<double, 6> jerk_right{};  ///< τ = 1 处 q⃛ 对 6 个边界量的偏导
  std::array<double, 6> snap_left{};   ///< τ = 0 处 q⃜ 对 6 个边界量的偏导
  std::array<double, 6> snap_right{};  ///< τ = 1 处 q⃜ 对 6 个边界量的偏导
};

/**
 * @brief 计算一段（时长为 T）的 jerk / snap 关于 6 个边界量的梯度。
 * @param T [in] 该段时长，单位 s，须 > 0。
 * @return 四个梯度行向量，顺序均为 (qL, vL, aL, qR, vR, aR)。
 *
 * @note 推导（与文件头"边界条件的解析解"配合看）：
 *       记 d0 = (qR−qL) − vL·T − aL·T²/2、d1 = vR·T − vL·T − aL·T²、
 *           d2 = aR·T² − aL·T²，并把 c3、c4、c5 的表达式代入残差多项式
 *           r(τ) = c3τ³ + c4τ⁴ + c5τ⁵，合并同类项得
 *             r'''(1)  = 6c3 + 24c4 + 60c5 = 60d0 − 36d1 + 9d2
 *             r'''(0)  = 6c3               = 60d0 − 24d1 + 3d2
 *             r''''(1) = 24c4 + 120c5      = 360d0 − 192d1 + 36d2
 *             r''''(0) = 24c4              = −360d0 + 168d1 − 24d2
 *       再除以 T³ / T⁴ 把"对 τ 的导数"换成"对 t 的导数"，而 d 又是 6 个边界量
 *       的线性组合：
 *             ∂d0 = (−1, −T, −T²/2, +1, 0, 0)
 *             ∂d1 = ( 0, −T, −T² ,  0, +T, 0)
 *             ∂d2 = ( 0,  0, −T² ,  0,  0, +T²)
 *       两者线性叠加即得下面的四个行向量。
 */
DerivativeGradients makeDerivativeGradients(double T)
{
  const double T2 = T * T;
  const double T3 = T2 * T;
  const double T4 = T3 * T;

  const std::array<double, 6> grad_d0{-1.0, -T, -T2 / 2.0, 1.0, 0.0, 0.0};
  const std::array<double, 6> grad_d1{0.0, -T, -T2, 0.0, T, 0.0};
  const std::array<double, 6> grad_d2{0.0, 0.0, -T2, 0.0, 0.0, T2};

  // 把 (k0·d0 + k1·d1 + k2·d2) / scale 这个标量表达式整体搬到梯度上：
  // 因为 d*(·) 都是线性的，系数可以逐项线性搬运。
  const auto combine = [&](double k0, double k1, double k2, double scale) {
    std::array<double, 6> out{};
    for (std::size_t i = 0; i < out.size(); ++i) {
      out[i] = (k0 * grad_d0[i] + k1 * grad_d1[i] + k2 * grad_d2[i]) / scale;
    }
    return out;
  };

  DerivativeGradients gradients;
  gradients.jerk_right = combine(60.0, -36.0, 9.0, T3);
  gradients.jerk_left = combine(60.0, -24.0, 3.0, T3);
  gradients.snap_right = combine(360.0, -192.0, 36.0, T4);
  gradients.snap_left = combine(-360.0, 168.0, -24.0, T4);
  return gradients;
}

/**
 * @brief 单段内 |q̇| 与 |q̈| 的峰值。
 * @param coefficients [in]  段系数。
 * @param velocity_peak [out] 速度峰值，单位与系数的位置量一致（rad/s 或 m/s）。
 * @param acceleration_peak [out] 加速度峰值，单位 rad/s² 或 m/s²。
 *
 * @note 用固定步长采样取峰值，属于**工程近似**：τ 空间的 q̇ 是四次式、q̈ 是
 *       三次式，精确求极值要解四次/三次方程。本模块只对 jerk 做精确解析，
 *       因为 jerk 是二次式、结构最简单，而它恰好又是最需要盯住的量。
 */
void peakVelocityAcceleration(
  const QuinticCoefficients & coefficients, double & velocity_peak, double & acceleration_peak)
{
  velocity_peak = 0.0;
  acceleration_peak = 0.0;
  for (unsigned int i = 0; i <= kLimitCheckSamples; ++i) {
    const double tau = static_cast<double>(i) / static_cast<double>(kLimitCheckSamples);
    double q = 0.0, qdot = 0.0, qddot = 0.0;
    evaluateQuinticSegment(coefficients, tau, q, qdot, qddot);
    velocity_peak = std::max(velocity_peak, std::abs(qdot));
    acceleration_peak = std::max(acceleration_peak, std::abs(qddot));
  }
}

/**
 * @brief 单段内 |q⃛| 的**精确**峰值。
 * @param coefficients [in] 段系数。
 * @return jerk 峰值，单位 rad/s³ 或 m/s³。
 *
 * @note τ 空间的三阶导 g(τ) = 6c3 + 24c4·τ + 60c5·τ² 是 τ 的二次式，
 *       极值只可能出现在两端的 τ = 0、τ = 1，以及 g'(τ) = 24c4 + 120c5·τ = 0
 *       给出的驻点 τ* = −c4 / (5·c5)（若该驻点落在段内）。三处取最大即可，
 *       不需要任何迭代或采样。
 */
double peakJerk(const QuinticCoefficients & coefficients)
{
  if (coefficients.duration <= 0.0) {
    return 0.0;
  }
  const double c3 = coefficients.c[3];
  const double c4 = coefficients.c[4];
  const double c5 = coefficients.c[5];

  const auto g = [&](double tau) {
    return 6.0 * c3 + 24.0 * c4 * tau + 60.0 * c5 * tau * tau;
  };

  double best = std::max(std::abs(g(0.0)), std::abs(g(1.0)));
  if (c5 != 0.0) {
    const double tau = -c4 / (5.0 * c5);
    if (tau > 0.0 && tau < 1.0) {
      best = std::max(best, std::abs(g(tau)));
    }
  }
  return best / (coefficients.duration * coefficients.duration * coefficients.duration);
}

}  // namespace

// ---------------------------------------------------------------------------
// 一、单段：系数与求值
// ---------------------------------------------------------------------------

bool computeQuinticCoefficients(
  double q0, double v0, double a0, double q1, double v1, double a1, double duration,
  QuinticCoefficients & coefficients)
{
  if (duration <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "computeQuinticCoefficients: 段时长必须 > 0（当前 %.6f s）",
      duration);
    return false;
  }

  // 左端三个条件直接确定前三个系数（注意 v、a 要乘 T 的幂，
  // 因为对 τ 求导时会各带下来一个 1/T）。
  const double c0 = q0;
  const double c1 = v0 * duration;
  const double c2 = a0 * duration * duration / 2.0;

  // 右端条件化归成三个残差
  const double d0 = (q1 - q0) - c1 - c2;
  const double d1 = v1 * duration - c1 - 2.0 * c2;
  const double d2 = a1 * duration * duration - 2.0 * c2;

  // 解 r(1) = d0、r'(1) = d1、r''(1) = d2，其中 r(τ) = c3τ³ + c4τ⁴ + c5τ⁵
  const double c3 = 10.0 * d0 - 4.0 * d1 + 0.5 * d2;
  const double c4 = -15.0 * d0 + 7.0 * d1 - d2;
  const double c5 = 6.0 * d0 - 3.0 * d1 + 0.5 * d2;

  coefficients.c = {c0, c1, c2, c3, c4, c5};
  coefficients.duration = duration;
  return true;
}

bool evaluateQuinticSegment(
  const QuinticCoefficients & coefficients, double tau, double & q, double & qdot, double & qddot)
{
  if (coefficients.duration <= 0.0) {
    return false;
  }

  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;

  const std::array<double, 6> & c = coefficients.c;
  const double q_tau =
    c[0] + c[1] * tau + c[2] * tau2 + c[3] * tau3 + c[4] * tau4 + c[5] * tau5;
  const double dq_tau =
    c[1] + 2.0 * c[2] * tau + 3.0 * c[3] * tau2 + 4.0 * c[4] * tau3 + 5.0 * c[5] * tau4;
  const double ddq_tau =
    2.0 * c[2] + 6.0 * c[3] * tau + 12.0 * c[4] * tau2 + 20.0 * c[5] * tau3;

  // 把"对 τ 的导数"换算成"对 t 的导数"：每求一次导就多除一个 T。
  const double inv_T = 1.0 / coefficients.duration;
  q = q_tau;
  qdot = dq_tau * inv_T;
  qddot = ddq_tau * inv_T * inv_T;
  return true;
}

double quinticSegmentJerk(const QuinticCoefficients & coefficients, double tau)
{
  if (coefficients.duration <= 0.0) {
    return 0.0;
  }
  const std::array<double, 6> & c = coefficients.c;
  const double jerk_tau = 6.0 * c[3] + 24.0 * c[4] * tau + 60.0 * c[5] * tau * tau;
  const double inv_T = 1.0 / coefficients.duration;
  return jerk_tau * inv_T * inv_T * inv_T;
}

double quinticSegmentSnap(const QuinticCoefficients & coefficients, double tau)
{
  if (coefficients.duration <= 0.0) {
    return 0.0;
  }
  const std::array<double, 6> & c = coefficients.c;
  const double snap_tau = 24.0 * c[4] + 120.0 * c[5] * tau;
  const double inv_T = 1.0 / coefficients.duration;
  return snap_tau * inv_T * inv_T * inv_T * inv_T;
}

// ---------------------------------------------------------------------------
// 二、轨迹类成员函数
// ---------------------------------------------------------------------------

double QuinticTrajectory::segmentDuration(unsigned int index) const
{
  if (index >= durations_.size()) {
    return 0.0;
  }
  return durations_[index];
}

const QuinticCoefficients & QuinticTrajectory::segmentCoefficients(
  unsigned int segment, unsigned int joint) const
{
  // 越界时返回一个静态的空系数，避免调用者拿到悬垂引用。
  static const QuinticCoefficients kEmpty;
  if (segment >= coefficients_.size() || joint >= coefficients_[segment].size()) {
    return kEmpty;
  }
  return coefficients_[segment][joint];
}

bool QuinticTrajectory::locateSegment(double t, unsigned int & index, double & tau) const
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

bool QuinticTrajectory::sample(
  double t, KDL::JntArray & q, KDL::JntArray & qdot, KDL::JntArray & qddot) const
{
  unsigned int index = 0;
  double tau = 0.0;
  if (!locateSegment(t, index, tau)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "sample: 轨迹尚未成功构建，无法采样");
    return false;
  }

  q.resize(joints_);
  qdot.resize(joints_);
  qddot.resize(joints_);

  for (unsigned int j = 0; j < joints_; ++j) {
    double q_j = 0.0, qdot_j = 0.0, qddot_j = 0.0;
    evaluateQuinticSegment(coefficients_[index][j], tau, q_j, qdot_j, qddot_j);
    q(j) = q_j;
    qdot(j) = qdot_j;
    qddot(j) = qddot_j;
  }
  return true;
}

bool QuinticTrajectory::sampleDerivatives(
  double t, KDL::JntArray & jerk, KDL::JntArray & snap) const
{
  unsigned int index = 0;
  double tau = 0.0;
  if (!locateSegment(t, index, tau)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "sampleDerivatives: 轨迹尚未成功构建，无法采样");
    return false;
  }

  jerk.resize(joints_);
  snap.resize(joints_);
  for (unsigned int j = 0; j < joints_; ++j) {
    jerk(j) = quinticSegmentJerk(coefficients_[index][j], tau);
    snap(j) = quinticSegmentSnap(coefficients_[index][j], tau);
  }
  return true;
}

bool QuinticTrajectory::validateLimits(const JointLimits & limits, std::string & message) const
{
  message.clear();
  if (!valid()) {
    message = "轨迹为空，无法校验";
    return false;
  }

  // 约束数组一旦启用，长度就必须与关节数一致，否则"哪个关节"无从谈起。
  const auto size_mismatch = [&](const KDL::JntArray & bound, const std::string & name) {
    if (bound.rows() != 0 && bound.rows() != joints_) {
      message = name + "上限的长度(" + std::to_string(bound.rows()) + ")与关节数(" +
                std::to_string(joints_) + ")不一致";
      return true;
    }
    return false;
  };
  if (size_mismatch(limits.max_velocity, "速度") ||
      size_mismatch(limits.max_acceleration, "加速度") || size_mismatch(limits.max_jerk, "jerk")) {
    return false;
  }

  // 只要速度、加速度里有一项要查，就一次性把两个峰值都采出来：
  // 反正都是同一趟循环里顺带算的，省下一次重复采样。
  const bool check_velocity = limits.check_velocity();
  const bool check_acceleration = limits.check_acceleration();
  const bool need_peaks = check_velocity || check_acceleration;

  for (unsigned int k = 0; k < segmentCount(); ++k) {
    for (unsigned int j = 0; j < joints_; ++j) {
      const QuinticCoefficients & coefficients = coefficients_[k][j];

      if (need_peaks) {
        double velocity_peak = 0.0, acceleration_peak = 0.0;
        peakVelocityAcceleration(coefficients, velocity_peak, acceleration_peak);
        if (check_velocity && velocity_peak > limits.max_velocity(j)) {
          message = "第 " + std::to_string(k) + " 段、关节 " + std::to_string(j) + " 的速度峰值 " +
                    std::to_string(velocity_peak) + " rad/s 超过上限 " +
                    std::to_string(limits.max_velocity(j)) + " rad/s";
          return false;
        }
        if (check_acceleration && acceleration_peak > limits.max_acceleration(j)) {
          message = "第 " + std::to_string(k) + " 段、关节 " + std::to_string(j) +
                    " 的加速度峰值 " + std::to_string(acceleration_peak) + " rad/s² 超过上限 " +
                    std::to_string(limits.max_acceleration(j)) + " rad/s²";
          return false;
        }
      }

      if (limits.check_jerk()) {
        const double jerk_peak = peakJerk(coefficients);
        const double bound = limits.max_jerk(j);
        if (jerk_peak > bound) {
          message = "第 " + std::to_string(k) + " 段、关节 " + std::to_string(j) + " 的 jerk 峰值 " +
                    std::to_string(jerk_peak) + " rad/s³ 超过上限 " + std::to_string(bound) +
                    " rad/s³";
          return false;
        }
      }
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 三、构建入口
// ---------------------------------------------------------------------------

TrajectoryResult buildQuinticTrajectory(
  const std::vector<KDL::JntArray> & waypoints, const std::vector<double> & durations,
  const JointLimits & limits)
{
  TrajectoryResult result;

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
  const unsigned int joints = waypoints.front().rows();
  if (joints == 0) {
    result.message = "路点向量长度为 0，至少要有一个关节";
    return result;
  }
  for (std::size_t k = 0; k < waypoints.size(); ++k) {
    if (waypoints[k].rows() != joints) {
      result.message = "第 " + std::to_string(k) + " 个路点的长度(" +
                       std::to_string(waypoints[k].rows()) + ")与第 0 个(" +
                       std::to_string(joints) + ")不一致";
      return result;
    }
  }
  for (std::size_t k = 0; k < durations.size(); ++k) {
    if (durations[k] <= 0.0) {
      result.message = "第 " + std::to_string(k) + " 段时长必须 > 0（当前 " +
                       std::to_string(durations[k]) + " s）";
      return result;
    }
  }

  // ---- 2. 落盘路点、时长、路点时刻 ----
  QuinticTrajectory & trajectory = result.trajectory;
  trajectory.joints_ = joints;
  trajectory.waypoints_ = waypoints;
  trajectory.durations_ = durations;
  trajectory.knot_times_.assign(durations.size() + 1, 0.0);
  for (unsigned int k = 0; k < segments; ++k) {
    trajectory.knot_times_[k + 1] = trajectory.knot_times_[k] + durations[k];
  }

  // ---- 3. 端点速度、加速度固定为 0（静止起步、静止停住）----
  // 内部路点的值稍后由方程组解出，这里先全部置零占位。
  trajectory.knot_velocities_.assign(waypoints.size(), KDL::JntArray(joints));
  trajectory.knot_accelerations_.assign(waypoints.size(), KDL::JntArray(joints));

  // ---- 4. 逐关节求解内部路点的 (v_i, a_i) ----
  //
  // 方程组（每个内部路点两个方程）：
  //     jerk 连续：第 k 段右端 jerk  = 第 k+1 段左端 jerk
  //     snap 连续：第 k 段右端 snap  = 第 k+1 段左端 snap
  // 未知量编号：x[2(i−1)] = v_i，x[2(i−1)+1] = a_i，i = 1..m−1
  //
  // 注意各关节之间**没有耦合**：连续性条件天然按关节独立，所以这里逐关节
  // 单独求解，矩阵最大也就 2(m−1) 阶，代码和数学一一对应，便于课堂核对。
  const unsigned int unknowns = 2 * (segments - 1);
  for (unsigned int j = 0; j < joints; ++j) {
    if (unknowns == 0) {
      break;  // 单段情形没有内部路点，端点又恒为 0，无需解方程
    }

    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(unknowns, unknowns);
    Eigen::VectorXd rhs = Eigen::VectorXd::Zero(unknowns);

    /// 路点 i 是否为未知量：只有内部路点（1 ≤ i ≤ m−1）才是。
    const auto is_unknown = [segments](unsigned int knot) {
      return knot >= 1 && knot < segments;
    };
    const auto velocity_index = [](unsigned int knot) { return 2 * static_cast<int>(knot) - 2; };
    const auto acceleration_index = [](unsigned int knot) { return 2 * static_cast<int>(knot) - 1; };

    // 把一段在某一端处的 jerk（或 snap）表达式累加进 (matrix, rhs)。
    // grad 的顺序为 (qL, vL, aL, qR, vR, aR)，sign 用于"两段相减"。
    const auto accumulate = [&](int row, double sign, const std::array<double, 6> & grad,
                                unsigned int knot_left, unsigned int knot_right) {
      // 位置是已知量，直接移到方程右端
      rhs(row) -= sign * (grad[0] * trajectory.waypoints_[knot_left](j) +
                          grad[3] * trajectory.waypoints_[knot_right](j));

      if (is_unknown(knot_left)) {
        matrix(row, velocity_index(knot_left)) += sign * grad[1];
        matrix(row, acceleration_index(knot_left)) += sign * grad[2];
      } else {
        // 端点：值已知（0），照样移到右端，保证代码对"端点非零"也是对的
        rhs(row) -= sign * (grad[1] * trajectory.knot_velocities_[knot_left](j) +
                            grad[2] * trajectory.knot_accelerations_[knot_left](j));
      }

      if (is_unknown(knot_right)) {
        matrix(row, velocity_index(knot_right)) += sign * grad[4];
        matrix(row, acceleration_index(knot_right)) += sign * grad[5];
      } else {
        rhs(row) -= sign * (grad[4] * trajectory.knot_velocities_[knot_right](j) +
                            grad[5] * trajectory.knot_accelerations_[knot_right](j));
      }
    };

    for (unsigned int k = 1; k < segments; ++k) {
      const int row_jerk = static_cast<int>(2 * (k - 1));
      const int row_snap = row_jerk + 1;

      const DerivativeGradients left = makeDerivativeGradients(durations[k - 1]);
      const DerivativeGradients right = makeDerivativeGradients(durations[k]);

      // 第 k 段：路点 k−1 → k；第 k+1 段：路点 k → k+1
      accumulate(row_jerk, +1.0, left.jerk_right, k - 1, k);
      accumulate(row_jerk, -1.0, right.jerk_left, k, k + 1);
      accumulate(row_snap, +1.0, left.snap_right, k - 1, k);
      accumulate(row_snap, -1.0, right.snap_left, k, k + 1);
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> solver(matrix);
    const Eigen::VectorXd solution = solver.solve(rhs);
    const double residual = (matrix * solution - rhs).norm();
    if (!solver.isInvertible() || residual > 1e-8 * (1.0 + rhs.norm())) {
      result.message = "关节 " + std::to_string(j) +
                       " 的连续性方程组奇异或病态（残差 " + std::to_string(residual) +
                       "），无法求出唯一的速度/加速度解";
      return result;
    }

    for (unsigned int k = 1; k < segments; ++k) {
      trajectory.knot_velocities_[k](j) = solution(velocity_index(k));
      trajectory.knot_accelerations_[k](j) = solution(acceleration_index(k));
    }
  }

  // ---- 5. 用解出来的路点边界条件，逐段逐关节求五次多项式系数 ----
  trajectory.coefficients_.resize(segments);
  for (unsigned int k = 0; k < segments; ++k) {
    trajectory.coefficients_[k].resize(joints);
    for (unsigned int j = 0; j < joints; ++j) {
      // 这里就是"多段复用单段公式"的落点：每段的系数计算与单段演示完全同源。
      computeQuinticCoefficients(
        trajectory.waypoints_[k](j), trajectory.knot_velocities_[k](j),
        trajectory.knot_accelerations_[k](j), trajectory.waypoints_[k + 1](j),
        trajectory.knot_velocities_[k + 1](j), trajectory.knot_accelerations_[k + 1](j),
        durations[k], trajectory.coefficients_[k][j]);
    }
  }

  // ---- 6. 整条轨迹的可行性校验：超限就判不可行，不做任何 clamp ----
  std::string message;
  if (!trajectory.validateLimits(limits, message)) {
    result.message = "轨迹不可行（给定时间参数太短或路点跨度过大）：" + message;
    return result;
  }

  result.success = true;
  return result;
}

}  // namespace kdl_interpolation
