// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：关节空间五次多项式插值。
//
// ===========================================================================
// 一、为什么是"五次"
// ===========================================================================
// 想让每个关节平滑地"起步"和"停住"，通常要求起点、终点的位置、速度、
// 加速度都已知，一共 6 个边界条件。多项式至少要 6 个系数才能被唯一确定，
// 所以取五次：
//
//     q(t) = c0 + c1·t + c2·t² + c3·t³ + c4·t⁴ + c5·t⁵
//
// 三次多项式只有 4 个系数，只能管住位置和速度，管不住加速度；
// 五次才能同时把位置、速度、加速度都约束住。
//
// ===========================================================================
// 二、为什么把时间归一化
// ===========================================================================
// 令 τ = t / T（τ ∈ [0,1]，T 为该段时长），把 q 视作 τ 的函数：
//
//     q(τ) = c0 + c1·τ + c2·τ² + c3·τ³ + c4·τ⁴ + c5·τ⁵
//
// 好处：系数公式与段时长 T 彻底解耦，T 只出现在"求导换算"这一步，
// 于是每段都能复用同一套公式，中途不必反复除 T。
//
// 换算关系（求值时务必记住，因为物理量要的是"每秒"而不是"每 τ"）：
//
//     q̇ = q'/T ,  q̈ = q''/T² ,  q⃛ = q'''/T³ ,  q⃜ = q''''/T⁴
//
// 其中撇号表示对 τ 求导。本头文件的返回量一律是**物理量**（含时间单位），
// 只有 QuinticCoefficients::c 里装的是 τ 空间的系数。
//
// ===========================================================================
// 三、边界条件的解析解
// ===========================================================================
// 记 c1 = v0·T、c2 = a0·T²/2（由左端条件直接确定），再记
//
//     d0 = (q1 − q0) − c1 − c2        （右端位置残差）
//     d1 = v1·T − c1 − 2·c2           （右端"速度×T"残差）
//     d2 = a1·T² − 2·c2               （右端"加速度×T²"残差）
//
// 剩余三个系数就是残差多项式 r(τ) = c3τ³ + c4τ⁴ + c5τ⁵ 的解：
//
//     c3 = 10·d0 − 4·d1 + 0.5·d2
//     c4 = −15·d0 + 7·d1 − d2
//     c5 = 6·d0 − 3·d1 + 0.5·d2
//
// 当 d1 = d2 = 0、d0 = Δq 时，它退化成教科书里的最小 jerk 轨迹
// q(τ) = q0 + Δq·(10τ³ − 15τ⁴ + 6τ⁵)。
//
// ===========================================================================
// 四、多段怎么拼接（本模块的核心）
// ===========================================================================
// 约定：每个路点只分配**一组** (v_i, a_i)，左右两段共用这一组值。
// 于是"速度连续 + 加速度连续"在构造上就自动成立，不需要额外列方程。
//
// 剩下的自由度用来让三阶导（jerk）与四阶导（snap）在路点处也连续，
// 方程数与未知量数恰好相等，因此有唯一解：
//
//     未知量：内部路点的 (v_i, a_i)，i = 1..m−1            —— 2(m−1) 个
//     方  程：内部路点处 jerk 连续 + snap 连续              —— 2(m−1) 个
//     已  知：两端 (v0, a0) 与 (vm, am)，由调用者给定（默认全零）
//
// 结果整条轨迹 **C⁴ 连续**（位置、速度、加速度、jerk、snap 全部连续）。
// 线性系统按关节解耦：不同关节之间没有耦合，所以可以逐个关节独立求解。
//
// ---------------------------------------------------------------------------
// 必须知道的取舍：端点 jerk 不为零
// ---------------------------------------------------------------------------
// "两端 v = a = 0" 与 "两端 jerk = 0" 在五次多项式框架下是**互斥**的。
// 以单段为例，满足两端 v = a = 0 的五次多项式只有一条，即
//
//     q(τ) = q0 + Δq·(10τ³ − 15τ⁴ + 6τ⁵),  其 q⃛(0) = 60·Δq/T³ ≠ 0
//
// 换句话说，端点的 jerk 不是可以随便指定的量，它由"总位移 ÷ 时间的三次方"
// 决定：想快就得忍受大 jerk。这恰恰是工程上最需要盯住的量，所以本模块在
// 构建轨迹后会**校验**整条轨迹的速度/加速度/jerk 是否超出机器人限制，
// 超限就判定"该时间参数下轨迹不可行"并返回错误，而不是悄悄把结果裁掉。
//
// ===========================================================================
// 五、模块内的东西怎么配合
// ===========================================================================
//   单段：computeQuinticCoefficients() → evaluateQuinticSegment()
//         （纯数学，不含任何机器人约束概念，方便课堂上单独讲）
//   多段：buildQuinticTrajectory() 内部逐段复用上面的单段公式，
//         并做整条轨迹的约束校验
//   打印：见 kdl_interpolation_print.hpp（求解逻辑不需要知道怎么打印）

#ifndef KDL_INTERPOLATION__KDL_QUINTIC_HPP_
#define KDL_INTERPOLATION__KDL_QUINTIC_HPP_

#include <array>
#include <string>
#include <vector>

#include <kdl/jntarray.hpp>

namespace kdl_interpolation
{

// ---------------------------------------------------------------------------
// 一、单段：系数与求值
// ---------------------------------------------------------------------------

/**
 * @brief 一段五次多项式的系数（时间已归一化为 τ = t / T）。
 *
 * @note 系数是"对 τ"的，不是"对 t"的：q(τ) = c[0] + c[1]·τ + ... + c[5]·τ⁵。
 *       想拿物理速度/加速度，请用 evaluateQuinticSegment()，它内部会做
 *       q̇ = q'/T、q̈ = q''/T² 的换算，避免调用者自己漏除 T 的幂。
 */
struct QuinticCoefficients
{
  /// τ 空间系数：q(τ) = c[0] + c[1]τ + c[2]τ² + c[3]τ³ + c[4]τ⁴ + c[5]τ⁵
  std::array<double, 6> c{};

  /// 该段时长 T，单位 s（必须 > 0）。
  double duration = 0.0;
};

/**
 * @brief 由 6 个边界条件求单段五次多项式系数（纯数学，不含任何限位概念）。
 * @param q0 [in]  起点位置，单位 rad（平移关节为 m）。
 * @param v0 [in]  起点速度，单位 rad/s（平移关节为 m/s）。
 * @param a0 [in]  起点加速度，单位 rad/s²（平移关节为 m/s²）。
 * @param q1 [in]  终点位置，单位 rad（平移关节为 m）。
 * @param v1 [in]  终点速度，单位 rad/s（平移关节为 m/s）。
 * @param a1 [in]  终点加速度，单位 rad/s²（平移关节为 m/s²）。
 * @param duration [in] 段时长 T，单位 s，必须 > 0。
 * @param coefficients [out] 求解得到的系数（含归一化所需的时间 T）。
 * @return true 表示成功；false 表示 duration <= 0，此时 coefficients 不被修改。
 *
 * @note 六个边界条件里任意三个都由左端确定、三个由右端确定，所以调用者可以
 *       "只想约束起点"或"只想约束终点"，把不关心的量传 0 即可——代价是那条
 *       曲线真的会按 0 去走，所以别把"不关心"和"随便"混为一谈。
 */
bool computeQuinticCoefficients(
  double q0, double v0, double a0, double q1, double v1, double a1, double duration,
  QuinticCoefficients & coefficients);

/**
 * @brief 在归一化时刻 τ 处求单段的位置、速度、加速度。
 * @param coefficients [in]  段系数，须由 computeQuinticCoefficients() 得到。
 * @param tau          [in]  归一化时刻 ∈ [0,1]（τ = t/T）。
 * @param q            [out] 位置，单位 rad（平移关节为 m）。
 * @param qdot         [out] 速度，单位 rad/s（平移关节为 m/s）。
 * @param qddot        [out] 加速度，单位 rad/s²（平移关节为 m/s²）。
 * @return true 表示成功；false 表示 coefficients.duration <= 0（无效段）。
 *
 * @note 本函数不检查 τ 的范围：越界外插在数学上是有定义的，只是不再代表
 *       原来那条"两端停住"的轨迹。多段轨迹的 sample() 才会做 clamp。
 */
bool evaluateQuinticSegment(
  const QuinticCoefficients & coefficients, double tau, double & q, double & qdot,
  double & qddot);

/**
 * @brief 在归一化时刻 τ 处求单段的 jerk（三阶导）。
 * @param coefficients [in] 段系数。
 * @param tau          [in] 归一化时刻 ∈ [0,1]。
 * @return jerk，单位 rad/s³（平移关节为 m/s³）；duration <= 0 时返回 0。
 *
 * @note τ 空间的 q''' 是 τ 的二次式，所以 jerk 的极值可以解析求出：
 *       令 d(q''')/dτ = 0 解一个一次方程即可，详见 kdl_quintic.cpp。
 *       这正是本模块用"解析法"而不是"采样法"校验 jerk 的原因。
 */
double quinticSegmentJerk(const QuinticCoefficients & coefficients, double tau);

/**
 * @brief 在归一化时刻 τ 处求单段的 snap（四阶导）。
 * @param coefficients [in] 段系数。
 * @param tau          [in] 归一化时刻 ∈ [0,1]。
 * @return snap，单位 rad/s⁴（平移关节为 m/s⁴）；duration <= 0 时返回 0。
 *
 * @note snap 在 τ 空间是 τ 的一次式，其最大值必在段的两端，无需解方程。
 */
double quinticSegmentSnap(const QuinticCoefficients & coefficients, double tau);

// ---------------------------------------------------------------------------
// 二、关节约束（用于判断轨迹是否可行）
// ---------------------------------------------------------------------------

/**
 * @brief 机器人各关节的速度/加速度/jerk 上限。
 *
 * @note 三个数组彼此独立：某个数组长度为 0 就表示"不校验这一项"。
 *       默认构造出来的三个数组都为空，即**完全不做校验**——这样单段纯插值
 *       演示不会被限位打扰，只有显式设置过的项才会参与可行性判定。
 * @note 单位：速度 rad/s、加速度 rad/s²、jerk rad/s³（平移关节相应换成 m 系）。
 */
struct JointLimits
{
  /// |q̇| 上限 [rad/s]；长度 0 表示不校验。
  KDL::JntArray max_velocity;

  /// |q̈| 上限 [rad/s²]；长度 0 表示不校验。
  KDL::JntArray max_acceleration;

  /// |q⃛| 上限 [rad/s³]；长度 0 表示不校验。
  KDL::JntArray max_jerk;

  /// @return 是否已设置速度上限。
  bool check_velocity() const { return max_velocity.rows() > 0; }

  /// @return 是否已设置加速度上限。
  bool check_acceleration() const { return max_acceleration.rows() > 0; }

  /// @return 是否已设置 jerk 上限。
  bool check_jerk() const { return max_jerk.rows() > 0; }
};

// ---------------------------------------------------------------------------
// 三、多段 C⁴ 连续轨迹
// ---------------------------------------------------------------------------

struct TrajectoryResult;

/**
 * @brief 关节空间五次样条轨迹：路点 + 段时长 → C⁴ 连续的多段轨迹。
 *
 * @note 本类只做"存储 + 采样 + 校验"，不允许外部随意捏造内部状态，
 *       所以唯一正规的产生方式是 buildQuinticTrajectory()。
 */
class QuinticTrajectory
{
public:
  QuinticTrajectory() = default;

  /**
   * @brief 轨迹是否可用。
   * @return 构建成功时返回 true。
   */
  bool valid() const { return !coefficients_.empty(); }

  /**
   * @brief 关节数（= 每个路点向量的长度）。
   * @return 关节数；轨迹无效时返回 0。
   */
  unsigned int joints() const { return joints_; }

  /**
   * @brief 段数 m（路点数为 m + 1）。
   * @return 段数；轨迹无效时返回 0。
   */
  unsigned int segmentCount() const { return static_cast<unsigned int>(durations_.size()); }

  /**
   * @brief 轨迹总时长。
   * @return 各段时长之和，单位 s；轨迹无效时返回 0。
   */
  double duration() const { return knot_times_.empty() ? 0.0 : knot_times_.back(); }

  /**
   * @brief 取第 index 段的时长。
   * @param index [in] 段序号，从 0 开始，须 < segmentCount()。
   * @return 段时长，单位 s；越界返回 0。
   */
  double segmentDuration(unsigned int index) const;

  /**
   * @brief 取路点序列（含首尾）。
   * @return 长度 (段数 + 1) 的数组，每个元素是该路点的关节角 [rad]。
   */
  const std::vector<KDL::JntArray> & waypoints() const { return waypoints_; }

  /**
   * @brief 取各路点处求解出来的关节速度。
   * @return 长度 (段数 + 1) 的数组，单位 rad/s；两端恒为 0。
   *
   * @note 内部路点的速度是"全局方程组解出来的"，不是差分估出来的；
   *       它左右两段共用，所以速度连续是构造上成立的。
   */
  const std::vector<KDL::JntArray> & knotVelocities() const { return knot_velocities_; }

  /**
   * @brief 取各路点处求解出来的关节加速度。
   * @return 长度 (段数 + 1) 的数组，单位 rad/s²；两端恒为 0。
   */
  const std::vector<KDL::JntArray> & knotAccelerations() const { return knot_accelerations_; }

  /**
   * @brief 取各路点对应的时刻。
   * @return 长度 (段数 + 1) 的数组，单位 s，首元素为 0。
   */
  const std::vector<double> & knotTimes() const { return knot_times_; }

  /**
   * @brief 取第 segment 段、第 joint 个关节的系数。
   * @param segment [in] 段序号，从 0 开始。
   * @param joint   [in] 关节序号，从 0 开始。
   * @return 该段该关节的系数；越界时返回 duration = 0 的空系数。
   */
  const QuinticCoefficients & segmentCoefficients(unsigned int segment, unsigned int joint) const;

  /**
   * @brief 在时刻 t 采样轨迹。
   * @param t     [in]  时间，单位 s。超出 [0, 总时长] 时**自动 clamp** 到端点，
   *                    即 t<0 返回起点值、t>总时长 返回终点值，不报错。
   * @param q     [out] 关节角，单位 rad（平移关节为 m）；函数内部会自动 resize。
   * @param qdot  [out] 关节速度，单位 rad/s。
   * @param qddot [out] 关节加速度，单位 rad/s²。
   * @return true 表示成功；false 表示轨迹无效（此时输出不被修改）。
   *
   * @note 用一次调用同时取回三个量，是因为控制器每个周期本来就要三者齐备，
   *       分三次调用反而要多做三次"定位到哪一段"的查找。
   */
  bool sample(
    double t, KDL::JntArray & q, KDL::JntArray & qdot, KDL::JntArray & qddot) const;

  /**
   * @brief 在时刻 t 采样轨迹的 jerk 与 snap（用于观察平滑性、复核约束）。
   * @param t    [in]  时间，单位 s，越界同样 clamp。
   * @param jerk [out] 关节 jerk，单位 rad/s³。
   * @param snap [out] 关节 snap，单位 rad/s⁴。
   * @return true 表示成功；false 表示轨迹无效。
   *
   * @note 单段五次多项式的 snap 是 τ 的一次式，jerk 是二次式，两者都能精确
   *       求值；把它们单独作为接口，是为了让示例可以直接画出"jerk 在路点处
   *       连续"这件事，而不必去读内部系数。
   */
  bool sampleDerivatives(double t, KDL::JntArray & jerk, KDL::JntArray & snap) const;

  /**
   * @brief 校验整条轨迹是否满足关节约束。
   * @param limits  [in]  约束；未设置的项自动跳过。
   * @param message [out] 失败时填入"第几段、哪个关节、哪一项、峰值多少、
   *                      上限多少"的可读说明；成功时清空。
   * @return true 表示全部满足（或没有任何约束需要检查）。
   *
   * @note 判定方式：
   *       - jerk：解析求极值——jerk 在 τ 空间是二次式，令其导数为 0 解出驻点，
   *         再与两端比较，结果精确；
   *       - 速度、加速度：按固定步长在每段内采样取峰值。这两者在 τ 空间分别
   *         是四次、三次式，精确求极值需要解四次/三次方程，为教学简洁起见改用
   *         密集采样（kLimitCheckSamples 个点），并在结论里如实说明这是近似。
   */
  bool validateLimits(const JointLimits & limits, std::string & message) const;

private:
  friend TrajectoryResult buildQuinticTrajectory(
    const std::vector<KDL::JntArray> &, const std::vector<double> &, const JointLimits &);

  /**
   * @brief 把时刻 t 定位到具体段与归一化时刻 τ。
   * @param t     [in]  时间，单位 s（内部先 clamp 到 [0, 总时长]）。
   * @param index [out] 段序号。
   * @param tau   [out] 归一化时刻 ∈ [0,1]。
   * @return true 表示轨迹有效、定位成功。
   */
  bool locateSegment(double t, unsigned int & index, double & tau) const;

  /// 关节数。
  unsigned int joints_ = 0;

  /// 路点关节角 [rad]，长度 = 段数 + 1。
  std::vector<KDL::JntArray> waypoints_;

  /// 路点关节速度 [rad/s]，长度 = 段数 + 1。
  std::vector<KDL::JntArray> knot_velocities_;

  /// 路点关节加速度 [rad/s²]，长度 = 段数 + 1。
  std::vector<KDL::JntArray> knot_accelerations_;

  /// 各段时长 [s]，长度 = 段数。
  std::vector<double> durations_;

  /// 各路点时刻 [s]，长度 = 段数 + 1，首元素为 0。
  std::vector<double> knot_times_;

  /// 各段各关节的系数：coefficients_[段][关节]。
  std::vector<std::vector<QuinticCoefficients>> coefficients_;
};

// ---------------------------------------------------------------------------
// 四、构建入口
// ---------------------------------------------------------------------------

/**
 * @brief 轨迹构建结果。
 *
 * @note 与 kdl_kinematics 的 VelocityResult 同样的思路：把"能不能用"和
 *       "为什么不能用"一起交给调用者，避免用异常打断实时控制循环。
 */
struct TrajectoryResult
{
  /// 构建成功时可直接使用的轨迹；失败时为一具空壳（valid() == false）。
  QuinticTrajectory trajectory;

  /// 是否构建成功。
  bool success = false;

  /// 失败原因（中文、可直接打印）：输入非法、方程组奇异、或哪一段哪个关节超限。
  std::string message;
};

/**
 * @brief 由路点与段时长构建 C⁴ 连续的五次样条轨迹。
 * @param waypoints [in] 路点关节角，长度 ≥ 2；每个元素长度须一致且 ≥ 1，
 *                      单位 rad（平移关节为 m）。
 * @param durations [in] 各段时长，长度须 = waypoints.size() − 1，每个元素 > 0，单位 s。
 * @param limits    [in] 关节约束；默认构造为"不校验"。
 * @return TrajectoryResult；success 为 false 时 message 说明原因。
 *
 * @note 两端速度、加速度固定为 0（静止起步、静止停住）；内部路点的速度与
 *       加速度由全局方程组解出，使得整条轨迹 C⁴ 连续。详见文件头注释。
 * @note 失败一律**不 clamp、不自动拉长时间**：超出机器人限制就如实判为不可行，
 *       由调用者自己决定是放慢、改路点还是换方案——"悄悄改慢"会让实际执行
 *       的时间和调用者以为的时间不一致，是更隐蔽的错。
 * @note 构建完会立即调用一次 validateLimits()，所以 success == true 就意味着
 *       这条轨迹在给定约束下是可行的。
 */
TrajectoryResult buildQuinticTrajectory(
  const std::vector<KDL::JntArray> & waypoints, const std::vector<double> & durations,
  const JointLimits & limits = JointLimits());

}  // namespace kdl_interpolation

#endif  // KDL_INTERPOLATION__KDL_QUINTIC_HPP_
