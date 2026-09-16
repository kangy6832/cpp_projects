// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：笛卡尔空间位姿插值（位置用五次多项式，姿态用 slerp）。
//
// ===========================================================================
// 一、笛卡尔插值和关节空间插值的区别
// ===========================================================================
// 关节空间插值（kdl_quintic）插的是 q，末端走成什么形状由机构决定，"路径"在
// 关节空间里看不见；笛卡尔插值插的是末端位姿 T ∈ SE(3)，末端沿你指定的几何
// 路径走，本模块取的就是最朴素的一条：位置走直线，姿态绕固定轴转过去。
//
// 位姿不能当成一个整体来插，因为它的两半性质完全不同：
//   位置 p ∈ R³：三个普通标量，可以逐分量插值 → 五次多项式（复用 kdl_quintic）
//   姿态 R ∈ SO(3)：不构成线性空间，逐元素插值得到的矩阵通常不再是旋转
//                    → 只能用 slerp（球面线性插值）
//
// 于是本模块的核心就是一句话：
//   位置用五次多项式、姿态用 slerp，两者用同一个归一化时刻 τ = t/T 保持同步。
//
// ===========================================================================
// 二、位置部分：为什么可以照搬 kdl_quintic
// ===========================================================================
// x、y、z 是三个互不耦合的标量，p(τ) = Σ c_k·τ^k 逐分量成立，所以关节空间那套
// 结论原样搬过来：6 个边界条件 → 6 个系数；"两端 v=a=0"与"端点 jerk=0"不可兼得。
//
// 单段且两端 v=a=0 时还有一个很好的性质（本模块 kStop 模式每段都满足）：
// 三个分量共用同一个角参数形状 s(τ) = 10τ³ − 15τ⁴ + 6τ⁵，即
//
//     p(τ) = p0 + (p1 − p0)·s(τ)
//
// 因为三个分量被同一个 s(τ) 驱动，p 永远落在线段 p0→p1 上——**位置严格走直线**。
// 由此还能解析地写出速度、加速度峰值（Δp = |p1 − p0|）：
//
//     |v|max = 15·Δp/(8T) ≈ 1.875·Δp/T      （τ = 0.5 处）
//     |a|max = 10·Δp/(√3 T²) ≈ 5.7735·Δp/T² （τ = 1/2 ∓ √3/6 处）
//
// ===========================================================================
// 三、姿态部分：slerp 是什么
// ===========================================================================
// 任意两个姿态之间，总存在"转轴 n̂ + 转角 θ"的等效描述。把姿态想成单位球面上的
// 点（单位四元数），slerp 就是"沿两者之间那段大圆等速走过去"：
//
//     slerp(q0, q1, s) = q0 · (q0⁻¹ q1)^s ,   s ∈ [0,1]
//
// 记相对旋转 R_rel = R0⁻¹·R1 的等效转轴为 n̂（表达在 R0 坐标系中）、转角为 θ，
// 那么上面这条式子等价于"绕固定轴匀速转"。注意乘法次序——转轴表达在基座系里，
// 所以是**左乘**："先在基座系里绕固定轴转过 θ·s，再叠加起点姿态"：
//
//     R(s) = Rot(R0·n̂, θ·s) · R0        （ 等价于 R0 · Rot(n̂, θ·s) ）
//
// 本模块存的就是基座系里的那根轴 R0·n̂（SlerpSegment::axis_base），代码里用的是
// 左边的写法。两条式子靠旋转矩阵的相似变换性质 S·Rot(a,φ)·Sᵀ = Rot(S·a, φ) 互推。
//
// 几何图像：slerp 出来的是旋转球面上的一条测地线（大圆弧），因此它天然满足
// "转角与 s 成正比"，而且不会像逐元素插值那样在中间破坏正交性。
//
// ===========================================================================
// 四、角参数 s 为什么必须用五次时间标度（本模块最关键的取舍）
// ===========================================================================
// 最朴素的写法是 s(τ) = τ，即"匀速转过去"。但它的角速度是
//
//     |ω| = θ / T   （全程恒定，包括 t = 0 和 t = T 这两个瞬间）
//
// 而位置那一侧是按五次多项式走的，两端速度严格为 0。于是两条曲线对不上：
//   位置：静止起步 → 加速 → 减速 → 静止停住
//   姿态：一上来就以 θ/T 突跳着转起来，到点再突跳回 0
// 角速度的突跳意味着角加速度是 δ 函数，在真机上就是冲击。
//
// 所以本模块让角参数也走那条"最小 jerk 剖面"——它正是位置在两端 v=a=0 时解出来
// 的形状（同一个 s(τ)！）：
//
//     s(τ)   = 10τ³ − 15τ⁴ + 6τ⁵
//     s'(τ)  = 30τ²(1−τ)²            →  s'(0)  = s'(1)  = 0
//     s''(τ) = 60τ − 180τ² + 120τ³   →  s''(0) = s''(1) = 0
//
// 两端角速度、角加速度精确为 0，和位置同起同停。代价是峰值被抬高了：
//
//     |ω|max = θ·max|s'| /T  = 15θ/(8T)     ≈ 1.875·θ/T     （τ = 0.5 处）
//     |α|max = θ·max|s''|/T² = 10θ/(√3 T²)  ≈ 5.7735·θ/T²   （τ = 1/2 ∓ √3/6 处）
//
// 对照"匀速转"的 θ/T：想两端真正停住，峰值就得让出 1.875 倍。这是普遍取舍。
//
// ===========================================================================
// 五、多段怎么办：slerp 决定了"内部路点只能停住"
// ===========================================================================
// 每一段的 slerp 都是"绕该段自己的固定轴 n̂_k 转"（n̂_k 表达在基座系里，
// 见第三节的乘法次序），所以该段的角速度方向恒为 n̂_k：
//
//     ω(τ) = (θ_k/T_k)·s'(τ)·n̂_k      —— 方向与 τ 无关
//
// 相邻两段的轴一般**不平行**，要让角速度在路点处连续，就得满足
//
//     θ_{k−1}·s'_{k−1}(1)/T_{k−1} · n̂_{k−1}  =  θ_k·s'_k(0)/T_k · n̂_k
//
// n̂ 不平行时这个向量方程只有零解。结论：
//
//     **在 slerp 框架下，内部路点处的角速度只能为 0**（除非相邻段轴恰好共线）。
//
// 又因为本模块用的 s 满足 s'(0) = s'(1) = s''(0) = s''(1) = 0，"角速度与角加速度
// 在路点处为 0"是自动成立的，两种模式下都不需要额外解任何方程。
//
// 位置那一侧则有两种走法，本模块都提供，用 WaypointBehavior 选择：
//   - kStop（默认）：位置每段也两端 v = a = 0。整条轨迹在每个路点都精确停住，
//     位置与姿态行为一致（"走走停停"），而且从头到尾只用同一套单段公式，
//     不需要解任何线性方程组——因为起点静止、终点静止，永远不存在耦合。
//   - kPassThrough ：位置转调 kdl_quintic 的 C⁴ 全局拼接，路过内部路点时不减速
//     （更流畅、更省时间），但姿态受上面的限制仍会在路点停住，于是出现
//     "位置在动、姿态停住"。这不是实现偷懒，而是 slerp 框架下无法回避的取舍，
//     示例里会把它打印出来给你看。
//
// ===========================================================================
// 六、slerp 的两个奇异点
// ===========================================================================
// 由相对旋转取轴角要经过四元数 (x,y,z,w)，再解出
//
//     θ = 2·atan2(|v|, w),   v = (x,y,z),   n̂ = v/|v|
//
// 两个退化情形：
//   1) θ ≈ 0（两姿态几乎相同）：|v| → 0，转轴没有定义（转 0 度"绕哪根轴"都行，
//      也都不行）。本模块直接判定该段"无姿态变化"：θ = 0、轴取零向量，
//      姿态恒等于起点姿态，角速度与角加速度恒为 0，不会出现 NaN。
//   2) θ ≈ π（转 180°）：w ≈ 0，但此时 |v| ≈ 1，上面那条公式**依然稳定**，
//      所以 π 反而不是数值危险点。不唯一的只是轴的正负号（n̂ 与 −n̂ 描述同一个
//      180° 旋转，结果矩阵完全相同），对结果没有影响。
//
// "最短弧"由四元数的双重覆盖性质保证：q 与 −q 表示同一个姿态。拿到 q 后先看 w
// 的符号，w < 0 就整体取反，于是 θ ∈ [0, π] 恒成立——机器人不会为了"从负角那侧
// 绕"而白转一大圈。注意这个符号修正只影响"怎么转"，不影响"转到哪里"。
//
// ===========================================================================
// 七、模块内的东西怎么配合
// ===========================================================================
//   单段：computeCartesianSegment() → evaluateCartesianSegment()
//         （纯数学：3 个标量五次多项式 + 1 个 slerp，不含任何限位概念）
//   多段：buildCartesianTrajectory() 内部逐段复用上面的单段公式
//         （kPassThrough 模式下位置转调 kdl_quintic 的多段求解器）
//   打印：见 kdl_cartesian_print.hpp（求解逻辑不需要知道怎么打印）
//
// 单位约定：位置 m，角度 rad；速度 m/s 与 rad/s，加速度 m/s² 与 rad/s²。
// 所有量都表达在**基座坐标系**里，本模块完全不涉及关节与机构。

#ifndef KDL_INTERPOLATION__KDL_CARTESIAN_HPP_
#define KDL_INTERPOLATION__KDL_CARTESIAN_HPP_

#include <array>
#include <string>
#include <vector>

#include <kdl/frames.hpp>

#include "kdl_quintic.hpp"

namespace kdl_interpolation
{

// ---------------------------------------------------------------------------
// 一、单段：系数与求值
// ---------------------------------------------------------------------------

/**
 * @brief 五次"最小 jerk"剖面 s(τ) = 10τ³ − 15τ⁴ + 6τ⁵ 及其一、二阶导数。
 * @param tau [in]  归一化时刻 τ ∈ [0,1]（τ = t/T）。
 * @param s   [out] 角参数 s(τ)，取值 0 → 1。
 * @param ds  [out] s 对 τ 的一阶导 s'(τ)。
 * @param dds [out] s 对 τ 的二阶导 s''(τ)。
 *
 * @note 位置那一侧它就是"两端 v = a = 0 的五次多项式"的形状；姿态这一侧它是
 *       角参数。两条曲线共用同一个 s，这正是位置与姿态能"同起同停"的原因。
 * @note 导数关系（本模块内部靠它把"对 τ 的导数"换算成"对 t 的导数"）：
 *       ω = (θ/T)·s'(τ)、α = (θ/T²)·s''(τ)。
 * @note 三阶、四阶导也可直接写出，但它们只影响 jerk/snap，本模块不做笛卡尔
 *       jerk 校验，需要时可以自行求导：s''' = 60 − 360τ + 360τ²，
 *       s'''' = −360 + 720τ。
 */
void quinticSmoothStep(double tau, double & s, double & ds, double & dds);

/**
 * @brief 一段 slerp 姿态插值的全部数据（时间同样归一化为 τ = t/T）。
 *
 * @note 姿态这一侧没有"系数"可存：绕固定轴转的旋转由"起点姿态 + 转轴 + 转角"
 *       三个量唯一确定。存的是**基座坐标系下的转轴**（已由起点姿态搬过来），
 *       这样求角速度时不必再乘一次 R(τ)，见文件头第五节。
 */
struct SlerpSegment
{
  /// 起点姿态 R0（相对基座坐标系）。
  KDL::Rotation start_rotation;

  /// 转轴，**单位向量**，已表达在基座坐标系中（左乘：R(τ) = Rot(axis_base, θ·s(τ))·R0）；
  /// 零向量表示该段没有姿态变化。
  KDL::Vector axis_base;

  /// 绕 axis_base 的转角 θ ∈ [0, π]，单位 rad（走最短弧）。
  double angle = 0.0;

  /// 该段时长 T，单位 s（必须 > 0）。
  double duration = 0.0;

  /**
   * @brief 该段是否真的在转。
   * @return θ > 0 时返回 true。
   *
   * @note 起点与终点姿态完全相同时 θ = 0，此时 axis_base 是零向量，
   *       求值会退化成"姿态恒等于起点"，这是被显式支持的正常情形。
   */
  bool rotating() const { return angle > 0.0; }
};

/**
 * @brief 笛卡尔状态：位姿 + 线/角速度 + 线/角加速度。
 *
 * @note 四项速度/加速度都表达在基座坐标系；"线"是矢量本身的导数，"角"是
 *       角速度矢量（方向即瞬时转轴，正负按右手定则）。
 * @note 之所以把五个量打包返回，理由与 QuinticTrajectory::sample() 相同：
 *       控制器每个周期本来就要它们齐备，分开取只是多写几行代码。
 */
struct CartesianState
{
  /// 末端位姿：位置单位 m，姿态相对基座坐标系。
  KDL::Frame pose;

  /// 线速度 v，单位 m/s（基座系）。
  KDL::Vector linear_velocity;

  /// 角速度 ω，单位 rad/s（基座系）。
  KDL::Vector angular_velocity;

  /// 线加速度 a，单位 m/s²（基座系）。
  KDL::Vector linear_acceleration;

  /// 角加速度 α，单位 rad/s²（基座系）。
  KDL::Vector angular_acceleration;
};

/**
 * @brief 由起止位姿与段时长构造**单段**"位置五次 + 姿态 slerp"插值。
 * @param start       [in]  起点位姿（位置 m、姿态相对基座）。
 * @param goal        [in]  终点位姿。
 * @param duration    [in]  段时长 T，单位 s，必须 > 0。
 * @param position    [out] 位置的三组五次系数，下标 0/1/2 依次对应 x/y/z；
 *                          里面装的是**归一化到 τ** 的系数，且各自带着 T。
 * @param orientation [out] 姿态的 slerp 数据（起点姿态、转轴、转角、T）。
 * @return true 表示成功；false 表示 duration <= 0，此时输出不被修改。
 *
 * @note 这是本模块的"纯数学"入口，和 kdl_quintic 的 computeQuinticCoefficients()
 *       地位相同：不含任何限位、可行性、路点概念，方便课堂上单独推导。
 * @note 两种边界都做成"静止起步、静止停住"：
 *       位置取 v0 = v1 = a0 = a1 = 0；姿态取 s'(0) = s'(1) = s''(0) = s''(1) = 0
 *       （由 quinticSmoothStep 天然满足，不需要额外条件）。
 * @note 起止位置相同时位置系数仍会正常算出（该分量恒定）；起止姿态相同时
 *       θ = 0、axis_base 为零向量，见文件头第六节。
 */
bool computeCartesianSegment(
  const KDL::Frame & start, const KDL::Frame & goal, double duration,
  std::array<QuinticCoefficients, 3> & position, SlerpSegment & orientation);

/**
 * @brief 在归一化时刻 τ 处求单段的位置、姿态及其一、二阶导数。
 * @param position    [in]  位置系数，须由 computeCartesianSegment() 得到。
 * @param orientation [in]  姿态数据，须由 computeCartesianSegment() 得到。
 * @param tau         [in]  归一化时刻 ∈ [0,1]（τ = t/T）。
 * @param state       [out] 采样结果（位姿、线/角速度、线/角加速度）。
 * @return true 表示成功；false 表示段时长非法（duration <= 0）。
 *
 * @note 本函数不检查 τ 的范围：越界外插在数学上有定义，只是不再代表那条
 *       "两端停住"的轨迹。多段轨迹的 CartesianTrajectory::sample() 才会 clamp。
 * @note 换算关系（本函数内部负责，调用者拿到的一律是含时间单位的物理量）：
 *       ṗ = p'/T、p̈ = p''/T²，而姿态部分是解析解：
 *         R(τ) = Rot(axis_base, θ·s(τ)) · R0
 *         ω(τ) = (θ/T)·s'(τ)·axis_base     —— 方向恒定，因为绕固定轴转
 *         α(τ) = (θ/T²)·s''(τ)·axis_base
 */
bool evaluateCartesianSegment(
  const std::array<QuinticCoefficients, 3> & position, const SlerpSegment & orientation, double tau,
  CartesianState & state);

// ---------------------------------------------------------------------------
// 二、笛卡尔约束（用于判断轨迹是否可行）
// ---------------------------------------------------------------------------

/**
 * @brief 末端线/角速度、线/角加速度的峰值上限。
 *
 * @note 四项彼此独立：某一项 <= 0 就表示"不校验这一项"。默认构造出来的四项
 *       全是 0，即**完全不做校验**——这样单段纯插值演示不会被限位打扰，
 *       与 JointLimits 用"长度 0 表示不校验"是同一个思路。
 * @note 只看**矢量模长**（合速度、合角速度），不像关节限位那样逐关节判定：
 *       笛卡尔空间本来就是一个 6 维整体量，没有"第几个轴"的概念。
 * @note 单位：线速度 m/s、线加速度 m/s²、角速度 rad/s、角加速度 rad/s²。
 */
struct CartesianLimits
{
  /// |v| 上限，单位 m/s；<= 0 表示不校验。
  double max_linear_velocity = 0.0;

  /// |a| 上限，单位 m/s²；<= 0 表示不校验。
  double max_linear_acceleration = 0.0;

  /// |ω| 上限，单位 rad/s；<= 0 表示不校验。
  double max_angular_velocity = 0.0;

  /// |α| 上限，单位 rad/s²；<= 0 表示不校验。
  double max_angular_acceleration = 0.0;

  /// @return 是否已设置线速度上限。
  bool check_linear_velocity() const { return max_linear_velocity > 0.0; }

  /// @return 是否已设置线加速度上限。
  bool check_linear_acceleration() const { return max_linear_acceleration > 0.0; }

  /// @return 是否已设置角速度上限。
  bool check_angular_velocity() const { return max_angular_velocity > 0.0; }

  /// @return 是否已设置角加速度上限。
  bool check_angular_acceleration() const { return max_angular_acceleration > 0.0; }
};

// ---------------------------------------------------------------------------
// 三、多段轨迹
// ---------------------------------------------------------------------------

/**
 * @brief 多段轨迹在**内部路点**处的行为（只影响位置，姿态两种模式都停住）。
 *
 * @note 姿态为什么两种模式都必须停住，见文件头第五节——这是 slerp 段"绕固定轴
 *       转"带来的硬约束，不是可以调的参数。
 */
enum class WaypointBehavior
{
  /// 位置每段两端 v = a = 0：每个路点都精确停住，与姿态行为一致。
  kStop,

  /// 位置复用 kdl_quintic 的 C⁴ 全局拼接：路过内部路点不减速，但姿态仍停住。
  kPassThrough
};

struct CartesianResult;

/**
 * @brief 笛卡尔位姿轨迹：路点 + 段时长 → 可采样、可校验的多段轨迹。
 *
 * @note 与 QuinticTrajectory 同样的约定：本类只做"存储 + 采样 + 校验"，
 *       不允许外部随意捏造内部状态，唯一正规的产生方式是
 *       buildCartesianTrajectory()。
 */
class CartesianTrajectory
{
public:
  CartesianTrajectory() = default;

  /**
   * @brief 轨迹是否可用。
   * @return 构建成功时返回 true。
   */
  bool valid() const { return !position_.empty(); }

  /**
   * @brief 段数 m（路点数为 m + 1）。
   * @return 段数；轨迹无效时返回 0。
   */
  unsigned int segmentCount() const { return static_cast<unsigned int>(position_.size()); }

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
   * @brief 取构建时选择的路点行为。
   * @return kStop 或 kPassThrough。
   */
  WaypointBehavior waypointBehavior() const { return waypoint_behavior_; }

  /**
   * @brief 取路点位姿序列（含首尾）。
   * @return 长度 (段数 + 1) 的数组；位置单位 m，姿态相对基座。
   */
  const std::vector<KDL::Frame> & waypoints() const { return waypoints_; }

  /**
   * @brief 取各路点对应的时刻。
   * @return 长度 (段数 + 1) 的数组，单位 s，首元素为 0。
   */
  const std::vector<double> & knotTimes() const { return knot_times_; }

  /**
   * @brief 取第 segment 段的位置系数。
   * @param segment [in] 段序号，从 0 开始。
   * @return 长度 3 的数组，下标 0/1/2 依次是 x/y/z 的系数；越界时返回空系数
   *         （duration = 0，求值会失败）。
   */
  const std::array<QuinticCoefficients, 3> & positionCoefficients(unsigned int segment) const;

  /**
   * @brief 取第 segment 段的姿态 slerp 数据。
   * @param segment [in] 段序号，从 0 开始。
   * @return 该段的 SlerpSegment；越界时返回空段（duration = 0）。
   */
  const SlerpSegment & orientationSegment(unsigned int segment) const;

  /**
   * @brief 在时刻 t 采样轨迹。
   * @param t     [in]  时间，单位 s。超出 [0, 总时长] 时**自动 clamp** 到端点，
   *                    即 t<0 返回起点位姿、t>总时长 返回终点位姿，不报错。
   * @param state [out] 采样结果：位姿 + 线/角速度 + 线/角加速度（单位见
   *                    CartesianState 的注释）。
   * @return true 表示成功；false 表示轨迹无效（此时输出不被修改）。
   */
  bool sample(double t, CartesianState & state) const;

  /**
   * @brief 校验整条轨迹是否满足笛卡尔约束。
   * @param limits  [in]  约束；未设置的项自动跳过。
   * @param message [out] 失败时填入"第几段、哪一项、峰值多少、上限多少"的可读
   *                      说明；成功时清空。
   * @return true 表示全部满足（或没有任何约束需要检查）。
   *
   * @note 判定方式：逐段按固定步长采样（kLimitCheckSamples 个点，含两端）取峰值，
   *       属于**工程近似**。这一点必须讲清楚，因为本模块的两侧精度并不相同：
   *       - 姿态侧其实有解析峰值：|ω|max = 15θ/(8T)、|α|max = 10θ/(√3 T²)，
   *         采样只是用来复核它们；
   *       - 位置侧在 kStop 模式下同样有解析峰值（见文件头第二节），但在
   *         kPassThrough 模式下位置是多段拼接出来的，峰值没有闭式解。
   *       为了不让"同一个函数里两套判定逻辑"增加阅读负担，这里统一走采样，
   *       并在示例里把"采样峰值 vs 解析峰值"并排打出来，让误差看得见。
   */
  bool validateLimits(const CartesianLimits & limits, std::string & message) const;

private:
  friend CartesianResult buildCartesianTrajectory(
    const std::vector<KDL::Frame> &, const std::vector<double> &, WaypointBehavior,
    const CartesianLimits &);

  /**
   * @brief 把时刻 t 定位到具体段与归一化时刻 τ。
   * @param t     [in]  时间，单位 s（内部先 clamp 到 [0, 总时长]）。
   * @param index [out] 段序号。
   * @param tau   [out] 归一化时刻 ∈ [0,1]。
   * @return true 表示轨迹有效、定位成功。
   */
  bool locateSegment(double t, unsigned int & index, double & tau) const;

  /// 内部路点处的行为（只影响位置）。
  WaypointBehavior waypoint_behavior_ = WaypointBehavior::kStop;

  /// 路点位姿（相对基座），长度 = 段数 + 1。
  std::vector<KDL::Frame> waypoints_;

  /// 各段时长 [s]，长度 = 段数。
  std::vector<double> durations_;

  /// 各路点时刻 [s]，长度 = 段数 + 1，首元素为 0。
  std::vector<double> knot_times_;

  /// 各段的位置系数：position_[段][0/1/2] = x/y/z。
  std::vector<std::array<QuinticCoefficients, 3>> position_;

  /// 各段的姿态 slerp 数据，长度 = 段数。
  std::vector<SlerpSegment> orientation_;
};

// ---------------------------------------------------------------------------
// 四、构建入口
// ---------------------------------------------------------------------------

/**
 * @brief 轨迹构建结果。
 *
 * @note 与 kdl_kinematics 的 VelocityResult、kdl_quintic 的 TrajectoryResult
 *       同样的思路：把"能不能用"和"为什么不能用"一起交给调用者，
 *       避免用异常打断实时控制循环。
 */
struct CartesianResult
{
  /// 构建成功时可直接使用的轨迹；失败时为一具空壳（valid() == false）。
  CartesianTrajectory trajectory;

  /// 是否构建成功。
  bool success = false;

  /// 失败原因（中文、可直接打印）：输入非法、位置拼接失败、或哪一段哪一项超限。
  std::string message;
};

/**
 * @brief 由路点与段时长构建笛卡尔位姿轨迹（位置五次 + 姿态 slerp）。
 * @param waypoints [in] 路点位姿，长度 ≥ 2；位置单位 m，姿态相对基座。
 * @param durations [in] 各段时长，长度须 = waypoints.size() − 1，每个元素 > 0，单位 s。
 * @param waypoint_behavior [in] 内部路点处的行为，默认 kStop（见 WaypointBehavior）。
 * @param limits    [in] 笛卡尔约束；默认构造为"不校验"。
 * @return CartesianResult；success 为 false 时 message 说明原因。
 *
 * @note 输入校验、逐段组装、约束校验的次序与 buildQuinticTrajectory() 一致：
 *       先把输入查干净，后面就可以假定数据是规整的。
 * @note 失败一律**不 clamp、不自动拉长时间**：超限就如实判为不可行，由调用者
 *       自己决定是放慢、改路点还是换方案——"悄悄改慢"会让实际执行的时间和
 *       调用者以为的时间不一致，是更隐蔽的错。
 * @note 构建完会立即调用一次 validateLimits()，所以 success == true 就意味着
 *       这条轨迹在给定约束下是可行的。
 */
CartesianResult buildCartesianTrajectory(
  const std::vector<KDL::Frame> & waypoints, const std::vector<double> & durations,
  WaypointBehavior waypoint_behavior = WaypointBehavior::kStop,
  const CartesianLimits & limits = CartesianLimits());

}  // namespace kdl_interpolation

#endif  // KDL_INTERPOLATION__KDL_CARTESIAN_HPP_
