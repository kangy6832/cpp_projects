// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：速度级映射——关节速度与末端速度之间的相互转换。
//
// 速度级关系是全部机器人运动学的"线性化"核心，只有一行公式：
//
//     [ v ; ω ] = J(q) · q̇          （v 线速度，ω 角速度）
//
// 其中：
//   - 正向（关节 → 末端）：直接乘雅可比，永远是线性、无迭代、必定成功；
//   - 反向（末端 → 关节）：需要求 J 的"逆"，但 J 通常是 6 x n 的长方阵，
//     所以要用伪逆 J⁺。此时才会遇到真正麻烦的问题——奇异位形：
//     J 掉秩时 J⁺ 会发散（关节速度趋于无穷），必须靠截断或阻尼来救。
//
// 本模块把这两件事拆成两类函数，并统一返回带 σ_min 的结果结构，
// 让"离奇异有多近"变成可以打印、可以比较的数字。

#ifndef KDL_KINEMATICS__KDL_VELOCITY_HPP_
#define KDL_KINEMATICS__KDL_VELOCITY_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace kdl_kinematics
{

// ---------------------------------------------------------------------------
// 一、共享的数据结构
// ---------------------------------------------------------------------------

/**
 * @brief 把 6 维 Twist 拆成"线分量 + 角分量"的可读形式。
 *
 * @note KDL 用一个 Twist 同时装平移与旋转两种量，这在计算上很方便，
 *       但打印和阅读时容易看混，所以这里提供一个轻量视图。
 * @note 速度与加速度共用这个结构：线性字段的单位由上下文决定
 *       （速度 m/s，加速度 m/s²；角分量则 rad/s 或 rad/s²）。
 */
struct CartesianVector
{
  KDL::Vector linear;    ///< 线分量：平移速度(m/s) 或 平移加速度(m/s²)
  KDL::Vector angular;   ///< 角分量：旋转速度(rad/s) 或 旋转加速度(rad/s²)
};

/**
 * @brief 把 KDL::Twist 拆成线/角两部分。
 * @param twist [in] 待拆分的 6 维量（速度或加速度均可）。
 * @return 拆分后的可读结构。
 */
CartesianVector splitTwist(const KDL::Twist & twist);

/**
 * @brief 把线/角两部分合并回 KDL::Twist。
 * @param cartesian [in] 待合并的线/角结构。
 * @return 合并后的 6 维量。
 */
KDL::Twist mergeTwist(const CartesianVector & cartesian);

// ---------------------------------------------------------------------------
// 二、雅可比的"病态程度"指标
// ---------------------------------------------------------------------------

/**
 * @brief 计算雅可比 J(q) 的最小奇异值 σ_min。
 * @param chain [in] 运动学链。
 * @param q     [in] 关节角，单位 rad（平移关节为 m）；长度须等于链的自由度。
 * @return σ_min；若求解失败返回 -1。
 *
 * @note σ_min 是量化"离奇异位形有多近"的标准指标：越接近 0 越接近奇异。
 *       它同时也是伪逆 J⁺ 的放大倍数上界——σ_min 很小时，
 *       末端一个很小的速度指令会被放大成巨大的关节速度。
 * @note 本函数用 Eigen 的 SVD 直接对 J 分解。KDL 的 wdls 求解器内部也维护了
 *       同样的量（getSigmaMin()），但 pinv 没有暴露，为了让两条路径口径一致，
 *       这里统一自己算。
 */
double minSingularValue(const KDL::Chain & chain, const KDL::JntArray & q);

// ---------------------------------------------------------------------------
// 三、正向：关节速度 -> 末端速度
// ---------------------------------------------------------------------------

/**
 * @brief 关节速度 → 末端速度：twist = J(q) · q̇。
 * @param chain [in]  运动学链。
 * @param q     [in]  关节角，单位 rad（平移关节为 m）。
 * @param qdot  [in]  关节速度，单位 rad/s（平移关节为 m/s）。
 * @param twist [out] 末端速度，Twist 形式：
 *                    线分量单位 m/s，角分量单位 rad/s；
 *                    表达在基座坐标系，参考点在末端。
 * @return true 表示计算成功；false 表示尺寸不匹配。
 *
 * @note 这是纯线性映射，不存在不收敛、不唯一的问题，永远一步到位。
 * @note 与 kdl_ik.hpp 里的 FK 对应：那里算"位置"，这里算"速度"。
 */
bool jointToCartesianVel(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  KDL::Twist & twist);

// ---------------------------------------------------------------------------
// 四、反向：末端速度 -> 关节速度（需要伪逆）
// ---------------------------------------------------------------------------

/**
 * @brief 速度级逆映射的结果（两种求解器共用，便于课堂横向对比）。
 */
struct VelocityResult
{
  KDL::JntArray qdot;         ///< 解出的关节速度，单位 rad/s（平移关节为 m/s）
  int error_code = 0;         ///< KDL 错误码：0=正常，+100=收敛但伪逆奇异，负数=失败
  std::string message;        ///< 错误码对应的可读说明
  double sigma_min = 0.0;     ///< 当前位形下 J(q) 的最小奇异值

  /**
   * @brief 求解是否可用。
   * @return error_code >= 0 时返回 true。
   *
   * @note KDL 约定：0=无错误，正数=有降级的警告，负数=失败。
   *       所以 +100（伪逆奇异）算"可用但有警告"，仍返回 true。
   */
  bool success() const { return error_code >= 0; }

  /**
   * @brief 是否出现奇异警告（error_code > 0）。
   * @return 出现警告时返回 true；此时解虽然给出了，但关节速度可能已经不可信。
   */
  bool singular() const { return error_code > 0; }
};

/**
 * @brief 末端速度 → 关节速度：q̇ = pinv(J(q)) · twist（纯伪逆）。
 * @param chain [in]  运动学链。
 * @param q     [in]  当前关节角，单位 rad（平移关节为 m）。
 * @param twist [in]  期望的末端速度：线分量 m/s，角分量 rad/s。
 * @return VelocityResult，其中 qdot 为关节速度(rad/s)，sigma_min 为 J 的最小奇异值。
 *
 * @note 伪逆的做法是：对 J 做 SVD，把小于阈值 eps 的奇异值直接置零再求逆。
 *       好处是"有确定解时有最短解，无解时给最小二乘解"；
 *       坏处是阈值附近会突变，接近奇异时关节速度可能突然变得很大。
 */
VelocityResult cartesianToJointVel(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::Twist & twist);

/// 阻尼求解器的默认阻尼系数 λ（见 cartesianToJointVelDamped 的说明）。
constexpr double kDefaultDampingLambda = 0.01;

/// 阻尼求解器的默认奇异判定阈值（见 cartesianToJointVelDamped 的 @warning）。
constexpr double kDefaultSingularEps = 1e-2;

/**
 * @brief 末端速度 → 关节速度：q̇ = 阻尼伪逆 · twist（阻尼最小二乘 wdls）。
 * @param chain        [in] 运动学链。
 * @param q            [in] 当前关节角，单位 rad（平移关节为 m）。
 * @param twist        [in] 期望的末端速度：线分量 m/s，角分量 rad/s。
 * @param lambda       [in] 阻尼系数（无量纲）：越大越稳定、但末端跟踪误差越大。
 * @param singular_eps [in] 奇异判定阈值：只有当 σ_min < singular_eps 时，
 *                          求解器才认为位形奇异并真正启用阻尼。
 * @return VelocityResult，其中 qdot 为关节速度(rad/s)，sigma_min 为 J 的最小奇异值。
 *
 * @note 阻尼最小二乘用 (JᵀJ + λ²I)⁻¹Jᵀ 代替 J⁺：在奇异附近不再发散，
 *       代价是牺牲一点精度，是工程上处理奇异位形最常用的折中方案。
 *
 * @warning KDL 的 wdls 有一个很容易踩的坑：它只在**自己判定为奇异**时才施加阻尼，
 *          判据是 σ_min < eps，而 KDL 原生默认 eps = 1e-5 极其严格。
 *          后果是——对"接近奇异但还没到阈值"的位形，wdls 会退化成普通最小二乘，
 *          结果与 pinv 完全一致，看起来像"阻尼根本没生效"。
 *          因此本封装把 eps 显式暴露为 singular_eps，并给了更容易触发的默认值
 *          kDefaultSingularEps(1e-2)。请按你的机器人尺度调整它：
 *          太大则正常工作位形也被阻尼（跟踪变差），太小则接近奇异时它不出手。
 *
 * @note 前三个参数与 cartesianToJointVel 完全相同，可直接对比两者。
 */
VelocityResult cartesianToJointVelDamped(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::Twist & twist,
  double lambda = kDefaultDampingLambda, double singular_eps = kDefaultSingularEps);

}  // namespace kdl_kinematics

#endif  // KDL_KINEMATICS__KDL_VELOCITY_HPP_
