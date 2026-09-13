// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：加速度级映射——关节加速度与末端加速度之间的相互转换。
//
// 加速度级关系比速度级多出一项，这是本模块最核心的知识点：
//
//     a = J(q)·q̈  +  J̇(q,q̇)·q̇
//         ~~~~~~~     ~~~~~~~~~~
//         关节加速度   科氏 / 离心项（因为 J 本身随 q 变化而产生）
//         直接贡献
//
// 换句话说："末端有加速度"不一定是"关节在加速"，也可能是"关节在匀速运动，
// 但 J 在变化"造成的。忽略第二项只在 q̇ ≈ 0 时才勉强成立。
//
// 重要提示：KDL 只提供了抽象基类 ChainFkSolverAcc / ChainIkSolverAcc，
// 并没有任何可用的具体实现，所以本模块是"手工组合"出来的：
//   - J     来自 kdl_jacobian.hpp 的 computeJacobian
//   - J̇·q̇   来自 KDL 的 ChainJntToJacDotSolver
//   - 求逆   复用速度级的伪逆（见下方反向函数的说明）
//
// 因为速度与加速度共用同一种 6 维容器（KDL::Twist）与同一种"线性映射 + 求逆"
// 结构，所以本头文件包含 kdl_velocity.hpp，直接复用其中的
// CartesianVector（可读视图）与 minSingularValue（奇异程度指标）。
// splitTwist() 对加速度同样适用——它只是把 Twist 拆成线/角两行而已。

#ifndef KDL_KINEMATICS__KDL_ACCELERATION_HPP_
#define KDL_KINEMATICS__KDL_ACCELERATION_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_velocity.hpp"

namespace kdl_kinematics
{

// ---------------------------------------------------------------------------
// 一、核心中间项 J̇·q̇
// ---------------------------------------------------------------------------

/**
 * @brief 计算雅可比时间导数与关节速度的乘积 J̇(q,q̇)·q̇。
 * @param chain [in]  运动学链。
 * @param q     [in]  关节角，单位 rad（平移关节为 m）。
 * @param qdot  [in]  关节速度，单位 rad/s（平移关节为 m/s）。
 * @param jac_dot_q_dot [out] 结果，Twist 形式：线分量 m/s²，角分量 rad/s²。
 * @return true 表示计算成功；false 表示尺寸不匹配或求解器报错。
 *
 * @note 这一项也叫"科氏/离心加速度项"。它之所以存在，是因为雅可比依赖位形 q，
 *       当关节在运动时 J 本身也在随时间变化，于是同一时刻的末端速度不再守恒。
 * @note 这里用 KDL 的 ChainJntToJacDotSolver，并显式指定 HYBRID 表示
 *       （参考坐标系=基座，参考点=末端），好与 computeJacobian 的口径一致；
 *       换成 BODYFIXED / INERTIAL 会得到不同表达下的结果，不能混用。
 */
bool jacobianDotTimesQdot(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  KDL::Twist & jac_dot_q_dot);

// ---------------------------------------------------------------------------
// 二、正向：关节加速度 -> 末端加速度
// ---------------------------------------------------------------------------

/**
 * @brief 关节加速度 → 末端加速度：a = J(q)·q̈ + J̇(q,q̇)·q̇。
 * @param chain        [in]  运动学链。
 * @param q            [in]  关节角，单位 rad（平移关节为 m）。
 * @param qdot         [in]  关节速度，单位 rad/s（平移关节为 m/s）。
 * @param qddot        [in]  关节加速度，单位 rad/s²（平移关节为 m/s²）。
 * @param acceleration [out] 末端加速度，Twist 形式：
 *                           线分量 m/s²，角分量 rad/s²；
 *                           表达在基座坐标系，参考点在末端。
 * @return true 表示计算成功；false 表示尺寸不匹配或中间求解失败。
 *
 * @note 必须同时给出 q、q̇、q̈：加速度是"二阶"信息，只知道 q̈ 是不够的，
 *       因为 J̇·q̇ 那一项还依赖 q 和 q̇。
 */
bool jointToCartesianAcc(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, KDL::Twist & acceleration);

// ---------------------------------------------------------------------------
// 三、反向：末端加速度 -> 关节加速度
// ---------------------------------------------------------------------------

/**
 * @brief 加速度级逆映射的结果（两种求解器共用）。
 */
struct AccelerationResult
{
  KDL::JntArray qddot;        ///< 解出的关节加速度，单位 rad/s²（平移关节为 m/s²）
  int error_code = 0;         ///< KDL 错误码：0=正常，+100=收敛但伪逆奇异，负数=失败
  std::string message;        ///< 错误码对应的可读说明
  double sigma_min = 0.0;     ///< 当前位形下 J(q) 的最小奇异值

  /**
   * @brief 求解是否可用。
   * @return error_code >= 0 时返回 true（含 +100 的奇异警告）。
   */
  bool success() const { return error_code >= 0; }

  /**
   * @brief 是否出现奇异警告（error_code > 0）。
   * @return 出现警告时返回 true。
   */
  bool singular() const { return error_code > 0; }
};

/**
 * @brief 末端加速度 → 关节加速度：q̈ = pinv(J(q)) · (a − J̇(q,q̇)·q̇)。
 * @param chain        [in]  运动学链。
 * @param q            [in]  当前关节角，单位 rad（平移关节为 m）。
 * @param qdot         [in]  当前关节速度，单位 rad/s（平移关节为 m/s）。
 * @param acceleration [in]  期望的末端加速度：线分量 m/s²，角分量 rad/s²。
 * @return AccelerationResult，其中 qddot 为关节加速度(rad/s²)。
 *
 * @note 关键理解：先把科氏项减掉，剩下的 r = a − J̇·q̇ 与 q̈ 之间就恢复成
 *       纯粹的线性关系 r = J·q̈，于是"求加速度逆解"和"求速度逆解"变成了
 *       同一个线性方程组。这也是为什么本函数可以直接复用速度级的伪逆求解器：
 *       它只负责解 J·x = r，x 到底是速度还是加速度，由输入 r 的量纲决定。
 * @note 因此加速度逆解同样会遭遇奇异位形，处理方式与速度级完全一致。
 */
AccelerationResult cartesianToJointAcc(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::Twist & acceleration);

/**
 * @brief 末端加速度 → 关节加速度：用阻尼最小二乘（wdls）代替纯伪逆。
 * @param chain        [in] 运动学链。
 * @param q            [in] 当前关节角，单位 rad（平移关节为 m）。
 * @param qdot         [in] 当前关节速度，单位 rad/s（平移关节为 m/s）。
 * @param acceleration [in] 期望的末端加速度：线分量 m/s²，角分量 rad/s²。
 * @param lambda       [in] 阻尼系数（无量纲）。
 * @param singular_eps [in] 奇异判定阈值：只有 σ_min < singular_eps 时才真正启用阻尼。
 * @return AccelerationResult，其中 qddot 为关节加速度(rad/s²)。
 *
 * @note 与 cartesianToJointAcc 的差别只在"怎么求逆"，前四个参数完全相同，
 *       可直接用于对比奇异位形下的稳定性。
 * @warning 与速度级同样的坑：KDL 的 wdls 只在自判奇异时才阻尼，
 *          阈值由 eps 控制。详见 kdl_velocity.hpp 中
 *          cartesianToJointVelDamped 的 @warning。
 */
AccelerationResult cartesianToJointAccDamped(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::Twist & acceleration, double lambda = kDefaultDampingLambda,
  double singular_eps = kDefaultSingularEps);

}  // namespace kdl_kinematics

#endif  // KDL_KINEMATICS__KDL_ACCELERATION_HPP_
