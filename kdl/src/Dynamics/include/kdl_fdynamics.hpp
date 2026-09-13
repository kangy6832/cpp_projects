// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：正动力学——给定关节力矩，求机器人会产生多大的加速度。
//
// 正动力学是逆动力学的"反问题"：
//
//     已知 τ、q、q̇  ->  q̈ = ?
//
// 把逆动力学的三明治公式移项就得到它的数学形式：
//
//     q̈ = M(q)⁻¹ · [ τ − C(q,q̇)·q̇ − G(q) ]
//
// 也正因为要"除以"惯性矩阵，正动力学比逆动力学更贵：需要求解一个线性方程组。
// KDL 里对应的求解器是 ChainFdSolver_RNE，它内部正是这么做的
// （先用 ChainDynParam 得到 M 与"零加速度力矩"，再用 Cholesky 分解求解）。
//
// 正动力学最重要的用途是**仿真**：给定时矩指令，把 q̈ 积分成 q、q̇，
// 就能在没有真实硬件的情况下预测机器人的运动。示例里用 KDL 自带的
// RK4Integrator 演示了这一点。

#ifndef KDL_DYNAMICS__KDL_FDYNAMICS_HPP_
#define KDL_DYNAMICS__KDL_FDYNAMICS_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_idynamics.hpp"

namespace kdl_dynamics
{

// ---------------------------------------------------------------------------
// 结果结构
// ---------------------------------------------------------------------------

/**
 * @brief 正动力学的求解结果。
 */
struct FdResult
{
  KDL::JntArray qddot;    ///< 各关节加速度，单位 rad/s²（平移关节为 m/s²）
  int error_code = 0;     ///< KDL 错误码：0=正常，正数=降级警告，负数=失败
  std::string message;    ///< 错误码对应的可读说明

  /// @return error_code >= 0 时返回 true。
  bool success() const { return error_code >= 0; }
};

// ---------------------------------------------------------------------------
// 正动力学
// ---------------------------------------------------------------------------

/**
 * @brief 正动力学（不施加外力）：q̈ = M(q)⁻¹·[ τ − C(q,q̇)·q̇ − G(q) ]。
 * @param chain   [in] 运动学链。
 * @param q       [in] 关节角，单位 rad（平移关节为 m）。
 * @param qdot    [in] 关节速度，单位 rad/s（平移关节为 m/s）。
 * @param torque  [in] 施加在各关节上的力矩/力，单位 N·m（平移关节为 N）。
 * @param gravity [in] 重力向量，基座坐标系，单位 m/s²；默认 (0, 0, -9.81)。
 * @return FdResult，其中 qddot 为各关节加速度，单位 rad/s²。
 *
 * @note 三个输入的长度都必须等于 chain.getNrOfJoints()。
 * @note 与 inverseDynamics() 互为逆运算：
 *       对同一组 (q, q̇)，先 ID 得到 τ、再 FD 回到 q̈，应当与原 q̈ 一致。
 *       示例中会做这个往返闭合验证。
 */
FdResult forwardDynamics(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque, const KDL::Vector & gravity = defaultGravity());

/**
 * @brief 正动力学（末端施加外力，外力表达在**末端 link 局部坐标系**）。
 * @param chain      [in] 运动学链。
 * @param q          [in] 关节角，单位 rad。
 * @param qdot       [in] 关节速度，单位 rad/s。
 * @param torque     [in] 关节力矩，单位 N·m。
 * @param tip_wrench [in] 末端受到的外力/力矩，表达在**末端 link 自身的坐标系**，
 *                        参考点在该坐标系原点；力单位 N，力矩单位 N·m。
 * @param gravity    [in] 重力向量，基座坐标系，单位 m/s²。
 * @return FdResult，其中 qddot 为各关节加速度，单位 rad/s²。
 *
 * @note 坐标系口径与 inverseDynamicsWithTipWrenchLocal() 完全一致，
 *       两个函数的 Wrench 可以直接互换着用。
 */
FdResult forwardDynamicsWithTipWrenchLocal(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque, const KDL::Wrench & tip_wrench,
  const KDL::Vector & gravity = defaultGravity());

/**
 * @brief 正动力学（末端施加外力，外力表达在**基座坐标系**）。
 * @param chain           [in] 运动学链。
 * @param q               [in] 关节角，单位 rad。
 * @param qdot            [in] 关节速度，单位 rad/s。
 * @param torque          [in] 关节力矩，单位 N·m。
 * @param tip_wrench_base [in] 末端受到的外力/力矩，表达在**基座坐标系**，
 *                             参考点在末端；力单位 N，力矩单位 N·m。
 * @param gravity         [in] 重力向量，基座坐标系，单位 m/s²。
 * @return FdResult，其中 qddot 为各关节加速度，单位 rad/s²。
 *
 * @note 内部同样需要先做一次正运动学拿到末端姿态，才能把力旋量转进局部系。
 */
FdResult forwardDynamicsWithTipWrenchBase(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque, const KDL::Wrench & tip_wrench_base,
  const KDL::Vector & gravity = defaultGravity());

}  // namespace kdl_dynamics

#endif  // KDL_DYNAMICS__KDL_FDYNAMICS_HPP_
