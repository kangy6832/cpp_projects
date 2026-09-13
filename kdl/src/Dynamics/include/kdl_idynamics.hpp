// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：逆动力学——给定运动，求所需的关节力矩/力。
//
// 逆动力学回答的问题是：
//
//     想让机器人以指定的 q、q̇、q̈ 运动，每个关节需要出多大力？
//
// 对于刚性连杆机器人，答案是下面这个"三明治"公式：
//
//     τ = M(q)·q̈  +  C(q,q̇)·q̇  +  G(q)
//         ~~~~~~~     ~~~~~~~~~     ~~~~
//         惯性项       科氏/离心项    重力项
//
// KDL 里对应的求解器是 ChainIdSolver_RNE：用"递归牛顿-欧拉"算法
// （Featherstone 书第 96 页的伪代码）从基座向外推速度/加速度、
// 再从末端向内推力矩，一次遍历即可得到 τ，是机械臂控制里最常用的动力学算法。
//
// 本文件是该模块的"基础层"：kdl_dynparam.hpp 需要用到这里的重力约定，
// kdl_fdynamics.hpp 在实现上也依赖逆动力学的概念。

#ifndef KDL_DYNAMICS__KDL_IDYNAMICS_HPP_
#define KDL_DYNAMICS__KDL_IDYNAMICS_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace kdl_dynamics
{

// ---------------------------------------------------------------------------
// 一、重力约定
// ---------------------------------------------------------------------------

/// 标准重力加速度大小，单位 m/s²。
constexpr double kStandardGravityMagnitude = 9.81;

/**
 * @brief 返回常用的重力向量：基座坐标系下沿 -Z 方向。
 * @return (0, 0, -9.81)，单位 m/s²。
 *
 * @note KDL 的动力学求解器在**构造时**就必须知道重力向量，本模块把它做成
 *       带默认值的函数参数，绝大多数情况调用方不用传。
 * @note 注意重力向量是表达在**基座坐标系**下的。如果机器人被装在倾斜的
 *       基座上，需要自己把重力方向换算过去（例如侧向 45° 安装）。
 */
inline KDL::Vector defaultGravity()
{
  return KDL::Vector(0.0, 0.0, -kStandardGravityMagnitude);
}

// ---------------------------------------------------------------------------
// 二、结果结构
// ---------------------------------------------------------------------------

/**
 * @brief 逆动力学的求解结果。
 */
struct IdResult
{
  KDL::JntArray torque;   ///< 各关节所需力矩/力：旋转关节 N·m，平移关节 N
  int error_code = 0;     ///< KDL 错误码：0=正常，正数=降级警告，负数=失败
  std::string message;    ///< 错误码对应的可读说明

  /**
   * @brief 求解是否可用。
   * @return error_code >= 0 时返回 true。
   */
  bool success() const { return error_code >= 0; }
};

// ---------------------------------------------------------------------------
// 三、逆动力学
// ---------------------------------------------------------------------------

/**
 * @brief 逆动力学（不施加外力）：τ = M(q)·q̈ + C(q,q̇)·q̇ + G(q)。
 * @param chain   [in] 运动学链（由 kdl_tools::buildChain 从 URDF 截取得到，
 *                     链上每个段必须带惯量参数，否则算出来全是 0）。
 * @param q       [in] 关节角，单位 rad（平移关节为 m）。
 * @param qdot    [in] 关节速度，单位 rad/s（平移关节为 m/s）。
 * @param qddot   [in] 关节加速度，单位 rad/s²（平移关节为 m/s²）。
 * @param gravity [in] 重力向量，基座坐标系，单位 m/s²；默认 (0, 0, -9.81)。
 * @return IdResult，其中 torque 为各关节力矩，单位 N·m（平移关节为 N）。
 *
 * @note 三个输入的长度都必须等于 chain.getNrOfJoints()。
 * @note 想验证这个公式，可以把它与 kdl_dynparam.hpp 里的
 *       jointSpaceInertia / coriolisTorque / gravityTorque 拼起来对比，
 *       两者应当逐位一致 —— 这正是示例中的教学闭环。
 */
IdResult inverseDynamics(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Vector & gravity = defaultGravity());

/**
 * @brief 逆动力学（末端施加外力，外力表达在**末端 link 局部坐标系**）。
 * @param chain      [in] 运动学链。
 * @param q          [in] 关节角，单位 rad。
 * @param qdot       [in] 关节速度，单位 rad/s。
 * @param qddot      [in] 关节加速度，单位 rad/s²。
 * @param tip_wrench [in] 末端受到的外力/力矩，表达在**末端 link 自身的坐标系**，
 *                        参考点在该坐标系原点；力单位 N，力矩单位 N·m。
 * @param gravity    [in] 重力向量，基座坐标系，单位 m/s²。
 * @return IdResult，其中 torque 为各关节力矩，单位 N·m。
 *
 * @note 这是 KDL 的原生口径：ChainIdSolver_RNE 要求 f_ext 中每一项都表达在
 *       **对应段自己的局部坐标系**里，本函数只填充最后一段（末端），其余置零。
 * @note 局部坐标系会随关节角转动，所以同一个物理力，在不同位形下写出来的
 *       分量是不同的。如果觉得别扭，请用下面的 ...Base 版本。
 */
IdResult inverseDynamicsWithTipWrenchLocal(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Wrench & tip_wrench,
  const KDL::Vector & gravity = defaultGravity());

/**
 * @brief 逆动力学（末端施加外力，外力表达在**基座坐标系**）。
 * @param chain           [in] 运动学链。
 * @param q               [in] 关节角，单位 rad。
 * @param qdot            [in] 关节速度，单位 rad/s。
 * @param qddot           [in] 关节加速度，单位 rad/s²。
 * @param tip_wrench_base [in] 末端受到的外力/力矩，表达在**基座坐标系**，
 *                             参考点在末端；力单位 N，力矩单位 N·m。
 * @param gravity         [in] 重力向量，基座坐标系，单位 m/s²。
 * @return IdResult，其中 torque 为各关节力矩，单位 N·m。
 *
 * @note 这个版本更符合直觉："沿世界 Z 轴向下压 10 N"可以直接写成
 *       Wrench(Vector(0, 0, -10), Vector::Zero())，不必关心末端此刻的朝向。
 * @note 内部会先做一次正运动学求出末端姿态，再把力旋量从基座系转到末端局部系，
 *       最后转交给 inverseDynamicsWithTipWrenchLocal()。
 *       这也说明了运动学与动力学的接口在哪里：**要知道末端姿态**。
 */
IdResult inverseDynamicsWithTipWrenchBase(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Wrench & tip_wrench_base,
  const KDL::Vector & gravity = defaultGravity());

}  // namespace kdl_dynamics

#endif  // KDL_DYNAMICS__KDL_IDYNAMICS_HPP_
