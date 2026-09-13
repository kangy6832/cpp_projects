// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：逆运动学（位置 IK），并排封装两种经典求解器。
//
// 逆运动学回答的问题是正运动的"反问题"：
//
//     已知想让末端到达某个位姿 T_target，各关节应该转多少（q = ?）
//
// 这件事比 FK 难得多，原因是：
//   1) 解可能不存在（目标在工作空间之外）；
//   2) 解可能不唯一（同一个位姿常有多个关节构型）；
//   3) 数值迭代法必须给一个初值 q_init，而从不同初值可能收敛到不同解；
//   4) 6 轴以下/特殊构型下可能无解析解，只能数值逼近。
//
// 本文件封装两种数值求解器，用同一套返回结构，便于课堂对比：
//
//   solveIkLma   -> ChainIkSolverPos_LMA
//                   Levenberg-Marquardt 阻尼最小二乘，自带 FK 与雅可比，
//                   鲁棒、无需额外组件，默认参数即可用。
//
//   solveIkNrJl  -> ChainIkSolverPos_NR_JL
//                   经典牛顿-拉夫逊迭代，并显式考虑关节限位。
//                   但它需要外部再搭两个组件：一个 FK 求解器
//                   （用来算当前位姿差）和一个速度级 IK（ChainIkSolverVel_pinv，
//                    用来把位姿差转成关节增量），因此更能体现"求解器的层次"。

#ifndef KDL_KINEMATICS__KDL_IK_HPP_
#define KDL_KINEMATICS__KDL_IK_HPP_

#include <string>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace kdl_kinematics
{

/**
 * @brief 逆运动学的求解结果（两种求解器共用，因此可以直接横向对比）。
 *
 * @note 即使求解失败，q 里也会保留"迭代到某一步的关节角"，
 *       便于观察算法停在了哪里；是否成功请以 success() 为准。
 */
struct IkResult
{
  KDL::JntArray q;                       ///< 解出的关节角（失败时为迭代中间值）
  int error_code = 0;                    ///< KDL 错误码，0 即 KDL::SolverI::E_NOERROR
  std::string message;                   ///< 错误码对应的可读说明

  /**
   * @brief 是否成功收敛。
   * @return true 表示 error_code == 0。
   *
   * @note KDL 约定：0 = 无错误，正数 = 有降级的警告，负数 = 失败。
   */
  bool success() const { return error_code == 0; }
};

/**
 * @brief 逆运动学：ChainIkSolverPos_LMA（Levenberg-Marquardt）。
 * @param chain   运动学链。
 * @param q_init  迭代初值，长度必须等于 chain.getNrOfJoints()。
 * @param target  期望的末端位姿 T_base_tip（相对基座表达）。
 * @param eps     收敛精度（任务空间误差），默认 1e-5。
 * @param max_iter 最大迭代次数，默认 500。
 * @return IkResult，其中 q 为解出的关节角，message 来自 solver.strError()。
 *
 * @note LMA 自带前方 FK 与雅可比，所以构造函数只需一个 chain，最省事。
 * @note LMA 内部对 6 维误差做了加权（位置权重 1、姿态权重 0.01），
 *       意图是让平移和旋转在量纲上更可比，这也是它比较鲁棒的原因之一。
 */
IkResult solveIkLma(
  const KDL::Chain & chain, const KDL::JntArray & q_init, const KDL::Frame & target,
  double eps = 1e-5, unsigned int max_iter = 500);

/**
 * @brief 逆运动学：ChainIkSolverPos_NR_JL（牛顿-拉夫逊迭代 + 关节限位）。
 * @param chain   运动学链。
 * @param q_init  迭代初值，长度必须等于 chain.getNrOfJoints()。
 * @param target  期望的末端位姿 T_base_tip。
 * @param q_min   每个关节的位置下限，长度必须等于 chain.getNrOfJoints()。
 * @param q_max   每个关节的位置上限，长度必须等于 chain.getNrOfJoints()。
 * @param max_iter 最大迭代次数，默认 100。
 * @param eps     收敛精度，默认 1e-6。
 * @return IkResult，其中 q 为解出的关节角，且保证落在 [q_min, q_max] 内。
 *
 * @note 关节限位是"机器人本体信息"，KDL::Joint 并不保存它，必须由调用方传入。
 *       真实限位应来自 URDF：urdf::Model 中每个 joint 的 <limit lower= upper=>。
 * @note NR_JL 内部依赖一个 ChainFkSolverPos（算当前位姿）与一个 ChainIkSolverVel
 *       （ChainIkSolverVel_pinv，算关节增量），这两个组件在本函数内部构造，
 *       调用方无需关心；想深入理解时请翻看本函数的实现。
 */
IkResult solveIkNrJl(
  const KDL::Chain & chain, const KDL::JntArray & q_init, const KDL::Frame & target,
  const KDL::JntArray & q_min, const KDL::JntArray & q_max,
  unsigned int max_iter = 10000, double eps = 1e-6);

}  // namespace kdl_kinematics

#endif  // KDL_KINEMATICS__KDL_IK_HPP_
