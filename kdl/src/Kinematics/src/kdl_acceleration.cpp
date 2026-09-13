// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：kdl_acceleration.hpp 中声明的函数在这里落地。
//
// 本文件是整个模块里最"手工"的部分：KDL 没有可用的加速度求解器，
// 所以加速度关系被拆成三个可复用的零件重新组装：
//
//     J        <- kdl_jacobian.hpp::computeJacobian
//     J̇·q̇      <- ChainJntToJacDotSolver
//     J⁺       <- 速度级的伪逆 / 阻尼伪逆
//
// 组装公式：正向 a = J·q̈ + J̇·q̇；反向 q̈ = J⁺·(a − J̇·q̇)。

#include "kdl_acceleration.hpp"

#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/solveri.hpp>
#include <rclcpp/logging.hpp>

#include "kdl_jacobian.hpp"

namespace kdl_kinematics
{
namespace
{
/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_kinematics";

/**
 * @brief 内部辅助：构造一个"输入尺寸不匹配"的 AccelerationResult。
 * @param n    链的自由度，用来让 result.qddot 保持正确尺寸。
 * @param what 出问题的参数名，例如 "qdot"。
 * @return error_code 为 E_SIZE_MISMATCH 的结果。
 */
AccelerationResult sizeMismatch(unsigned int n, const std::string & what)
{
  AccelerationResult result;
  result.qddot = KDL::JntArray(n);
  result.error_code = KDL::SolverI::E_SIZE_MISMATCH;
  result.message = what + " 的长度与链的自由度不一致";
  RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
  return result;
}

}  // namespace

// ---------------------------------------------------------------------------
// 一、核心中间项 J̇·q̇
// ---------------------------------------------------------------------------

bool jacobianDotTimesQdot(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  KDL::Twist & jac_dot_q_dot)
{
  const unsigned int n = chain.getNrOfJoints();
  if (q.rows() != n || qdot.rows() != n) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag),
      "jacobianDotTimesQdot: 输入长度(%u, %u)与链的自由度(%u)不一致",
      static_cast<unsigned>(q.rows()), static_cast<unsigned>(qdot.rows()),
      static_cast<unsigned>(n));
    return false;
  }

  // 这个求解器的输入是"位置 + 速度"打包在一起的 JntArrayVel。
  const KDL::JntArrayVel q_vel(q, qdot);

  KDL::ChainJntToJacDotSolver solver(chain);
  // 显式指定 HYBRID 表示（参考坐标系 = 基座，参考点 = 末端），
  // 与 computeJacobian 默认的雅可比口径一致。
  // 这一点很重要：只有口径相同，J̇·q̇ 才能和 J·q̈ 直接相加。
  solver.setHybridRepresentation();

  const int error = solver.JntToJacDot(q_vel, jac_dot_q_dot);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "jacobianDotTimesQdot failed: %s",
      solver.strError(error));
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 二、正向：关节加速度 -> 末端加速度
// ---------------------------------------------------------------------------

bool jointToCartesianAcc(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, KDL::Twist & acceleration)
{
  const unsigned int n = chain.getNrOfJoints();
  if (qdot.rows() != n || qddot.rows() != n) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag),
      "jointToCartesianAcc: qdot/qddot 长度(%u, %u)与链的自由度(%u)不一致",
      static_cast<unsigned>(qdot.rows()), static_cast<unsigned>(qddot.rows()),
      static_cast<unsigned>(n));
    return false;
  }

  KDL::Jacobian jacobian;
  if (!computeJacobian(chain, q, jacobian)) {
    return false;
  }

  // 第一项：J · q̈ —— 关节加速度对末端加速度的直接贡献。
  KDL::Twist j_q_ddot;
  KDL::MultiplyJacobian(jacobian, qddot, j_q_ddot);

  // 第二项：J̇ · q̇ —— 科氏 / 离心项，即使 q̈ = 0 它也可能非零。
  KDL::Twist j_dot_q_dot;
  if (!jacobianDotTimesQdot(chain, q, qdot, j_dot_q_dot)) {
    return false;
  }

  // 两项必须表达在同一坐标系与同一参考点下才能相加，这正是上面
  // 显式指定 HYBRID 表示的原因。
  acceleration = j_q_ddot + j_dot_q_dot;
  return true;
}

// ---------------------------------------------------------------------------
// 三、反向：末端加速度 -> 关节加速度
// ---------------------------------------------------------------------------

AccelerationResult cartesianToJointAcc(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::Twist & acceleration)
{
  const unsigned int n = chain.getNrOfJoints();
  if (q.rows() != n) {
    return sizeMismatch(n, "q");
  }
  if (qdot.rows() != n) {
    return sizeMismatch(n, "qdot");
  }

  AccelerationResult result;
  result.qddot = KDL::JntArray(n);

  // 关键一步：把科氏项减掉。
  //   a = J·q̈ + J̇·q̇   =>   r = a − J̇·q̇ = J·q̈
  // 减完之后，问题就变成了和速度级一模一样的线性方程组 J·x = r，
  // 于是可以直接复用速度级求解器——它只负责解方程，
  // x 是速度还是加速度由输入 r 的量纲决定。
  KDL::Twist j_dot_q_dot;
  if (!jacobianDotTimesQdot(chain, q, qdot, j_dot_q_dot)) {
    result.error_code = KDL::SolverI::E_NO_CONVERGE;
    result.message = "计算 J̇·q̇ 失败（见上方日志）";
    return result;
  }
  const KDL::Twist residual = acceleration - j_dot_q_dot;

  KDL::ChainIkSolverVel_pinv solver(chain);
  const int error = solver.CartToJnt(q, residual, result.qddot);

  result.error_code = error;
  result.message = solver.strError(error);
  result.sigma_min = minSingularValue(chain, q);

  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "cartesianToJointAcc failed: %s",
      result.message.c_str());
  }
  return result;
}

AccelerationResult cartesianToJointAccDamped(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::Twist & acceleration, double lambda, double singular_eps)
{
  const unsigned int n = chain.getNrOfJoints();
  if (q.rows() != n) {
    return sizeMismatch(n, "q");
  }
  if (qdot.rows() != n) {
    return sizeMismatch(n, "qdot");
  }

  AccelerationResult result;
  result.qddot = KDL::JntArray(n);

  // 与 cartesianToJointAcc 完全相同的预处理：先扣掉科氏项。
  KDL::Twist j_dot_q_dot;
  if (!jacobianDotTimesQdot(chain, q, qdot, j_dot_q_dot)) {
    result.error_code = KDL::SolverI::E_NO_CONVERGE;
    result.message = "计算 J̇·q̇ 失败（见上方日志）";
    return result;
  }
  const KDL::Twist residual = acceleration - j_dot_q_dot;

  // 区别只在这里：用阻尼伪逆代替纯伪逆，奇异附近更稳定。
  // 注意 setEps：wdls 只在自判奇异（σ_min < eps）时才施加阻尼，
  // 不放开阈值的话它和 pinv 的结果会完全一样。
  KDL::ChainIkSolverVel_wdls solver(chain);
  solver.setLambda(lambda);
  solver.setEps(singular_eps);
  const int error = solver.CartToJnt(q, residual, result.qddot);

  result.error_code = error;
  result.message = solver.strError(error);
  result.sigma_min = minSingularValue(chain, q);

  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "cartesianToJointAccDamped failed: %s",
      result.message.c_str());
  }
  return result;
}

}  // namespace kdl_kinematics
