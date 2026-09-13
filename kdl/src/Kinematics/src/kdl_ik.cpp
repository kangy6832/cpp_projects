// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：kdl_ik.hpp 中声明的函数在这里落地。
//
// 两个求解器的共同点：都接受 (q_init, target) 得到 q，
// 都是数值迭代法，因此"初值"和"迭代上限"都会影响结果。

#include "kdl_ik.hpp"

#include <string>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/solveri.hpp>
#include <rclcpp/logging.hpp>

namespace kdl_kinematics
{
namespace
{
/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_kinematics";

/**
 * @brief 内部辅助：构造一个"输入尺寸不匹配"的 IkResult，并打一条日志。
 * @param q_init 迭代初值，用来让 result.q 保持合理尺寸。
 * @param what   出问题的参数名，例如 "q_min"。
 * @param chain  运动学链，用来报告期望的自由度数。
 * @return error_code 为 E_SIZE_MISMATCH 的结果。
 */
IkResult sizeMismatch(
  const KDL::JntArray & q_init, const std::string & what, const KDL::Chain & chain)
{
  IkResult result;
  result.q = q_init;
  result.error_code = KDL::SolverI::E_SIZE_MISMATCH;
  result.message = what + " 的长度与链的自由度(" +
    std::to_string(chain.getNrOfJoints()) + ")不一致";
  RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s", result.message.c_str());
  return result;
}

}  // namespace

IkResult solveIkLma(
  const KDL::Chain & chain, const KDL::JntArray & q_init, const KDL::Frame & target,
  double eps, unsigned int max_iter)
{
  IkResult result;
  // 输出数组必须先具有正确尺寸：KDL 求解器不会替你分配，
  // 尺寸不符时它会直接返回 E_SIZE_MISMATCH。
  result.q = q_init;

  if (q_init.rows() != chain.getNrOfJoints()) {
    return sizeMismatch(q_init, "q_init", chain);
  }

  // LMA 自带前方 FK 与雅可比，所以只需要一条链。
  KDL::ChainIkSolverPos_LMA solver(chain, eps, static_cast<int>(max_iter));
  const int error = solver.CartToJnt(q_init, target, result.q);

  result.error_code = error;
  result.message = solver.strError(error);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "solveIkLma failed: %s", result.message.c_str());
  }
  return result;
}

IkResult solveIkNrJl(
  const KDL::Chain & chain, const KDL::JntArray & q_init, const KDL::Frame & target,
  const KDL::JntArray & q_min, const KDL::JntArray & q_max,
  unsigned int max_iter, double eps)
{
  IkResult result;
  result.q = q_init;

  const unsigned int n = chain.getNrOfJoints();
  if (q_init.rows() != n) {
    return sizeMismatch(q_init, "q_init", chain);
  }
  if (q_min.rows() != n) {
    return sizeMismatch(q_init, "q_min", chain);
  }
  if (q_max.rows() != n) {
    return sizeMismatch(q_init, "q_max", chain);
  }

  // NR_JL 不像 LMA 那样自带全部零件，它需要调用方先备好两件事：
  //   1) FK 求解器：每轮迭代算一次"当前末端位姿"，好和目标相减得到误差；
  //   2) 速度级 IK（这里用伪逆 pinv）：把该位姿误差换算成关节增量 Δq。
  // 这也正是"牛顿-拉夫逊"的迭代结构：q ← q + J⁻¹·(target - fk(q))。
  KDL::ChainFkSolverPos_recursive fk_solver(chain);
  KDL::ChainIkSolverVel_pinv ik_vel_solver(chain);
  KDL::ChainIkSolverPos_NR_JL solver(
    chain, q_min, q_max, fk_solver, ik_vel_solver, max_iter, eps);

  const int error = solver.CartToJnt(q_init, target, result.q);

  result.error_code = error;
  result.message = solver.strError(error);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "solveIkNrJl failed: %s", result.message.c_str());
  }
  return result;
}

}  // namespace kdl_kinematics
