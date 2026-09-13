// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：kdl_jacobian.hpp 中声明的函数在这里落地。

#include "kdl_jacobian.hpp"

#include <kdl/chainjnttojacsolver.hpp>
#include <rclcpp/logging.hpp>

namespace kdl_kinematics
{
namespace
{
/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_kinematics";
}  // namespace

bool computeJacobian(
  const KDL::Chain & chain, const KDL::JntArray & q, KDL::Jacobian & jacobian)
{
  // 默认构造的 KDL::Jacobian 是 0 列，必须先按自由度分配成 6 x n。
  // （KDL 的求解器只负责填数，不负责分配。）
  jacobian.resize(chain.getNrOfJoints());

  KDL::ChainJntToJacSolver jac_solver(chain);
  const int error = jac_solver.JntToJac(q, jacobian);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "computeJacobian failed: %s",
      jac_solver.strError(error));
    return false;
  }
  return true;
}

}  // namespace kdl_kinematics
