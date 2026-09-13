// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：kdl_dynparam.hpp 中声明的函数在这里落地。
//
// 三个求解函数都只是"建一个 ChainDynParam，调它的一个方法"，
// 真正的工作量在 KDL 内部。这里额外做了两件教学上的事：
//   1) 把 KDL 的错误码翻译成可读说明，便于排查；
//   2) 提供惯性矩阵的对称性 / 正定性检查，把物理性质变成可验证结论。

#include "kdl_dynparam.hpp"

#include <cmath>
#include <string>

#include <Eigen/Cholesky>
#include <kdl/chaindynparam.hpp>
#include <kdl/solveri.hpp>
#include <rclcpp/logging.hpp>

namespace kdl_dynamics
{
namespace
{
/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_dynamics";
}  // namespace

// ---------------------------------------------------------------------------
// 二、三项分解
// ---------------------------------------------------------------------------

MassResult jointSpaceInertia(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::Vector & gravity)
{
  const unsigned int n = chain.getNrOfJoints();

  MassResult result;
  // JntSpaceInertiaMatrix 默认构造是 0x0，必须先 resize 成 n x n。
  result.mass.resize(n);

  if (q.rows() != n) {
    result.error_code = KDL::SolverI::E_SIZE_MISMATCH;
    result.message = "q 的长度必须等于链的自由度";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "jointSpaceInertia: %s", result.message.c_str());
    return result;
  }

  KDL::ChainDynParam solver(chain, gravity);
  const int error = solver.JntToMass(q, result.mass);

  result.error_code = error;
  result.message = solver.strError(error);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "jointSpaceInertia failed: %s", result.message.c_str());
  }
  return result;
}

CoriolisResult coriolisTorque(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::Vector & gravity)
{
  const unsigned int n = chain.getNrOfJoints();

  CoriolisResult result;
  result.torque = KDL::JntArray(n);

  if (q.rows() != n || qdot.rows() != n) {
    result.error_code = KDL::SolverI::E_SIZE_MISMATCH;
    result.message = "q / qdot 的长度必须都等于链的自由度";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "coriolisTorque: %s", result.message.c_str());
    return result;
  }

  KDL::ChainDynParam solver(chain, gravity);
  const int error = solver.JntToCoriolis(q, qdot, result.torque);

  result.error_code = error;
  result.message = solver.strError(error);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "coriolisTorque failed: %s", result.message.c_str());
  }
  return result;
}

GravityResult gravityTorque(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::Vector & gravity)
{
  const unsigned int n = chain.getNrOfJoints();

  GravityResult result;
  result.torque = KDL::JntArray(n);

  if (q.rows() != n) {
    result.error_code = KDL::SolverI::E_SIZE_MISMATCH;
    result.message = "q 的长度必须等于链的自由度";
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "gravityTorque: %s", result.message.c_str());
    return result;
  }

  KDL::ChainDynParam solver(chain, gravity);
  const int error = solver.JntToGravity(q, result.torque);

  result.error_code = error;
  result.message = solver.strError(error);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "gravityTorque failed: %s", result.message.c_str());
  }
  return result;
}

// ---------------------------------------------------------------------------
// 三、惯性矩阵的物理性质检查
// ---------------------------------------------------------------------------

bool isSymmetric(const KDL::JntSpaceInertiaMatrix & mass, double tol)
{
  const unsigned int n = mass.rows();
  if (n == 0 || mass.columns() != n) {
    return false;
  }

  // 只需要检查上三角：M(i,j) 与 M(j,i) 是否相等。
  for (unsigned int i = 0; i < n; ++i) {
    for (unsigned int j = i + 1; j < n; ++j) {
      if (std::abs(mass(i, j) - mass(j, i)) > tol) {
        return false;
      }
    }
  }
  return true;
}

bool isPositiveDefinite(const KDL::JntSpaceInertiaMatrix & mass)
{
  const unsigned int n = mass.rows();
  if (n == 0 || mass.columns() != n) {
    return false;
  }

  // Cholesky 分解（LLT）成功 <=> 矩阵正定。
  const Eigen::LLT<Eigen::MatrixXd> llt(mass.data);
  if (llt.info() != Eigen::Success) {
    return false;
  }

  // 再确认下三角的对角元严格为正：
  // 半正定矩阵虽然 LLT 可能"成功"，但会在对角上留下 0。
  const Eigen::MatrixXd lower = llt.matrixL();
  return (lower.diagonal().array() > 0.0).all();
}

}  // namespace kdl_dynamics
