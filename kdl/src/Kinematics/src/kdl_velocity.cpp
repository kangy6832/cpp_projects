// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：kdl_velocity.hpp 中声明的函数在这里落地。
//
// 正向（关节→末端）是纯粹的矩阵乘法；反向（末端→关节）才需要求逆，
// 而求逆的两种做法（截断伪逆 / 阻尼伪逆）就是本文件的主要看点。

#include "kdl_velocity.hpp"

#include <Eigen/SVD>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <rclcpp/logging.hpp>

#include "kdl_jacobian.hpp"

namespace kdl_kinematics
{
namespace
{
/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_kinematics";
}  // namespace

// ---------------------------------------------------------------------------
// 一、共享的数据结构
// ---------------------------------------------------------------------------

CartesianVector splitTwist(const KDL::Twist & twist)
{
  CartesianVector cartesian;
  cartesian.linear = twist.vel;   // Twist 前 3 维：线分量
  cartesian.angular = twist.rot;  // Twist 后 3 维：角分量
  return cartesian;
}

KDL::Twist mergeTwist(const CartesianVector & cartesian)
{
  // KDL::Twist 的构造函数顺序就是 (线, 角)。
  return KDL::Twist(cartesian.linear, cartesian.angular);
}

// ---------------------------------------------------------------------------
// 二、雅可比的"病态程度"指标
// ---------------------------------------------------------------------------

double minSingularValue(const KDL::Chain & chain, const KDL::JntArray & q)
{
  KDL::Jacobian jacobian;
  if (!computeJacobian(chain, q, jacobian)) {
    return -1.0;
  }

  // J 是 6 x n 的长方阵，做 SVD 拿奇异值。
  // Eigen 的 singularValues() 已按降序排列，所以最后一个就是 σ_min。
  const Eigen::MatrixXd matrix = jacobian.data;
  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd & sigma = svd.singularValues();
  if (sigma.size() == 0) {
    return -1.0;
  }
  return sigma(sigma.size() - 1);
}

// ---------------------------------------------------------------------------
// 三、正向：关节速度 -> 末端速度
// ---------------------------------------------------------------------------

bool jointToCartesianVel(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  KDL::Twist & twist)
{
  const unsigned int n = chain.getNrOfJoints();
  if (qdot.rows() != n) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag),
      "jointToCartesianVel: qdot 长度(%u)与链的自由度(%u)不一致",
      static_cast<unsigned>(qdot.rows()), static_cast<unsigned>(n));
    return false;
  }

  KDL::Jacobian jacobian;
  if (!computeJacobian(chain, q, jacobian)) {
    return false;
  }

  // twist = J · qdot：一步矩阵乘法，没有迭代、也不会失败。
  KDL::MultiplyJacobian(jacobian, qdot, twist);
  return true;
}

// ---------------------------------------------------------------------------
// 四、反向：末端速度 -> 关节速度
// ---------------------------------------------------------------------------

VelocityResult cartesianToJointVel(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::Twist & twist)
{
  // 输出必须先分配好尺寸：KDL 求解器只负责填数，不负责分配。
  VelocityResult result;
  result.qdot = KDL::JntArray(chain.getNrOfJoints());

  // 纯伪逆：对 J 做 SVD，把小于 eps 的奇异值置零后再求逆。
  // 位形 q 靠近奇异时，这一步会把解"切"得跳变，所以要看 sigma_min。
  KDL::ChainIkSolverVel_pinv solver(chain);
  const int error = solver.CartToJnt(q, twist, result.qdot);

  result.error_code = error;
  result.message = solver.strError(error);
  result.sigma_min = minSingularValue(chain, q);

  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "cartesianToJointVel failed: %s",
      result.message.c_str());
  }
  return result;
}

VelocityResult cartesianToJointVelDamped(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::Twist & twist,
  double lambda, double singular_eps)
{
  VelocityResult result;
  result.qdot = KDL::JntArray(chain.getNrOfJoints());

  // 阻尼最小二乘：(JᵀJ + λ²I)⁻¹Jᵀ。
  // λ 越大越稳定，但末端跟踪误差也越大——这是典型的"稳定性 vs 精度"折中。
  KDL::ChainIkSolverVel_wdls solver(chain);
  solver.setLambda(lambda);  // 必须在 CartToJnt 之前设置

  // 关键：还必须把奇异判定阈值放宽，否则 wdls 会认为"没奇异"而施加零阻尼，
  // 结果与 pinv 毫无区别。详见头文件里的 @warning。
  solver.setEps(singular_eps);

  const int error = solver.CartToJnt(q, twist, result.qdot);

  result.error_code = error;
  result.message = solver.strError(error);
  // 说明：wdls 内部其实也维护了同样的量（getSigmaMin()），
  // 但 pinv 没有暴露，为了让两条路径的指标口径完全一致、可以直接比较，
  // 这里仍然统一用 minSingularValue() 重算一次。
  result.sigma_min = minSingularValue(chain, q);

  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "cartesianToJointVelDamped failed: %s",
      result.message.c_str());
  }
  return result;
}

}  // namespace kdl_kinematics
