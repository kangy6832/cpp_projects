// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：kdl_fk.hpp 中声明的函数在这里落地。

#include "kdl_fk.hpp"

#include <kdl/chainfksolverpos_recursive.hpp>
#include <rclcpp/logging.hpp>

namespace kdl_kinematics
{
namespace
{
/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_kinematics";
}  // namespace

bool forwardKinematics(
  const KDL::Chain & chain, const KDL::JntArray & q, KDL::Frame & frame)
{
  // 求解器构造时会分配内存；这里每次新建只为教学清晰。
  // 真实控制循环里应把它提到循环外，只构造一次。
  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  // segmentNr = -1 表示"算到整条链的末端"。
  // 返回值为 KDL 错误码：0 = E_NOERROR，负数 = 失败。
  const int error = fk_solver.JntToCart(q, frame);
  if (error < 0) {
    // KDL 内部已经校验了 q 的尺寸，失败时用 strError 把错误码翻译成文本即可。
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "forwardKinematics failed: %s",
      fk_solver.strError(error));
    return false;
  }
  return true;
}

bool forwardKinematicsAllSegments(
  const KDL::Chain & chain, const KDL::JntArray & q, std::vector<KDL::Frame> & frames)
{
  KDL::ChainFkSolverPos_recursive fk_solver(chain);

  // 先按"段数"准备好输出（注意：段数 != 关节数，因为可能有固定段）。
  frames.assign(chain.getNrOfSegments(), KDL::Frame::Identity());

  const int error = fk_solver.JntToCart(q, frames);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "forwardKinematicsAllSegments failed: %s",
      fk_solver.strError(error));
    return false;
  }
  return true;
}

}  // namespace kdl_kinematics
