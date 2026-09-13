// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：kdl_idynamics.hpp 中声明的函数在这里落地。
//
// 三个函数的关系是"层层包装"，很值得顺着读一遍：
//
//   inverseDynamics                 （零外力）
//   inverseDynamicsWithTipWrenchLocal  （末端局部系外力）  <-+
//   inverseDynamicsWithTipWrenchBase   （基座系外力）      -- 先转坐标系，再调上一个
//
// 前两个最终都汇到同一个 solveInverseDynamics()，区别只在 f_ext 的填充内容。

#include "kdl_idynamics.hpp"

#include <string>

#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/solveri.hpp>
#include <rclcpp/logging.hpp>

#include "kdl_fk.hpp"  // kdl_kinematics::forwardKinematics：基座系外力需要末端姿态

namespace kdl_dynamics
{
namespace
{
/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_dynamics";

/**
 * @brief 组装 KDL 需要的 f_ext 数组。
 * @param nr_of_segments 链的段数（f_ext 的长度必须等于它）。
 * @param tip_wrench_local 末端受到的外力，表达在末端段自身的局部坐标系。
 * @return 长度为 nr_of_segments 的 Wrenches，只有最后一段非零，其余全零。
 *
 * @note KDL 的约定：f_ext[i] 表达在**第 i 段自己的局部坐标系**里。
 *       本模块只暴露"末端一段"的外力，所以其余元素填零。
 */
KDL::Wrenches makeExternalWrenches(
  unsigned int nr_of_segments, const KDL::Wrench & tip_wrench_local)
{
  KDL::Wrenches f_ext(nr_of_segments, KDL::Wrench::Zero());
  if (nr_of_segments > 0) {
    f_ext[nr_of_segments - 1] = tip_wrench_local;
  }
  return f_ext;
}

/**
 * @brief 校验三个关节量的长度是否都等于链的自由度。
 * @param chain 运动学链。
 * @param q     关节角。
 * @param qdot  关节速度。
 * @param qddot 关节加速度。
 * @return 全部匹配返回 true；否则打日志并返回 false。
 */
bool jointSizesValid(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot)
{
  const unsigned int n = chain.getNrOfJoints();
  if (q.rows() == n && qdot.rows() == n && qddot.rows() == n) {
    return true;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger(kLogTag),
    "q/qdot/qddot 的长度(%u, %u, %u)必须都等于链的自由度(%u)",
    static_cast<unsigned>(q.rows()), static_cast<unsigned>(qdot.rows()),
    static_cast<unsigned>(qddot.rows()), static_cast<unsigned>(n));
  return false;
}

/**
 * @brief 构造一个"输入尺寸不匹配"的 IdResult。
 * @param chain 运动学链，用来确定输出数组的尺寸。
 * @return error_code 为 E_SIZE_MISMATCH 的结果。
 */
IdResult sizeMismatchResult(const KDL::Chain & chain)
{
  IdResult result;
  result.torque = KDL::JntArray(chain.getNrOfJoints());
  result.error_code = KDL::SolverI::E_SIZE_MISMATCH;
  result.message = "q / qdot / qddot 的长度必须都等于链的自由度";
  return result;
}

/**
 * @brief 逆动力学的公共实现：三个公开函数最终都调用它。
 * @param chain     运动学链。
 * @param q         关节角，单位 rad。
 * @param qdot      关节速度，单位 rad/s。
 * @param qddot     关节加速度，单位 rad/s²。
 * @param tip_wrench_local 末端外力，表达在末端局部系；无外力时传 Wrench::Zero()。
 * @param gravity   重力向量，基座坐标系，单位 m/s²。
 * @return IdResult，其中 torque 为关节力矩，单位 N·m。
 */
IdResult solveInverseDynamics(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Wrench & tip_wrench_local,
  const KDL::Vector & gravity)
{
  IdResult result;
  // 输出必须先分配好尺寸：KDL 求解器只负责填数，不负责分配。
  result.torque = KDL::JntArray(chain.getNrOfJoints());

  // 逆动力学求解器在构造时就必须知道重力向量。
  KDL::ChainIdSolver_RNE solver(chain, gravity);

  const KDL::Wrenches f_ext =
    makeExternalWrenches(chain.getNrOfSegments(), tip_wrench_local);

  // 递归牛顿-欧拉：一次前向遍历推速度/加速度，再一次后向遍历推力矩。
  const int error = solver.CartToJnt(q, qdot, qddot, f_ext, result.torque);

  result.error_code = error;
  result.message = solver.strError(error);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "inverseDynamics failed: %s", result.message.c_str());
  }
  return result;
}

}  // namespace

IdResult inverseDynamics(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Vector & gravity)
{
  if (!jointSizesValid(chain, q, qdot, qddot)) {
    return sizeMismatchResult(chain);
  }
  return solveInverseDynamics(chain, q, qdot, qddot, KDL::Wrench::Zero(), gravity);
}

IdResult inverseDynamicsWithTipWrenchLocal(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Wrench & tip_wrench,
  const KDL::Vector & gravity)
{
  if (!jointSizesValid(chain, q, qdot, qddot)) {
    return sizeMismatchResult(chain);
  }
  return solveInverseDynamics(chain, q, qdot, qddot, tip_wrench, gravity);
}

IdResult inverseDynamicsWithTipWrenchBase(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & qddot, const KDL::Wrench & tip_wrench_base,
  const KDL::Vector & gravity)
{
  if (!jointSizesValid(chain, q, qdot, qddot)) {
    return sizeMismatchResult(chain);
  }

  // 要把"基座系表达"换成"末端局部系表达"，必须先知道末端此刻的姿态，
  // 于是这里跨到运动学模块借一次正运动学 —— 这正是两者的接口所在。
  KDL::Frame tip_frame;
  if (!kdl_kinematics::forwardKinematics(chain, q, tip_frame)) {
    IdResult result = sizeMismatchResult(chain);
    result.message = "求末端姿态失败，无法把外力换算到末端局部坐标系";
    return result;
  }

  // tip_frame.M 记作 R_base←tip：它把"末端系表达的分量"映射成"基座系表达"。
  // 我们要的是反方向，所以取它的逆：v_tip = M⁻¹ · v_base。
  const KDL::Rotation R_tip_from_base = tip_frame.M.Inverse();

  // 纯旋转只改变分量表达，不改变参考点（两者参考点都在末端），
  // 所以力和力矩用同一个旋转即可。
  const KDL::Wrench tip_wrench_local(
    R_tip_from_base * tip_wrench_base.force,
    R_tip_from_base * tip_wrench_base.torque);

  return solveInverseDynamics(chain, q, qdot, qddot, tip_wrench_local, gravity);
}

}  // namespace kdl_dynamics
