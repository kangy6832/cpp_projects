// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：kdl_fdynamics.hpp 中声明的函数在这里落地。
//
// 与 kdl_idynamics.cpp 结构完全对称，但有一个极易踩坑的细节要注意：
//
//   ChainIdSolver_RNE::CartToJnt(q, qdot, qddot, f_ext, torque)
//   ChainFdSolver_RNE::CartToJnt(q, qdot, torque, f_ext, qddot)
//                                      ~~~~~~        ~~~~~
//   两者的中间两个参数顺序是**反的**，写反了不会报错，只会算出错误结果。
//   具体见下面 solveForwardDynamics() 里的注释。

#include "kdl_fdynamics.hpp"

#include <string>

#include <kdl/chainfdsolver_recursive_newton_euler.hpp>
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
 * @note 与 kdl_idynamics.cpp 中同名函数逻辑一致；两个文件各自保持自包含，
 *       读任意一个文件都能完整看懂，因此这里刻意不做跨文件抽取。
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
 * @param chain  运动学链。
 * @param q      关节角。
 * @param qdot   关节速度。
 * @param torque 关节力矩。
 * @return 全部匹配返回 true；否则打日志并返回 false。
 */
bool jointSizesValid(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque)
{
  const unsigned int n = chain.getNrOfJoints();
  if (q.rows() == n && qdot.rows() == n && torque.rows() == n) {
    return true;
  }
  RCLCPP_ERROR(
    rclcpp::get_logger(kLogTag),
    "q/qdot/torque 的长度(%u, %u, %u)必须都等于链的自由度(%u)",
    static_cast<unsigned>(q.rows()), static_cast<unsigned>(qdot.rows()),
    static_cast<unsigned>(torque.rows()), static_cast<unsigned>(n));
  return false;
}

/**
 * @brief 构造一个"输入尺寸不匹配"的 FdResult。
 * @param chain 运动学链，用来确定输出数组的尺寸。
 * @return error_code 为 E_SIZE_MISMATCH 的结果。
 */
FdResult sizeMismatchResult(const KDL::Chain & chain)
{
  FdResult result;
  result.qddot = KDL::JntArray(chain.getNrOfJoints());
  result.error_code = KDL::SolverI::E_SIZE_MISMATCH;
  result.message = "q / qdot / torque 的长度必须都等于链的自由度";
  return result;
}

/**
 * @brief 正动力学的公共实现：三个公开函数最终都调用它。
 * @param chain     运动学链。
 * @param q         关节角，单位 rad。
 * @param qdot      关节速度，单位 rad/s。
 * @param torque    关节力矩，单位 N·m。
 * @param tip_wrench_local 末端外力，表达在末端局部系；无外力时传 Wrench::Zero()。
 * @param gravity   重力向量，基座坐标系，单位 m/s²。
 * @return FdResult，其中 qddot 为关节加速度，单位 rad/s²。
 */
FdResult solveForwardDynamics(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque, const KDL::Wrench & tip_wrench_local,
  const KDL::Vector & gravity)
{
  FdResult result;
  result.qddot = KDL::JntArray(chain.getNrOfJoints());

  KDL::ChainFdSolver_RNE solver(chain, gravity);

  const KDL::Wrenches f_ext =
    makeExternalWrenches(chain.getNrOfSegments(), tip_wrench_local);

  // 注意参数顺序：FD 是 (q, qdot, torque, f_ext, qddot)，
  // 而 ID 是 (q, qdot, qddot, f_ext, torque)。力矩和加速度的位置刚好互换。
  const int error = solver.CartToJnt(q, qdot, torque, f_ext, result.qddot);

  result.error_code = error;
  result.message = solver.strError(error);
  if (error < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "forwardDynamics failed: %s", result.message.c_str());
  }
  return result;
}

}  // namespace

FdResult forwardDynamics(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque, const KDL::Vector & gravity)
{
  if (!jointSizesValid(chain, q, qdot, torque)) {
    return sizeMismatchResult(chain);
  }
  return solveForwardDynamics(chain, q, qdot, torque, KDL::Wrench::Zero(), gravity);
}

FdResult forwardDynamicsWithTipWrenchLocal(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque, const KDL::Wrench & tip_wrench,
  const KDL::Vector & gravity)
{
  if (!jointSizesValid(chain, q, qdot, torque)) {
    return sizeMismatchResult(chain);
  }
  return solveForwardDynamics(chain, q, qdot, torque, tip_wrench, gravity);
}

FdResult forwardDynamicsWithTipWrenchBase(
  const KDL::Chain & chain, const KDL::JntArray & q, const KDL::JntArray & qdot,
  const KDL::JntArray & torque, const KDL::Wrench & tip_wrench_base,
  const KDL::Vector & gravity)
{
  if (!jointSizesValid(chain, q, qdot, torque)) {
    return sizeMismatchResult(chain);
  }

  // 与逆动力学的基座系版本一致：先借一次正运动学拿到末端姿态。
  KDL::Frame tip_frame;
  if (!kdl_kinematics::forwardKinematics(chain, q, tip_frame)) {
    FdResult result = sizeMismatchResult(chain);
    result.message = "求末端姿态失败，无法把外力换算到末端局部坐标系";
    return result;
  }

  // v_tip = M⁻¹ · v_base（tip_frame.M 记作 R_base←tip）。
  const KDL::Rotation R_tip_from_base = tip_frame.M.Inverse();
  const KDL::Wrench tip_wrench_local(
    R_tip_from_base * tip_wrench_base.force,
    R_tip_from_base * tip_wrench_base.torque);

  return solveForwardDynamics(chain, q, qdot, torque, tip_wrench_local, gravity);
}

}  // namespace kdl_dynamics
