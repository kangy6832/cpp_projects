// Copyright (c) 2026, kdl_kinematics authors.
// 教学示例：正运动学 FK、雅可比、逆运动学 IK 的完整闭环。
//
// 运行方式（先 colcon build，再 source install/setup.bash）：
//   ros2 run kdl_tools fk_ik_demo
//
// 演示顺序：
//   1) URDF -> KDL::Chain             （复用 kdl_tools，Kinematics 只吃 KDL::Chain）
//   2) FK：给一组关节角 -> 末端位姿    （forwardKinematics）
//   3) 雅可比 J(q)                     （computeJacobian）
//   4) IK：LMA 从全零初值解回目标      （solveIkLma）
//   5) IK：NR_JL 从全零初值解回目标    （solveIkNrJl，预期失败）
//   6) NR_JL 换成更接近目标的初值      （演示"局部收敛"特性）
//
// 两个关键教学点：
//   a) IK 的解不唯一，所以"对不对"不能靠比较关节角，
//      要用 FK 把解送回末端、与目标位姿比较误差。
//   b) 数值 IK 分"全局性"和"局部性"：LMA 自带阻尼与自适应步长，比较鲁棒；
//      NR_JL 没有步长控制，初值离目标太远时会一步撞上关节限位而卡住。

#include <iostream>
#include <string>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <rclcpp/rclcpp.hpp>
#include <urdf/model.h>

#include "kdl_fk.hpp"
#include "kdl_ik.hpp"
#include "kdl_jacobian.hpp"
#include "kdl_kinematics_print.hpp"
#include "kdl_tools.hpp"

// 由 CMake 在编译期传入（见 CMakeLists.txt 的 KDL_TOOLS_MODEL_DIR）。
#ifndef KDL_TOOLS_MODEL_DIR
#define KDL_TOOLS_MODEL_DIR "."
#endif

namespace
{

/**
 * @brief 从 urdf::Model 读取链上各关节的位置限位，供 NR_JL 使用。
 * @param model 已解析的 URDF 模型。
 * @param chain 运动学链。
 * @param q_min [输出] 每个关节的下限。
 * @param q_max [输出] 每个关节的上限。
 * @return true 表示所有关节都成功读到限位。
 *
 * @note 这一步必须回到 urdf::Model，因为 KDL::Joint 只保存"关节怎么动"
 *       （类型 + 轴），并不保存上下限。这就是"限位是外部信息"的含义。
 * @note KDL 给可动关节编号时会跳过固定段，所以这里也用同样的规则计数。
 */
bool readJointLimits(
  const urdf::Model & model, const KDL::Chain & chain,
  KDL::JntArray & q_min, KDL::JntArray & q_max)
{
  const unsigned int n = chain.getNrOfJoints();
  q_min.resize(n);
  q_max.resize(n);

  unsigned int index = 0;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain.getSegment(i).getJoint();
    if (joint.getType() == KDL::Joint::None) {
      continue;  // 固定段不占自由度，跳过
    }
    const auto it = model.joints_.find(joint.getName());
    if (it == model.joints_.end() || !it->second->limits) {
      return false;
    }
    q_min(index) = it->second->limits->lower;
    q_max(index) = it->second->limits->upper;
    ++index;
  }
  return index == n;
}

/**
 * @brief 计算两个位姿之间的误差。
 * @param a       位姿 a。
 * @param b       位姿 b。
 * @param pos_err [输出] 位置误差（米），即两点距离。
 * @param rot_err [输出] 姿态误差（弧度），即相对旋转的转角。
 */
void poseError(const KDL::Frame & a, const KDL::Frame & b, double & pos_err, double & rot_err)
{
  pos_err = (a.p - b.p).Norm();

  // 相对旋转 R_rel = R_b⁻¹ · R_a，取它的旋转角即为姿态误差。
  KDL::Vector axis;
  rot_err = (b.M.Inverse() * a.M).GetRotAngle(axis);
}

/**
 * @brief 用 FK 复核一次 IK 的结果，并打印误差。
 * @param chain  运动学链。
 * @param target 期望的末端位姿。
 * @param result 待复核的 IK 结果。
 * @return 位置误差（米）；若求解未成功或 FK 失败则返回 -1。
 */
double reportIkAccuracy(
  const KDL::Chain & chain, const KDL::Frame & target, const kdl_kinematics::IkResult & result)
{
  if (!result.success()) {
    std::cout << "  (求解未成功，跳过 FK 复核)\n";
    return -1.0;
  }

  KDL::Frame achieved;
  if (!kdl_kinematics::forwardKinematics(chain, result.q, achieved)) {
    return -1.0;
  }

  double pos_err = 0.0;
  double rot_err = 0.0;
  poseError(target, achieved, pos_err, rot_err);
  std::cout << "  FK 复核: 位置误差 = " << pos_err << " m, 姿态误差 = " << rot_err << " rad\n";
  return pos_err;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("fk_ik_demo");

  // ---- 0. 先建链：Kinematics 只接受 KDL::Chain，建链交给 Tools ----
  const std::string urdf_file = std::string(KDL_TOOLS_MODEL_DIR) + "/robotic_arm.urdf";
  KDL::Chain chain;
  if (!kdl_tools::buildChainFromUrdfFile(urdf_file, chain)) {
    RCLCPP_ERROR(node->get_logger(), "建链失败，退出");
    rclcpp::shutdown();
    return 1;
  }
  const unsigned int n = chain.getNrOfJoints();
  std::cout << "\nchain: " << n << " joints, " << chain.getNrOfSegments() << " segments\n";

  // ---- 1. FK：给一组关节角 -> 末端位姿 ----
  // 这组角度取自 URDF 各关节限位之内，是本例的"真值"。
  KDL::JntArray q_target(n);
  const double demo_angles[] = {0.3, -0.4, 0.6, 0.2, -0.3, 0.5};
  const unsigned int demo_count = sizeof(demo_angles) / sizeof(demo_angles[0]);
  for (unsigned int i = 0; i < n && i < demo_count; ++i) {
    q_target(i) = demo_angles[i];
  }

  std::cout << "\n>>> 1) 正运动学 FK：关节角 -> 末端位姿\n";
  kdl_kinematics::printJntArray(q_target);

  KDL::Frame target;
  if (!kdl_kinematics::forwardKinematics(chain, q_target, target)) {
    rclcpp::shutdown();
    return 1;
  }
  kdl_kinematics::printFrame(target);

  // ---- 2. 雅可比：描述关节速度 -> 末端速度的线性映射 ----
  std::cout << "\n>>> 2) 末端速度雅可比 J(q)\n";
  KDL::Jacobian jacobian;
  if (kdl_kinematics::computeJacobian(chain, q_target, jacobian)) {
    kdl_kinematics::printJacobian(jacobian);
  }

  // ---- 3. IK：LMA（自带 FK 与雅可比，最省事）----
  KDL::JntArray q_zero(n);  // 迭代初值：全零位
  std::cout << "\n>>> 3) IK - ChainIkSolverPos_LMA（初值全零）\n";
  const kdl_kinematics::IkResult lma_result =
    kdl_kinematics::solveIkLma(chain, q_zero, target);
  kdl_kinematics::printIkResult(lma_result);
  const double lma_pos_err = reportIkAccuracy(chain, target, lma_result);

  // ---- 4. IK：NR_JL（牛顿-拉夫逊 + 关节限位）----
  // 先准备好"限位"这个外部信息，它只能来自 URDF。
  urdf::Model model;
  KDL::JntArray q_min;
  KDL::JntArray q_max;
  if (!kdl_tools::loadUrdfModel(urdf_file, model) ||
    !readJointLimits(model, chain, q_min, q_max))
  {
    RCLCPP_ERROR(node->get_logger(), "读取关节限位失败，退出");
    rclcpp::shutdown();
    return 1;
  }

  std::cout << "\n>>> 4) IK - ChainIkSolverPos_NR_JL（初值全零，带 URDF 关节限位）\n";
  std::cout << "  关节限位(来自 URDF):\n";
  kdl_kinematics::printJntArray(q_min);
  kdl_kinematics::printJntArray(q_max);

  const kdl_kinematics::IkResult nrjl_zero =
    kdl_kinematics::solveIkNrJl(chain, q_zero, target, q_min, q_max);
  kdl_kinematics::printIkResult(nrjl_zero);
  const double nrjl_zero_err = reportIkAccuracy(chain, target, nrjl_zero);

  std::cout << "  说明：NR_JL 每步直接 q += pinv(J)·误差，没有步长控制。\n"
            << "        初值离目标太远时，第一步增量就大到被限位截断，卡在边界上出不来。\n";

  // ---- 5. 同一个目标，把初值放得离目标近一些，NR_JL 就能收敛 ----
  // 这里用"目标角 × scale"来构造初值，scale 越小表示初值离目标越远。
  std::cout << "\n>>> 5) NR_JL 的初值敏感性（初值 = 目标角 × scale）\n";
  const double scales[] = {0.2, 0.4, 0.6, 0.8};
  double nrjl_best_err = -1.0;
  for (double scale : scales) {
    KDL::JntArray q_init(n);
    for (unsigned int i = 0; i < n; ++i) {
      q_init(i) = q_target(i) * scale;
    }

    const kdl_kinematics::IkResult result =
      kdl_kinematics::solveIkNrJl(chain, q_init, target, q_min, q_max);

    std::cout << "  scale = " << scale << " -> "
              << (result.success() ? "收敛" : "未收敛")
              << " (error_code = " << result.error_code << ")";
    if (result.success()) {
      KDL::Frame achieved;
      double pos_err = 0.0;
      double rot_err = 0.0;
      kdl_kinematics::forwardKinematics(chain, result.q, achieved);
      poseError(target, achieved, pos_err, rot_err);
      std::cout << ", FK 位置误差 = " << pos_err << " m";
      nrjl_best_err = pos_err;
    }
    std::cout << "\n";
  }

  // ---- 6. 对比总结 ----
  std::cout << "\n>>> 6) 两种求解器对比（同一目标）\n";
  std::cout << "  LMA   (初值全零)      : " << (lma_result.success() ? "收敛" : "未收敛")
            << ", 位置误差 = " << lma_pos_err << " m\n";
  std::cout << "  NR_JL (初值全零)      : " << (nrjl_zero.success() ? "收敛" : "未收敛")
            << ", 位置误差 = " << nrjl_zero_err << " m\n";
  std::cout << "  NR_JL (初值接近目标)  : " << (nrjl_best_err >= 0.0 ? "收敛" : "未收敛")
            << ", 位置误差 = " << nrjl_best_err << " m\n";
  std::cout << "\n  结论：LMA 鲁棒，不需要初值足够接近，适合冷启动；\n"
            << "        NR_JL 轻量，但需要给一个足够接近的初值（例如上一周期的解）。\n"
            << "        两者解出的关节角可能不同，但末端位姿都吻合同一个目标。\n";

  rclcpp::shutdown();
  return 0;
}
