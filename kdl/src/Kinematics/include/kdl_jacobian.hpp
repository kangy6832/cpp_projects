// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：末端速度雅可比。
//
// 雅可比 J(q) 描述了"关节速度"到"末端速度"的线性映射：
//
//     [ v ; ω ] = J(q) · qdot          （v 线速度，ω 角速度）
//
// 它之所以重要，是因为动力学、静力分析、以及所有"速度级"的逆解
// （例如牛顿-拉夫逊迭代里的 ChainIkSolverVel_pinv）都建立在它之上。
//
// J 是一个 6 x n 的矩阵（n = 可动关节数）：
//   - 前 3 行对应线速度 v，后 3 行对应角速度 ω；
//   - 第 j 列描述"只让第 j 个关节以单位速度运动时，末端的速度"，
//     旋转关节的列 = [ axis × (p_tip - p_joint) ; axis ]。
//
// 本模块用 ChainJntToJacSolver，结果表达在"基座坐标系、参考点在末端"。

#ifndef KDL_KINEMATICS__KDL_JACOBIAN_HPP_
#define KDL_KINEMATICS__KDL_JACOBIAN_HPP_

#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

namespace kdl_kinematics
{

/**
 * @brief 计算末端速度雅可比 J(q)。
 * @param chain    运动学链（由 kdl_tools::buildChain 从 URDF 截取得到）。
 * @param q        关节角向量，长度必须等于 chain.getNrOfJoints()。
 * @param jacobian [输出] 6 x n 的雅可比；本函数内部会先 resize 成正确尺寸，
 *                 因此调用方传一个默认构造的 KDL::Jacobian 即可。
 * @return true 表示求解成功；false 表示输入尺寸不匹配或求解器报错。
 *
 * @note 雅可比是"当前位形"的局部线性化：q 变了，J 就变了，
 *       所以它必须和 q 成对出现，不能脱离位形单独讨论。
 * @note 若某一位形下 J 的秩下降（末端失去某个方向的运动能力），
 *       该位形称为奇异位形，此时速度级逆解会失效或数值爆炸。
 */
bool computeJacobian(
  const KDL::Chain & chain, const KDL::JntArray & q, KDL::Jacobian & jacobian);

}  // namespace kdl_kinematics

#endif  // KDL_KINEMATICS__KDL_JACOBIAN_HPP_
