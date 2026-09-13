// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：正运动学（位置 FK）。
//
// 正运动学回答的问题非常朴素：
//
//     已知每个关节转了多少（关节角 q），末端在哪里？
//
// KDL 里对应的求解器是 ChainFkSolverPos_recursive：
// 它从基座开始，沿着链把每个 segment 的变换依次右乘起来：
//
//     T_base_tip = T_seg1(q1) · T_seg2(q2) · ... · T_segN(qN)
//
// 注意"recursive"指的就是这个逐段递推，并非递归函数调用。

#ifndef KDL_KINEMATICS__KDL_FK_HPP_
#define KDL_KINEMATICS__KDL_FK_HPP_

#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

namespace kdl_kinematics
{

/**
 * @brief 正运动学（位置）：给定关节角，求末端坐标系相对基座的位姿。
 * @param chain 运动学链（由 kdl_tools::buildChain 从 URDF 截取得到）。
 * @param q     关节角向量，长度必须等于 chain.getNrOfJoints()。
 * @param frame [输出] 末端位姿 T_base_tip；成功时被填充，失败时保持原样。
 * @return true 表示求解成功；false 表示输入尺寸不匹配或求解器报错。
 *
 * @note 本函数只做 FK 本身，不做任何缓存：每次调用都会新建一个求解器。
 *       若要在控制循环里高频调用，应把 ChainFkSolverPos_recursive 提到循环外
 *       只构造一次（构造时会有内存分配），这里为了教学清晰才每次新建。
 * @note 链里可能含有固定段（fixed segment），它们不消耗关节角，
 *       所以 q 的长度是"可动关节数"，而不是"段数"。
 */
bool forwardKinematics(
  const KDL::Chain & chain, const KDL::JntArray & q, KDL::Frame & frame);

/**
 * @brief 正运动学（逐段）：求出链上每一段末端相对基座的位姿。
 * @param chain  运动学链。
 * @param q      关节角向量，长度必须等于 chain.getNrOfJoints()。
 * @param frames [输出] 大小等于 chain.getNrOfSegments()；
 *               frames[i] 是"第 i 段末端"相对基座的位姿，frames.back() 即末端。
 * @return true 表示求解成功。
 *
 * @note 教学价值：能直观看到每个关节坐标系的落点，
 *       也方便验证 frames.back() 与 forwardKinematics() 的结果一致。
 */
bool forwardKinematicsAllSegments(
  const KDL::Chain & chain, const KDL::JntArray & q, std::vector<KDL::Frame> & frames);

}  // namespace kdl_kinematics

#endif  // KDL_KINEMATICS__KDL_FK_HPP_
