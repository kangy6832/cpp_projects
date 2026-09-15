// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：把插值结果"讲人话"地打印到终端。
//
// 与 kdl_tools_print / kdl_kinematics_print 同样的理由单独成文件：
// 求解逻辑不需要知道怎么打印。这里只用标准库 std::ostream，不依赖 ROS 日志，
// 因此既能直接 cout 到屏幕，也能写进 std::ostringstream 做单元测试。

#ifndef KDL_INTERPOLATION__KDL_INTERPOLATION_PRINT_HPP_
#define KDL_INTERPOLATION__KDL_INTERPOLATION_PRINT_HPP_

#include <iostream>

#include <kdl/jntarray.hpp>

#include "kdl_quintic.hpp"

namespace kdl_interpolation
{

/**
 * @brief 单行打印一个关节向量。
 * @param values [in] 待打印的关节量（角度、速度、加速度、jerk 都可以）。
 * @param os     [in,out] 输出流，默认 std::cout。
 *
 * @note 只打印数值、不标注单位，请在调用处说明是角度还是速度
 *       （角度 rad、速度 rad/s、加速度 rad/s²、jerk rad/s³）。
 */
void printJointVector(const KDL::JntArray & values, std::ostream & os = std::cout);

/**
 * @brief 打印单段五次多项式的系数。
 * @param coefficients [in] 待打印的段系数。
 * @param os           [in,out] 输出流，默认 std::cout。
 *
 * @note 会把"归一化系数"和它对应的时长一起打出来，提醒读者这些系数是
 *       对 τ = t/T 的，不是对 t 的——这是本模块最容易看错的一处。
 */
void printQuinticCoefficients(const QuinticCoefficients & coefficients, std::ostream & os = std::cout);

/**
 * @brief 打印关节约束。
 * @param limits [in] 待打印的约束。
 * @param os     [in,out] 输出流，默认 std::cout。
 *
 * @note 长度为 0 的项打印为 "not checked"，与"该关节不受限"做区分：
 *       前者是调用者没设置，后者才是真的不限。
 */
void printJointLimits(const JointLimits & limits, std::ostream & os = std::cout);

/**
 * @brief 打印轨迹概览：关节数、段数、总时长、各段时长。
 * @param trajectory [in] 待打印的轨迹。
 * @param os         [in,out] 输出流，默认 std::cout。
 */
void printTrajectorySummary(const QuinticTrajectory & trajectory, std::ostream & os = std::cout);

/**
 * @brief 打印路点表：每个路点的时刻、关节角、以及解出的速度与加速度。
 * @param trajectory [in] 待打印的轨迹。
 * @param os         [in,out] 输出流，默认 std::cout。
 *
 * @note 这张表是观察 C⁴ 拼接的关键：同一行里的速度、加速度就是左右两段
 *       共用的那一组值，所以只要表里每个路点只有"一个"速度，连续性就已经
 *       成立；具体数值是否合理（比如中间点速度会不会突然很大）一眼可见。
 */
void printKnotTable(const QuinticTrajectory & trajectory, std::ostream & os = std::cout);

/**
 * @brief 打印一次采样的结果：时刻、关节角、速度、加速度。
 * @param t     [in] 采样时刻，单位 s。
 * @param q     [in] 关节角，单位 rad。
 * @param qdot  [in] 关节速度，单位 rad/s。
 * @param qddot [in] 关节加速度，单位 rad/s²。
 * @param os    [in,out] 输出流，默认 std::cout。
 */
void printTrajectorySample(
  double t, const KDL::JntArray & q, const KDL::JntArray & qdot, const KDL::JntArray & qddot,
  std::ostream & os = std::cout);

/**
 * @brief 打印一次轨迹构建的结果：成功与否、失败原因。
 * @param result [in] 待打印的构建结果。
 * @param os     [in,out] 输出流，默认 std::cout。
 *
 * @note 成功时只打印一行结论；轨迹本体请用 printTrajectorySummary 或
 *       printKnotTable 单独展示，避免把两种信息搅在一起。
 */
void printTrajectoryResult(const TrajectoryResult & result, std::ostream & os = std::cout);

}  // namespace kdl_interpolation

#endif  // KDL_INTERPOLATION__KDL_INTERPOLATION_PRINT_HPP_
