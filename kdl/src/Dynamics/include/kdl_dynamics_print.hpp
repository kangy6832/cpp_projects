// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：把动力学结果"讲人话"地打印到终端。
//
// 与 kdl_tools_print / kdl_kinematics_print 同样的理由单独成文件：
// 求解逻辑不需要知道怎么打印，打印逻辑也不关心怎么求解。
// 这里只用标准库 std::ostream，不依赖 ROS 日志，
// 因此既能直接 cout 到屏幕，也能写进 std::ostringstream 做单元测试。

#ifndef KDL_DYNAMICS__KDL_DYNAMICS_PRINT_HPP_
#define KDL_DYNAMICS__KDL_DYNAMICS_PRINT_HPP_

#include <iostream>

#include <kdl/frames.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

#include "kdl_dynparam.hpp"
#include "kdl_fdynamics.hpp"
#include "kdl_idynamics.hpp"

namespace kdl_dynamics
{

/**
 * @brief 打印关节空间惯性矩阵 M(q)。
 * @param mass 待打印的惯性矩阵（n x n，对称）。
 * @param os   输出流，默认 std::cout。
 *
 * @note KDL::JntSpaceInertiaMatrix 没有重载 operator<<，所以逐元素输出。
 * @note 对角项体现"各关节自身的惯量"，非对角项体现"关节之间的耦合"，
 *       两者一起看才能理解为什么 M 是稠密矩阵。
 */
void printMassMatrix(const KDL::JntSpaceInertiaMatrix & mass, std::ostream & os = std::cout);

/**
 * @brief 打印一个外力旋量（力 + 力矩）。
 * @param wrench 待打印的 Wrench，力单位 N，力矩单位 N·m。
 * @param os     输出流，默认 std::cout。
 *
 * @note Wrench 的两个分量都表达在**它当前所处的坐标系**里，参考点为该坐标系原点。
 *       打印时无法体现这一点，请在调用处说明用的是末端局部系还是基座系。
 */
void printWrench(const KDL::Wrench & wrench, std::ostream & os = std::cout);

/**
 * @brief 打印一次逆动力学结果：状态、错误码、以及各关节力矩。
 * @param result 待打印的结果。
 * @param os     输出流，默认 std::cout。
 */
void printIdResult(const IdResult & result, std::ostream & os = std::cout);

/**
 * @brief 打印一次正动力学结果：状态、错误码、以及各关节加速度。
 * @param result 待打印的结果。
 * @param os     输出流，默认 std::cout。
 */
void printFdResult(const FdResult & result, std::ostream & os = std::cout);

/**
 * @brief 打印一次惯性矩阵计算的结果：状态、错误码、以及矩阵本身。
 * @param result 待打印的结果。
 * @param os     输出流，默认 std::cout。
 */
void printMassResult(const MassResult & result, std::ostream & os = std::cout);

/**
 * @brief 打印一次科氏/离心项的结果：状态、错误码、以及各关节力矩。
 * @param result 待打印的结果。
 * @param os     输出流，默认 std::cout。
 */
void printCoriolisResult(const CoriolisResult & result, std::ostream & os = std::cout);

/**
 * @brief 打印一次重力项的结果：状态、错误码、以及各关节力矩。
 * @param result 待打印的结果。
 * @param os     输出流，默认 std::cout。
 */
void printGravityResult(const GravityResult & result, std::ostream & os = std::cout);

}  // namespace kdl_dynamics

#endif  // KDL_DYNAMICS__KDL_DYNAMICS_PRINT_HPP_
