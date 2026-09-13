// Copyright (c) 2026, kdl_kinematics authors.
// 教学用途：把运动学结果"讲人话"地打印到终端。
//
// 与 kdl_tools_print 同样的理由单独成文件：求解逻辑不需要知道怎么打印。
// 这里只用标准库 std::ostream，不依赖 ROS 日志，
// 因此既能直接 cout 到屏幕，也能写进 std::ostringstream 做单元测试。

#ifndef KDL_KINEMATICS__KDL_KINEMATICS_PRINT_HPP_
#define KDL_KINEMATICS__KDL_KINEMATICS_PRINT_HPP_

#include <iostream>

#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_acceleration.hpp"
#include "kdl_ik.hpp"
#include "kdl_velocity.hpp"

namespace kdl_kinematics
{

/**
 * @brief 打印一个末端位姿：位置、RPY 欧拉角、以及 3x3 旋转矩阵。
 * @param frame 待打印的位姿。
 * @param os    输出流，默认 std::cout。
 *
 * @note KDL::Frame 没有重载 operator<<，所以这里手工逐项输出。
 * @note RPY = GetRPY()，即绕固定轴 X-Y-Z 依次转 roll-pitch-yaw，
 *       是最直观的一种姿态表示；矩阵则给出完整信息。
 */
void printFrame(const KDL::Frame & frame, std::ostream & os = std::cout);

/**
 * @brief 单行打印一个关节角向量。
 * @param q  待打印的关节角，例如 FK 的输入或 IK 的输出。
 * @param os 输出流，默认 std::cout。
 *
 * @note 数组长度即链的自由度，索引顺序与 KDL::Chain 中可动关节的顺序一致。
 */
void printJntArray(const KDL::JntArray & q, std::ostream & os = std::cout);

/**
 * @brief 打印雅可比矩阵（6 x n），并标注每一块的含义。
 * @param jacobian 待打印的雅可比。
 * @param os       输出流，默认 std::cout。
 *
 * @note 前 3 行是线速度部分，后 3 行是角速度部分。
 */
void printJacobian(const KDL::Jacobian & jacobian, std::ostream & os = std::cout);

/**
 * @brief 打印一次 IK 的求解结果：成功与否、错误码、说明、以及解出的关节角。
 * @param result 待打印的求解结果。
 * @param os     输出流，默认 std::cout。
 *
 * @note 这是两种求解器共用同一个返回结构带来的直接好处：
 *       用同一个打印函数就能把 LMA 与 NR_JL 的结果并排对比。
 */
void printIkResult(const IkResult & result, std::ostream & os = std::cout);

/**
 * @brief 打印一个 6 维 Twist：拆成"线分量 / 角分量"两行。
 * @param twist 待打印的 6 维量；速度与加速度共用这个容器，两者都能传。
 * @param os    输出流，默认 std::cout。
 *
 * @note 这里只打印数值、不标注单位，请在调用处说明是速度还是加速度
 *       （速度：m/s 与 rad/s；加速度：m/s² 与 rad/s²）。
 * @note KDL::Twist 没有重载 operator<<，所以借用 splitTwist() 手工拆开输出。
 */
void printTwist(const KDL::Twist & twist, std::ostream & os = std::cout);

/**
 * @brief 打印一次速度级逆映射的结果：状态、错误码、σ_min、以及解出的关节速度。
 * @param result 待打印的结果。
 * @param os     输出流，默认 std::cout。
 *
 * @note σ_min 会被一并打印，方便直接观察"当前位形离奇异有多近"。
 */
void printVelocityResult(const VelocityResult & result, std::ostream & os = std::cout);

/**
 * @brief 打印一次加速度级逆映射的结果：状态、错误码、σ_min、以及解出的关节加速度。
 * @param result 待打印的结果。
 * @param os     输出流，默认 std::cout。
 */
void printAccelerationResult(const AccelerationResult & result, std::ostream & os = std::cout);

}  // namespace kdl_kinematics

#endif  // KDL_KINEMATICS__KDL_KINEMATICS_PRINT_HPP_
