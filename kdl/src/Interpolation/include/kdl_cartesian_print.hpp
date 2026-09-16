// Copyright (c) 2026, kdl_interpolation authors.
// 教学用途：把笛卡尔插值的结果"讲人话"地打印到终端。
//
// 与 kdl_interpolation_print（关节空间那一套）保持完全相同的分工：
// 求解逻辑不需要知道怎么打印。这里只用标准库 std::ostream，不依赖 ROS 日志，
// 因此既能直接 cout 到屏幕，也能写进 std::ostringstream 做单元测试。
//
// 单独成文件的另一个理由：笛卡尔量比关节量"多一层"——位姿里有位置和姿态两半，
// 速度里有线和角两半，混在关节空间的打印文件里只会让两边都变难读。

#ifndef KDL_INTERPOLATION__KDL_CARTESIAN_PRINT_HPP_
#define KDL_INTERPOLATION__KDL_CARTESIAN_PRINT_HPP_

#include <array>
#include <iostream>

#include <kdl/frames.hpp>

#include "kdl_cartesian.hpp"
#include "kdl_quintic.hpp"

namespace kdl_interpolation
{

/**
 * @brief 打印一个末端位姿：位置、RPY 欧拉角、以及四元数。
 * @param pose [in] 待打印的位姿（位置单位 m，姿态相对基座）。
 * @param os   [in,out] 输出流，默认 std::cout。
 *
 * @note KDL::Frame 没有重载 operator<<，所以这里手工逐项输出。
 * @note 姿态同时给 RPY 与四元数两种表示：RPY 直观（示例里靠它看懂姿态在怎么变），
 *       四元数则是 slerp 真正工作的那个空间，两者互为校验。
 * @note 四元数按 (x, y, z, w) 顺序打印，与 KDL::Rotation::GetQuaternion() 一致，
 *       而 w 是标量部分——这一点在别处常写成 (w, x, y, z)，读的时候要看清楚。
 */
void printCartesianPose(const KDL::Frame & pose, std::ostream & os = std::cout);

/**
 * @brief 打印一次采样得到的完整笛卡尔状态：位姿 + 线/角速度 + 线/角加速度。
 * @param state [in] 待打印的状态。
 * @param os    [in,out] 输出流，默认 std::cout。
 *
 * @note 五行分别对应：位姿、v (m/s)、ω (rad/s)、a (m/s²)、α (rad/s²)，全部表达在
 *       基座坐标系。单位写在行末而不是塞进数值里，是为了让数值列仍然能被直接复制。
 */
void printCartesianState(const CartesianState & state, std::ostream & os = std::cout);

/**
 * @brief 打印某一时刻的笛卡尔状态（先打时刻，再打状态本体）。
 * @param t     [in] 采样时刻，单位 s。
 * @param state [in] 该时刻的状态。
 * @param os    [in,out] 输出流，默认 std::cout。
 */
void printCartesianSample(double t, const CartesianState & state, std::ostream & os = std::cout);

/**
 * @brief 打印笛卡尔约束。
 * @param limits [in] 待打印的约束。
 * @param os     [in,out] 输出流，默认 std::cout。
 *
 * @note 没设置（<= 0）的项打印为 "not checked"，与"该项不受限"做区分：
 *       前者是调用者没设置，后者才是真的不限。
 */
void printCartesianLimits(const CartesianLimits & limits, std::ostream & os = std::cout);

/**
 * @brief 打印单段的位置系数与姿态 slerp 数据。
 * @param position    [in] 长度 3 的位置系数，下标 0/1/2 依次是 x/y/z。
 * @param orientation [in] 该段的姿态数据。
 * @param os          [in,out] 输出流，默认 std::cout。
 *
 * @note 位置系数是"对 τ = t/T"的，这一点必须和 JointLimits 无关但和 QuinticCoefficients
 *       的注释一致地提醒一次——它是最容易看错的地方。
 * @note 姿态没有系数可打，打的是 (转轴, 转角 θ)：转轴**已表达在基座坐标系**，
 *       且该段自始至终绕它转（这正是角速度方向恒定的原因）。
 */
void printCartesianSegment(
  const std::array<QuinticCoefficients, 3> & position, const SlerpSegment & orientation,
  std::ostream & os = std::cout);

/**
 * @brief 打印轨迹概览：路点行为、段数、总时长、各段时长。
 * @param trajectory [in] 待打印的轨迹。
 * @param os         [in,out] 输出流，默认 std::cout。
 *
 * @note 路点行为一定要打出来：同一个路点序列在 kStop 与 kPassThrough 下总时长
 *       相同，但速度剖面完全不同，不写清楚就分不出手上这条是哪一条。
 */
void printCartesianTrajectorySummary(
  const CartesianTrajectory & trajectory, std::ostream & os = std::cout);

/**
 * @brief 打印各段的姿态表：起点 RPY、转轴、转角 θ、平均角速度 θ/T。
 * @param trajectory [in] 待打印的轨迹。
 * @param os         [in,out] 输出流，默认 std::cout。
 *
 * @note 这张表是观察"多段姿态为什么只能停"的关键：把每段的转轴并排打出来，
 *       就能直接看到相邻两段的轴一般不平行——要角速度连续，就只能两边都为 0。
 * @note θ/T 是"匀速转"时的角速度，本模块实际用的是五次时间标度，峰值是它的
 *       1.875 倍，这个数并排放在一起正好说明"平滑的代价"。
 */
void printCartesianOrientationTable(
  const CartesianTrajectory & trajectory, std::ostream & os = std::cout);

/**
 * @brief 打印一次轨迹构建的结果：成功与否、失败原因。
 * @param result [in] 待打印的构建结果。
 * @param os     [in,out] 输出流，默认 std::cout。
 *
 * @note 成功时只打印一行结论；轨迹本体请用 printCartesianTrajectorySummary 或
 *       printCartesianOrientationTable 单独展示，避免把两种信息搅在一起。
 */
void printCartesianResult(const CartesianResult & result, std::ostream & os = std::cout);

}  // namespace kdl_interpolation

#endif  // KDL_INTERPOLATION__KDL_CARTESIAN_PRINT_HPP_
