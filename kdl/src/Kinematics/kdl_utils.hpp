/**
 * @file kdl_utils.hpp
 * @brief 基于 KDL 与 URDF 的运动学工具函数集合（header-only）
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/segment.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf_parser/urdf_parser.h>

/**
 * @namespace kdl_utils
 * @brief URDF 解析、运动链构建、位姿打印与结果自检等通用工具
 */
namespace kdl_utils {

/**
 * @brief 从 URDF 文件构建 KDL::Tree
 * @param[in]  urdf_file URDF 文件路径
 * @param[out] tree      构建得到的运动学树
 * @return 成功返回 true；URDF 解析失败或建树失败返回 false
 */
inline bool buildTreeFromFile(const std::string& urdf_file, KDL::Tree& tree)
{
    urdf::ModelInterfaceSharedPtr model = urdf::parseURDFFile(urdf_file);
    if (!model) {
        std::cerr << "无法解析 URDF 文件: " << urdf_file << std::endl;
        return false;
    }
    if (!kdl_parser::treeFromUrdfModel(*model, tree)) {
        std::cerr << "无法从 URDF 模型构建 KDL::Tree" << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 从 URDF 文件中取出 root -> tip 的运动链，同时保留 urdf 模型
 * @param[in]  urdf_file URDF 文件路径
 * @param[in]  root_link 起始连杆名
 * @param[in]  tip_link  末端连杆名
 * @param[out] chain     提取得到的运动链
 * @param[out] model     URDF 模型句柄，可用于后续读取关节限位等
 * @return 成功返回 true；解析、建树、取链任一步失败返回 false
 */
inline bool buildChainFromFile(const std::string& urdf_file,
                               const std::string& root_link,
                               const std::string& tip_link,
                               KDL::Chain& chain,
                               urdf::ModelInterfaceSharedPtr& model)
{
    model = urdf::parseURDFFile(urdf_file);
    if (!model) {
        std::cerr << "无法解析 URDF 文件: " << urdf_file << std::endl;
        return false;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(*model, tree)) {
        std::cerr << "无法从 URDF 模型构建 KDL::Tree" << std::endl;
        return false;
    }
    if (!tree.getChain(root_link, tip_link, chain)) {
        std::cerr << "无法从 " << root_link << " 到 " << tip_link << " 建立运动链"
                  << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 打印运动链结构：关节数、段数，以及每段的关节名/类型/轴向/偏移
 * @param[in] chain 待打印的运动链
 */
inline void printChain(const KDL::Chain& chain)
{
    std::cout << "关节数: " << chain.getNrOfJoints()
              << ", 段数: " << chain.getNrOfSegments() << std::endl;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& seg = chain.getSegment(i);
        const KDL::Joint& jnt = seg.getJoint();
        const KDL::Frame& f = seg.getFrameToTip();

        std::cout << "  [" << i << "] segment=" << seg.getName()
                  << " joint=" << jnt.getName()
                  << " type=" << jnt.getTypeName()
                  << " axis=(" << jnt.JointAxis().x() << ", "
                  << jnt.JointAxis().y() << ", " << jnt.JointAxis().z() << ")"
                  << " p=(" << f.p.x() << ", " << f.p.y() << ", " << f.p.z() << ")"
                  << std::endl;
    }
}

/**
 * @brief 打印位姿：位置 p 与姿态 RPY（同时输出弧度和角度）
 * @param[in] name 位姿名称，作为输出前缀
 * @param[in] pose 待打印的位姿
 * @note RPY 采用 KDL 约定：先绕 X 转 roll，再绕原 Y 转 pitch，最后绕原 Z 转 yaw；
 *       pitch = ±pi/2 时解不唯一，KDL 会强制取 roll = 0
 */
inline void printPose(const std::string& name, const KDL::Frame& pose)
{
    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    pose.M.GetRPY(roll, pitch, yaw);

    std::cout << "  " << name
              << ": p=(" << pose.p.x() << ", " << pose.p.y() << ", " << pose.p.z() << ")"
              << " rpy=(" << roll << ", " << pitch << ", " << yaw << ") rad"
              << " / (" << roll * 180.0 / M_PI << ", " << pitch * 180.0 / M_PI
              << ", " << yaw * 180.0 / M_PI << ") deg" << std::endl;
}

/**
 * @brief 以 4x4 齐次变换矩阵形式打印位姿
 * @param[in] pose 待打印的位姿
 */
inline void printMatrix(const KDL::Frame& pose)
{
    for (int r = 0; r < 3; ++r) {
        std::cout << "    [";
        for (int c = 0; c < 3; ++c) {
            std::cout << std::setw(12) << pose.M(r, c);
        }
        std::cout << std::setw(12) << pose.p(r) << " ]" << std::endl;
    }
    std::cout << "    [" << std::setw(12) << 0.0 << std::setw(12) << 0.0
              << std::setw(12) << 0.0 << std::setw(12) << 1.0 << " ]" << std::endl;
}

/**
 * @brief 从 URDF 中读取运动链各关节的限位；连续关节用 ±inf_limit 表示无限位
 * @param[in]  chain     运动链
 * @param[in]  model     URDF 模型句柄（由 buildChainFromFile 输出）
 * @param[out] q_min     各关节下限位，长度与关节数一致
 * @param[out] q_max     各关节上限位，长度与关节数一致
 * @param[in]  inf_limit 无限位关节使用的替代边界，默认 1e9
 * @return 成功填满所有关节返回 true；链上关节在 URDF 中查不到导致数量不匹配返回 false
 */
inline bool jointLimits(const KDL::Chain& chain,
                        const urdf::ModelInterfaceSharedPtr& model,
                        KDL::JntArray& q_min, KDL::JntArray& q_max,
                        double inf_limit = 1e9)
{
    q_min.resize(chain.getNrOfJoints());
    q_max.resize(chain.getNrOfJoints());

    unsigned int j = 0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const std::string& jnt_name = chain.getSegment(i).getJoint().getName();
        if (jnt_name.empty()) continue;

        auto it = model->joints_.find(jnt_name);
        if (it == model->joints_.end()) continue;
        const urdf::JointSharedPtr& uj = it->second;

        if (uj->type == urdf::Joint::CONTINUOUS || !uj->limits) {
            q_min(j) = -inf_limit;
            q_max(j) = inf_limit;
        } else {
            q_min(j) = uj->limits->lower;
            q_max(j) = uj->limits->upper;
        }
        ++j;
    }
    return j == chain.getNrOfJoints();
}

/**
 * @brief 依据 URDF 限位检查关节角，越界时向 std::cerr 打印警告
 * @param[in] chain 运动链
 * @param[in] q     待检查的关节角，长度需与 chain 的关节数一致
 * @param[in] model URDF 模型句柄
 * @return 全部关节都在限位内返回 true；存在越界关节返回 false
 */
inline bool checkJointLimits(const KDL::Chain& chain, const KDL::JntArray& q,
                             const urdf::ModelInterfaceSharedPtr& model)
{
    bool ok = true;
    unsigned int j = 0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const std::string& jnt_name = chain.getSegment(i).getJoint().getName();
        if (jnt_name.empty()) continue;

        auto it = model->joints_.find(jnt_name);
        if (it == model->joints_.end()) continue;
        const urdf::JointSharedPtr& uj = it->second;

        const double value = q(j++);
        if (uj->type == urdf::Joint::CONTINUOUS || !uj->limits) continue;

        if (value < uj->limits->lower || value > uj->limits->upper) {
            std::cerr << "警告: 关节 " << jnt_name << " = " << value
                      << " 超出限位 [" << uj->limits->lower << ", "
                      << uj->limits->upper << "]" << std::endl;
            ok = false;
        }
    }
    return ok;
}

/**
 * @brief 计算两个位姿之间的误差，用于自检
 * @param[in] a 位姿 A
 * @param[in] b 位姿 B
 * @return std::pair<位置误差, 旋转误差>；位置误差为平移向量之差的范数（米），
 *         旋转误差为相对旋转的等效转角（弧度）
 */
inline std::pair<double, double> poseError(const KDL::Frame& a, const KDL::Frame& b)
{
    const double pos_err = (a.p - b.p).Norm();
    KDL::Vector axis;
    const double rot_err = (a.M * b.M.Inverse()).GetRotAngle(axis);
    return {pos_err, rot_err};
}

/**
 * @brief 逐段累乘得到每个连杆末端（tip frame）在根坐标系下的位姿
 * @param[in] chain 运动链
 * @param[in] q     关节角，长度需与 chain 的关节数一致
 * @return 每段对应的位姿数组，大小等于 chain.getNrOfSegments()；最后一个元素即末端位姿
 */
inline std::vector<KDL::Frame> linkFrames(const KDL::Chain& chain, const KDL::JntArray& q)
{
    std::vector<KDL::Frame> frames;
    frames.reserve(chain.getNrOfSegments());

    KDL::Frame f = KDL::Frame::Identity();
    unsigned int j = 0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& seg = chain.getSegment(i);
        if (seg.getJoint().getType() != KDL::Joint::None) {
            f = f * seg.pose(q(j));
            ++j;
        } else {
            f = f * seg.getFrameToTip();
        }
        frames.push_back(f);
    }
    return frames;
}

/**
 * @brief 计算两个位姿之间的最大偏差（位置偏差与旋转矩阵元素偏差取最大值），用于自检
 * @param[in] a 位姿 A
 * @param[in] b 位姿 B
 * @return 最大偏差值；位置部分单位为米，旋转部分为无量纲的矩阵元素之差
 */
inline double poseDistance(const KDL::Frame& a, const KDL::Frame& b)
{
    double err = (a.p - b.p).Norm();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            err = std::max(err, std::fabs(a.M(r, c) - b.M(r, c)));
        }
    }
    return err;
}

}  // namespace kdl_utils
