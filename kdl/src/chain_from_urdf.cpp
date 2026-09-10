/**
 * @file chain_from_urdf.cpp
 * @brief 从 URDF 构建运动学树并提取运动链，附带零位姿下的正运动学验证
 *
 * 用法: chain_from_urdf [urdf_file] [root_link] [tip_link]
 *   urdf_file  URDF 文件路径（默认 ../model/robotic_arm.urdf）
 *   root_link  根连杆名（默认 base_link）
 *   tip_link   末端连杆名（默认 link6）
 *
 * @par 示例
 * @code
 *   ./chain_from_urdf
 *   ./chain_from_urdf ../model/robotic_arm.urdf base_link link6
 * @endcode
 */

#include <iostream>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_utils.hpp"

/**
 * @brief 程序入口：解析 URDF -> 打印树根 -> 提取 root->tip 运动链 -> 零位姿正解验证
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数数组，argv[1..3] 依次为 urdf 文件、根连杆、末端连杆
 * @return 0 表示建链与正解验证均成功；1 表示 URDF 解析失败、取链失败或正解失败
 */
int main(int argc, char** argv)
{
#ifndef DEFAULT_URDF_FILE
#define DEFAULT_URDF_FILE "../model/robotic_arm.urdf"
#endif

    const std::string urdf_file = (argc > 1) ? argv[1] : DEFAULT_URDF_FILE;
    const std::string root_link = (argc > 2) ? argv[2] : "base_link";
    const std::string tip_link = (argc > 3) ? argv[3] : "link6";

    KDL::Tree tree;
    if (!kdl_utils::buildTreeFromFile(urdf_file, tree)) {
        return 1;
    }
    std::cout << "机器人: " << tree.getRootSegment()->second.segment.getName()
              << std::endl;

    KDL::Chain chain;
    if (!tree.getChain(root_link, tip_link, chain)) {
        std::cerr << "无法从 " << root_link << " 到 " << tip_link << " 建立运动链"
                  << std::endl;
        return 1;
    }
    std::cout << "chain: " << root_link << " -> " << tip_link << std::endl;
    kdl_utils::printChain(chain);

    // 正运动学验证：零位姿
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::JntArray q(chain.getNrOfJoints());
    for (unsigned int i = 0; i < q.rows(); ++i) {
        q(i) = 0.0;
    }

    KDL::Frame pose;
    if (fk.JntToCart(q, pose) < 0) {
        std::cerr << "正运动学求解失败" << std::endl;
        return 1;
    }

    double roll = 0.0, pitch = 0.0, yaw = 0.0;
    pose.M.GetRPY(roll, pitch, yaw);
    std::cout << "末端位姿(零位): p=(" << pose.p.x() << ", " << pose.p.y() << ", "
              << pose.p.z() << ") rpy=(" << roll << ", " << pitch << ", " << yaw
              << ")" << std::endl;

    return 0;
}
