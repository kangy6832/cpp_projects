#include <iostream>
#include <string>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_utils.hpp"

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
