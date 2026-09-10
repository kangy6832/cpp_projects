/**
 * @file forward_kinematics.cpp
 * @brief 机械臂正向运动学（FK）：给定关节角，求末端位姿
 *
 * 用法: forward_kinematics [选项] [q1 q2 ... qN]
 *   --urdf <file>   URDF 文件（默认 model/robotic_arm.urdf）
 *   --root <link>   根连杆（默认 base_link）
 *   --tip  <link>   末端连杆（默认 link6）
 *   --deg           关节角输入单位为度（默认弧度）
 *   --all           同时输出每个连杆的中间位姿
 *   -h, --help      显示帮助
 *
 * @par 示例
 * @code
 *   ./forward_kinematics --deg 0 -30 60 0 45 0
 *   ./forward_kinematics --root base_link --tip link3 0.1 0.2 0.3
 * @endcode
 */

#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_utils.hpp"

#ifndef DEFAULT_URDF_FILE
#define DEFAULT_URDF_FILE "../model/robotic_arm.urdf"
#endif

namespace {

/**
 * @brief 命令行选项与关节角输入的集合
 */
struct Options
{
    std::string urdf_file = DEFAULT_URDF_FILE;  ///< URDF 文件路径
    std::string root_link = "base_link";        ///< 根连杆名
    std::string tip_link = "link6";             ///< 末端连杆名
    bool degrees = false;                       ///< 输入的关节角是否为角度制
    bool all_frames = false;                    ///< 是否输出每个连杆的中间位姿
    std::vector<double> q;                      ///< 输入的关节角，不足的按 0 补齐
};

/**
 * @brief 打印命令行用法说明
 * @param[in] prog 程序名（通常为 argv[0]）
 */
void printUsage(const char* prog)
{
    std::cout
        << "用法: " << prog << " [选项] [q1 q2 ... qN]\n"
        << "  --urdf <file>   URDF 文件（默认 " << DEFAULT_URDF_FILE << "）\n"
        << "  --root <link>   根连杆（默认 base_link）\n"
        << "  --tip  <link>   末端连杆（默认 link6）\n"
        << "  --deg           关节角输入单位为度（默认弧度）\n"
        << "  --all           同时输出每个连杆的中间位姿\n"
        << "  -h, --help      显示本帮助\n"
        << "\n示例:\n"
        << "  " << prog << "                        # 零位姿\n"
        << "  " << prog << " --deg 0 -30 60 0 45 0 # 角度制输入\n"
        << "  " << prog << " 0.1 0.2 0.3           # 不足的关节按 0 补齐\n";
}

/**
 * @brief 解析命令行参数
 * @param[in]  argc 命令行参数个数
 * @param[in]  argv 命令行参数数组
 * @param[out] opt  解析得到的选项，其中 q 按出现顺序收集所有数值参数
 * @return 解析成功返回 true；缺少参数值、无法解析为数值或遇到未知选项时返回 false
 * @note 遇到 -h/--help 会打印用法后直接 std::exit(0)
 */
bool parseArgs(int argc, char** argv, Options& opt)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        auto takeValue = [&](std::string& out) -> bool {
            if (i + 1 >= argc) {
                std::cerr << "错误: " << arg << " 缺少参数值" << std::endl;
                return false;
            }
            out = argv[++i];
            return true;
        };

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--urdf") {
            if (!takeValue(opt.urdf_file)) return false;
        } else if (arg == "--root") {
            if (!takeValue(opt.root_link)) return false;
        } else if (arg == "--tip") {
            if (!takeValue(opt.tip_link)) return false;
        } else if (arg == "--deg" || arg == "--degree") {
            opt.degrees = true;
        } else if (arg == "--all") {
            opt.all_frames = true;
        } else if (arg.size() > 1 && arg[0] == '-' &&
                   !(std::isdigit(static_cast<unsigned char>(arg[1])) || arg[1] == '.')) {
            std::cerr << "错误: 未知选项 " << arg << std::endl;
            return false;
        } else {
            try {
                opt.q.push_back(std::stod(arg));
            } catch (const std::exception&) {
                std::cerr << "错误: 无法解析关节角 '" << arg << "'" << std::endl;
                return false;
            }
        }
    }
    return true;
}

}  // namespace

/**
 * @brief 程序入口：解析参数 -> 构建运动链 -> 正解求解 -> 逐段累乘自检
 * @param[in] argc 命令行参数个数
 * @param[in] argv 命令行参数数组
 * @return 0 表示求解并输出成功；1 表示参数错误、建模失败或正解失败
 */
int main(int argc, char** argv)
{
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        printUsage(argv[0]);
        return 1;
    }

    KDL::Chain chain;
    urdf::ModelInterfaceSharedPtr model;
    if (!kdl_utils::buildChainFromFile(opt.urdf_file, opt.root_link, opt.tip_link,
                                       chain, model)) {
        return 1;
    }

    const unsigned int n = chain.getNrOfJoints();
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "机器人: " << model->getName()
              << " | 运动链: " << opt.root_link << " -> " << opt.tip_link
              << " | 关节数: " << n << std::endl;

    // 关节角：不足补 0，多余报错
    if (opt.q.size() > n) {
        std::cerr << "错误: 提供了 " << opt.q.size() << " 个关节角，但运动链只有 "
                  << n << " 个关节" << std::endl;
        return 1;
    }
    if (!opt.q.empty() && opt.q.size() < n) {
        std::cerr << "提示: 只提供了 " << opt.q.size() << " 个关节角，其余按 0 补齐"
                  << std::endl;
    }

    KDL::JntArray q(n);
    for (unsigned int i = 0; i < n; ++i) {
        const double raw = (i < opt.q.size()) ? opt.q[i] : 0.0;
        q(i) = opt.degrees ? raw * M_PI / 180.0 : raw;
    }

    std::cout << "输入关节角(rad):";
    for (unsigned int i = 0; i < n; ++i) std::cout << " " << q(i);
    std::cout << std::endl;

    kdl_utils::checkJointLimits(chain, q, model);

    // 正解：递归牛顿-欧拉（链式累乘）
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::Frame pose;
    if (int ret = fk.JntToCart(q, pose); ret < 0) {
        std::cerr << "正运动学求解失败 (错误码 " << ret << ")" << std::endl;
        return 1;
    }

    std::cout << "\n末端 " << opt.tip_link << " 相对 " << opt.root_link
              << " 的位姿:" << std::endl;
    kdl_utils::printPose("T", pose);

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 0.0;
    pose.M.GetQuaternion(qx, qy, qz, qw);
    std::cout << "  四元数 (x, y, z, w) = (" << qx << ", " << qy << ", " << qz
              << ", " << qw << ")" << std::endl;
    std::cout << "  齐次变换矩阵 T:" << std::endl;
    kdl_utils::printMatrix(pose);

    // 逐段累乘：既能输出中间连杆位姿，也可作为解算器结果的自检
    const std::vector<KDL::Frame> frames = kdl_utils::linkFrames(chain, q);
    if (opt.all_frames) {
        std::cout << "\n各连杆末端位姿:" << std::endl;
        for (unsigned int i = 0; i < frames.size(); ++i) {
            kdl_utils::printPose(chain.getSegment(i).getName(), frames[i]);
        }
    }
    std::cout << "\n自检: 逐段累乘与解算器结果最大偏差 = "
              << kdl_utils::poseDistance(pose, frames.back()) << std::endl;

    return 0;
}
