// 机械臂逆向运动学（IK）示例：给定末端目标位姿，求关节角
//
// 用法: inverse_kinematics --xyz x y z [--rpy r p y | --quat x y z w] [选项]
//   --xyz x y z          目标位置（单位 m），也可用位置参数 "x y z [r p y]" 给出
//   --rpy  r p y         目标姿态（RPY，默认 0 0 0）
//   --quat x y z w       目标姿态（四元数，与 --rpy 二选一）
//   --deg                --rpy / --q0 的单位为度
//   --q0   a b c ...     迭代初值（默认全 0）
//   --urdf <file>        URDF 文件（默认 model/robotic_arm.urdf）
//   --root <link>        根连杆（默认 base_link）
//   --tip  <link>        末端连杆（默认 link6）
//   --solver nr|nr_jl|lma 求解器，默认 nr（nr_jl 带关节限位，lma 为列文伯格-马夸尔特）
//   --maxiter N          最大迭代次数（默认 500）
//   --eps   E            收敛阈值（默认 1e-6）
//   --tries N            失败时在限位内随机重启的次数（默认 20）
//   --seed  N            随机初值种子（默认 1，保证可复现）
//   -h, --help           显示帮助
//
// 例: ./inverse_kinematics --xyz 0.9 -0.04 0.53 --rpy 0.8 1.51 3.14
//     ./inverse_kinematics 0.9 -0.04 0.53 --deg --solver nr_jl

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include "kdl_utils.hpp"

#ifndef DEFAULT_URDF_FILE
#define DEFAULT_URDF_FILE "../model/robotic_arm.urdf"
#endif

namespace {

constexpr double kRad2Deg = 180.0 / M_PI;

struct Options
{
    std::string urdf_file = DEFAULT_URDF_FILE;
    std::string root_link = "base_link";
    std::string tip_link = "link6";
    std::string solver = "nr_jl";
    bool degrees = false;
    unsigned int maxiter = 500;
    double eps = 1e-6;
    unsigned int tries = 20;
    unsigned int seed = 1;
    std::vector<double> xyz;
    std::vector<double> rpy;
    std::vector<double> quat;
    std::vector<double> q0;
};

void printUsage(const char* prog)
{
    std::cout
        << "用法: " << prog << " --xyz x y z [--rpy r p y | --quat x y z w] [选项]\n"
        << "  --xyz x y z          目标位置(m)，也可直接作为位置参数给出: x y z [r p y]\n"
        << "  --rpy  r p y         目标姿态 RPY（默认 0 0 0）\n"
        << "  --quat x y z w       目标姿态四元数（与 --rpy 二选一）\n"
        << "  --deg                --rpy / --q0 使用角度制\n"
        << "  --q0   a b c ...     迭代初值（默认全 0）\n"
        << "  --urdf <file>        URDF 文件（默认 " << DEFAULT_URDF_FILE << "）\n"
        << "  --root <link>        根连杆（默认 base_link）\n"
        << "  --tip  <link>        末端连杆（默认 link6）\n"
        << "  --solver nr|nr_jl|lma  求解器（默认 nr_jl，带关节限位）\n"
        << "  --maxiter N          最大迭代次数（默认 500）\n"
        << "  --eps   E            收敛阈值（默认 1e-6）\n"
        << "  --tries N            失败后随机重启次数（默认 20）\n"
        << "  --seed  N            随机初值种子（默认 1）\n"
        << "  -h, --help           显示本帮助\n"
        << "\n示例:\n"
        << "  " << prog << " --xyz 0.9 -0.04 0.53 --rpy 0.80 1.51 3.14\n"
        << "  " << prog << " 0.9 -0.04 0.53 --deg --solver nr_jl\n";
}

bool isNumberToken(const std::string& s)
{
    if (s.empty()) return false;
    size_t pos = (s[0] == '-' || s[0] == '+') ? 1 : 0;
    if (pos == s.size()) return false;
    return std::isdigit(static_cast<unsigned char>(s[pos])) || s[pos] == '.';
}

double toRad(double v, bool degrees) { return degrees ? v * M_PI / 180.0 : v; }

// 读取固定个数的数值参数
bool takeValues(int argc, char** argv, int& i, const std::string& opt,
                std::vector<double>& out, size_t count)
{
    out.clear();
    for (size_t k = 0; k < count; ++k) {
        if (i + 1 >= argc || !isNumberToken(argv[i + 1])) {
            std::cerr << "错误: " << opt << " 需要 " << count << " 个数值" << std::endl;
            return false;
        }
        out.push_back(std::stod(argv[++i]));
    }
    return true;
}

bool parseArgs(int argc, char** argv, Options& opt)
{
    std::vector<double> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--xyz") {
            if (!takeValues(argc, argv, i, arg, opt.xyz, 3)) return false;
        } else if (arg == "--rpy") {
            if (!takeValues(argc, argv, i, arg, opt.rpy, 3)) return false;
        } else if (arg == "--quat") {
            if (!takeValues(argc, argv, i, arg, opt.quat, 4)) return false;
        } else if (arg == "--deg") {
            opt.degrees = true;
        } else if (arg == "--urdf") {
            if (i + 1 >= argc) return false;
            opt.urdf_file = argv[++i];
        } else if (arg == "--root") {
            if (i + 1 >= argc) return false;
            opt.root_link = argv[++i];
        } else if (arg == "--tip") {
            if (i + 1 >= argc) return false;
            opt.tip_link = argv[++i];
        } else if (arg == "--solver") {
            if (i + 1 >= argc) return false;
            opt.solver = argv[++i];
        } else if (arg == "--maxiter") {
            if (i + 1 >= argc) return false;
            opt.maxiter = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "--eps") {
            if (i + 1 >= argc) return false;
            opt.eps = std::stod(argv[++i]);
        } else if (arg == "--tries") {
            if (i + 1 >= argc) return false;
            opt.tries = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "--seed") {
            if (i + 1 >= argc) return false;
            opt.seed = static_cast<unsigned int>(std::stoul(argv[++i]));
        } else if (arg == "--q0") {
            // 一直读到下一个选项为止
            while (i + 1 < argc && isNumberToken(argv[i + 1])) {
                opt.q0.push_back(std::stod(argv[++i]));
            }
            if (opt.q0.empty()) {
                std::cerr << "错误: --q0 后没有数值" << std::endl;
                return false;
            }
        } else if (isNumberToken(arg)) {
            positional.push_back(std::stod(arg));
        } else {
            std::cerr << "错误: 未知选项 " << arg << std::endl;
            return false;
        }
    }

    // 位置参数形式: x y z [r p y]
    if (opt.xyz.empty() && positional.size() >= 3) {
        opt.xyz.assign(positional.begin(), positional.begin() + 3);
        if (opt.rpy.empty() && opt.quat.empty() && positional.size() >= 6) {
            opt.rpy.assign(positional.begin() + 3, positional.begin() + 6);
        }
    }

    if (opt.solver != "nr" && opt.solver != "nr_jl" && opt.solver != "lma") {
        std::cerr << "错误: 未知求解器 '" << opt.solver << "'（可选 nr / nr_jl / lma）"
                  << std::endl;
        return false;
    }
    if (opt.xyz.size() != 3) {
        std::cerr << "错误: 必须用 --xyz x y z（或位置参数）指定目标位置" << std::endl;
        return false;
    }
    if (!opt.rpy.empty() && !opt.quat.empty()) {
        std::cerr << "错误: --rpy 与 --quat 只能二选一" << std::endl;
        return false;
    }
    return true;
}

// 角度归一化到 (-pi, pi]
double wrapAngle(double a)
{
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a <= -M_PI) a += 2.0 * M_PI;
    return a;
}

// 把转动关节的解折算到 (-pi, pi]，不改变末端位姿，仅让输出更直观
void normalizeRevolute(const KDL::Chain& chain, KDL::JntArray& q)
{
    unsigned int j = 0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Joint::JointType type = chain.getSegment(i).getJoint().getType();
        if (type != KDL::Joint::None) {
            if (type != KDL::Joint::TransAxis && type != KDL::Joint::TransX &&
                type != KDL::Joint::TransY && type != KDL::Joint::TransZ) {
                q(j) = wrapAngle(q(j));
            }
            ++j;
        }
    }
}

}  // namespace

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

    // 目标位姿
    KDL::Frame target;
    target.p = KDL::Vector(opt.xyz[0], opt.xyz[1], opt.xyz[2]);
    if (!opt.quat.empty()) {
        target.M = KDL::Rotation::Quaternion(opt.quat[0], opt.quat[1], opt.quat[2],
                                             opt.quat[3]);
    } else if (!opt.rpy.empty()) {
        target.M = KDL::Rotation::RPY(toRad(opt.rpy[0], opt.degrees),
                                      toRad(opt.rpy[1], opt.degrees),
                                      toRad(opt.rpy[2], opt.degrees));
    } else {
        target.M = KDL::Rotation::Identity();
    }
    std::cout << "目标位姿:" << std::endl;
    kdl_utils::printPose("T_goal", target);

    // 关节限位（用于 nr_jl 与随机重启）
    KDL::JntArray q_min(n), q_max(n);
    if (!kdl_utils::jointLimits(chain, model, q_min, q_max)) {
        std::cerr << "警告: 部分关节在 URDF 中未找到，限位按 ±1e9 处理" << std::endl;
    }

    // 迭代初值
    KDL::JntArray q_init(n);
    for (unsigned int i = 0; i < n; ++i) {
        const double raw = (i < opt.q0.size()) ? opt.q0[i] : 0.0;
        q_init(i) = toRad(raw, opt.degrees);
    }
    std::cout << "初始关节角(rad):";
    for (unsigned int i = 0; i < n; ++i) std::cout << " " << q_init(i);
    std::cout << std::endl;

    // 构造求解器：NR 需要 FK + 速度级 IK（雅可比伪逆）
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::ChainIkSolverVel_pinv ik_vel(chain);
    std::unique_ptr<KDL::ChainIkSolverPos> ik;
    if (opt.solver == "nr") {
        ik.reset(new KDL::ChainIkSolverPos_NR(chain, fk, ik_vel, opt.maxiter, opt.eps));
    } else if (opt.solver == "nr_jl") {
        ik.reset(new KDL::ChainIkSolverPos_NR_JL(chain, q_min, q_max, fk, ik_vel,
                                                 opt.maxiter, opt.eps));
    } else {
        ik.reset(new KDL::ChainIkSolverPos_LMA(chain, opt.eps, opt.maxiter, 1e-15));
    }
    std::cout << "求解器: " << opt.solver << " (maxiter=" << opt.maxiter
              << ", eps=" << opt.eps << ")" << std::endl;

    // 求解：失败时在限位内随机重启
    KDL::JntArray q_out(n);
    int ret = -1;
    unsigned int used = 1;
    std::mt19937 rng(opt.seed);
    for (unsigned int attempt = 0; attempt < std::max(1u, opt.tries); ++attempt) {
        if (attempt > 0) {
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            for (unsigned int i = 0; i < n; ++i) {
                const double lo = std::max(q_min(i), -2.0 * M_PI);
                const double hi = std::min(q_max(i), 2.0 * M_PI);
                q_init(i) = lo + dist(rng) * (hi - lo);
            }
        }
        ret = ik->CartToJnt(q_init, target, q_out);
        used = attempt + 1;
        if (ret >= 0) break;
    }

    std::cout << "\n求解状态: " << ret << " (" << ik->strError(ret) << ")"
              << " | 尝试次数: " << used << std::endl;
    if (ret < 0) {
        std::cerr << "逆运动学求解失败（目标可能超出工作空间，可增大 --tries 或换 --solver lma）"
                  << std::endl;
        return 1;
    }
    if (ret != 0) {
        std::cout << "提示: 解为退化解（如雅可比接近奇异），请留意精度" << std::endl;
    }

    // 输出解（转动关节折算到 (-pi, pi]）
    normalizeRevolute(chain, q_out);
    std::cout << "\n逆解关节角:" << std::endl;
    for (unsigned int i = 0; i < n; ++i) {
        const double v = wrapAngle(q_out(i));
        std::cout << "  q" << (i + 1) << " = " << v << " rad / " << v * kRad2Deg
                  << " deg" << std::endl;
    }
    std::cout << "  q(rad):";
    for (unsigned int i = 0; i < n; ++i) std::cout << " " << q_out(i);
    std::cout << std::endl;

    kdl_utils::checkJointLimits(chain, q_out, model);

    // 用正解回代验证
    KDL::Frame achieved;
    if (fk.JntToCart(q_out, achieved) < 0) {
        std::cerr << "正解回代失败" << std::endl;
        return 1;
    }
    std::cout << "\n正解回代位姿:" << std::endl;
    kdl_utils::printPose("T_fk", achieved);

    const auto [pos_err, rot_err] = kdl_utils::poseError(achieved, target);
    std::cout << "\n误差: 位置 " << pos_err * 1000.0 << " mm"
              << " | 姿态 " << rot_err * kRad2Deg << " deg" << std::endl;

    return (pos_err < 1e-3 && rot_err < 1e-3) ? 0 : 1;
}
