/**
 * @file dynamics_utils.hpp
 * @brief 基于 KDL 的动力学工具函数集合（header-only）
 *
 * 与 Kinematics/kdl_utils.hpp 配套：后者负责 URDF 建模与运动学，
 * 本文件负责动力学所需的建模检查、惯量矩阵、科氏/离心项、重力项、逆动力学、
 * 雅可比，以及结果自检。
 *
 * 运动方程约定（与 KDL 一致）：
 * \f[ \tau = H(q)\,\ddot q + C(q,\dot q) + G(q) \f]
 *  - \f$H\f$ 关节空间惯量矩阵，KDL::JntSpaceInertiaMatrix
 *  - \f$C\f$ 科氏与离心力矩（已含 \f$\dot q\f$），KDL::JntArray
 *  - \f$G\f$ 重力力矩，KDL::JntArray
 *
 * 势能定义：\f$U(q) = -\sum_i m_i\, g^{T} p^{com}_i\f$，满足 \f$G = \partial U/\partial q\f$
 * （该关系已用中心差分与 KDL::ChainDynParam 交叉验证一致）。
 *
 * @note URDF 若缺少 <inertial> 标签（常见于 STL 导出的模型），下列动力学量会退化为 0，
 *       可先用 hasInertiaData() / printInertiaInfo() 检查模型数据是否完整。
 * @note 每个计算函数都会临时构造求解器；若在实时循环中调用，建议直接复用
 *       KDL::ChainDynParam / KDL::ChainJntToJacSolver 实例。
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl/chainjnttojacdotsolver.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntarrayvel.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/tree.hpp>

#include "../Kinematics/kdl_utils.hpp"

/**
 * @namespace dyn_utils
 * @brief 动力学建模与计算工具，配合 kdl_utils 使用
 */
namespace dyn_utils {

/// 默认重力加速度向量（基座标系，z 轴向上时为 -9.81）
inline const KDL::Vector kGravity(0.0, 0.0, -9.81);

/// 判定数值为零的阈值
inline constexpr double kEpsilon = 1e-9;

// ============================================================================
// 模型建立与惯量数据检查
// ============================================================================

/**
 * @brief 统计各段的惯量数据是否齐全
 * @param[in] chain 运动链
 * @return 总质量大于零、且每个运动段都有非零质量与转动惯量时返回 true
 * @note 固定段（Joint::None）允许零质量——URDF 中常见的虚拟连杆
 */
inline bool hasInertiaData(const KDL::Chain& chain)
{
    const double total = [&chain] {
        double m = 0.0;
        for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
            m += chain.getSegment(i).getInertia().getMass();
        }
        return m;
    }();
    if (total <= kEpsilon) return false;

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& seg = chain.getSegment(i);
        if (seg.getJoint().getType() == KDL::Joint::None) continue;

        const KDL::RigidBodyInertia I = seg.getInertia();
        if (I.getMass() <= kEpsilon) return false;
        // 检查惯性张量在对角线上的分量是否全为零
        const KDL::RotationalInertia Ic = I.getRotationalInertia();
        const double trace = Ic.data[0] + Ic.data[4] + Ic.data[8];
        if (std::fabs(trace) <= kEpsilon) return false;
    }
    return true;
}

/**
 * @brief 从 URDF 建立可用于动力学计算的运动链（内含惯量数据校验）
 * @param[in]  urdf_file URDF 文件路径
 * @param[in]  root_link 根连杆名
 * @param[in]  tip_link  末端连杆名
 * @param[out] chain     提取得到的运动链
 * @param[out] model     URDF 模型句柄，可用于读取关节限位等
 * @return 建模成功且惯量数据有效返回 true；否则返回 false
 * @note 相比 kdl_utils::buildChainFromFile() 多了惯性参数检查，避免出现
 *       "运动学正确但动力学量全为 0" 的情况
 */
inline bool buildDynamicsChain(const std::string& urdf_file,
                               const std::string& root_link,
                               const std::string& tip_link,
                               KDL::Chain& chain,
                               urdf::ModelInterfaceSharedPtr& model)
{
    if (!kdl_utils::buildChainFromFile(urdf_file, root_link, tip_link, chain, model)) {
        return false;
    }
    if (!hasInertiaData(chain)) {
        std::cerr << "错误: URDF 缺少可用的惯性参数（<inertial> 标签），无法进行动力学计算"
                  << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 求整条运动链的总质量
 * @param[in] chain 运动链
 * @return 所有段质量之和（kg）
 */
inline double totalMass(const KDL::Chain& chain)
{
    double m = 0.0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        m += chain.getSegment(i).getInertia().getMass();
    }
    return m;
}

/**
 * @brief 打印每个连杆的质量、质心偏移与转动惯量对角线元素
 * @param[in] chain 运动链
 */
inline void printInertiaInfo(const KDL::Chain& chain)
{
    std::cout << "惯量参数 (总质量 = " << totalMass(chain) << " kg):" << std::endl;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::Segment& seg = chain.getSegment(i);
        const KDL::RigidBodyInertia I = seg.getInertia();
        const KDL::Vector cog = I.getCOG();
        const KDL::RotationalInertia Ic = I.getRotationalInertia();

        std::cout << "  [" << i << "] " << seg.getName()
                  << " m=" << I.getMass() << " kg"
                  << " cog=(" << cog.x() << ", " << cog.y() << ", " << cog.z() << ")"
                  << " I=( " << Ic.data[0] << ", " << Ic.data[4] << ", " << Ic.data[8] << " )"
                  << std::endl;
    }
}

// ============================================================================
// 动力学量计算
// ============================================================================

/**
 * @brief 计算关节空间惯量矩阵 H(q)
 * @param[in]  chain   运动链
 * @param[in]  q       关节角，长度需与 chain 关节数一致
 * @param[out] H       n×n 惯量矩阵
 * @param[in]  gravity 重力加速度向量（KDL::ChainDynParam 构造需要，不影响 H）
 * @return 成功返回 true；尺寸不符或求解失败返回 false
 */
inline bool massMatrix(const KDL::Chain& chain, const KDL::JntArray& q,
                       KDL::JntSpaceInertiaMatrix& H,
                       const KDL::Vector& gravity = kGravity)
{
    KDL::ChainDynParam dyn(chain, gravity);
    if (int ret = dyn.JntToMass(q, H); ret < 0) {
        std::cerr << "惯量矩阵计算失败: " << dyn.strError(ret) << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 计算科氏与离心力矩 C(q, qd)（已含 qd，即返回的是矩阵乘过速度后的矢量）
 * @param[in]  chain 运动链
 * @param[in]  q     关节角
 * @param[in]  qd    关节角速度
 * @param[out] C     科氏/离心力矩矢量
 * @param[in]  gravity 重力加速度向量
 * @return 成功返回 true；否则返回 false
 * @note KDL 的 JntToCoriolis 内部使用零重力，因此结果与 gravity 无关
 */
inline bool coriolisTorques(const KDL::Chain& chain, const KDL::JntArray& q,
                            const KDL::JntArray& qd, KDL::JntArray& C,
                            const KDL::Vector& gravity = kGravity)
{
    KDL::ChainDynParam dyn(chain, gravity);
    if (int ret = dyn.JntToCoriolis(q, qd, C); ret < 0) {
        std::cerr << "科氏力矩计算失败: " << dyn.strError(ret) << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 计算重力力矩 G(q)
 * @param[in]  chain   运动链
 * @param[in]  q       关节角
 * @param[out] G       重力力矩矢量
 * @param[in]  gravity 重力加速度向量，默认 (0, 0, -9.81)
 * @return 成功返回 true；否则返回 false
 */
inline bool gravityTorques(const KDL::Chain& chain, const KDL::JntArray& q,
                           KDL::JntArray& G, const KDL::Vector& gravity = kGravity)
{
    KDL::ChainDynParam dyn(chain, gravity);
    if (int ret = dyn.JntToGravity(q, G); ret < 0) {
        std::cerr << "重力力矩计算失败: " << dyn.strError(ret) << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 逆动力学：\f$\tau = H(q)\ddot q + C(q,\dot q) + G(q)\f$
 * @param[in]  chain   运动链
 * @param[in]  q       关节角
 * @param[in]  qd      关节角速度
 * @param[in]  qdd     关节角加速度
 * @param[out] tau     所需关节力矩
 * @param[in]  gravity 重力加速度向量
 * @return 成功返回 true；任一分项失败返回 false
 */
inline bool inverseDynamics(const KDL::Chain& chain, const KDL::JntArray& q,
                            const KDL::JntArray& qd, const KDL::JntArray& qdd,
                            KDL::JntArray& tau, const KDL::Vector& gravity = kGravity)
{
    const unsigned int n = chain.getNrOfJoints();
    KDL::JntSpaceInertiaMatrix H(n);
    KDL::JntArray C(n), G(n);
    if (!massMatrix(chain, q, H, gravity)) return false;
    if (!coriolisTorques(chain, q, qd, C, gravity)) return false;
    if (!gravityTorques(chain, q, G, gravity)) return false;

    tau.resize(n);
    tau.data = H.data * qdd.data + C.data + G.data;
    return true;
}

/**
 * @brief 计算末端雅可比 J(q)（参考系：基座；参考点：末端）
 * @param[in]  chain 运动链
 * @param[in]  q     关节角
 * @param[out] J     6×n 雅可比矩阵
 * @param[in]  seg_nr 计算到哪一段，默认 -1 表示整条链的末端
 * @return 成功返回 true；否则返回 false
 */
inline bool jacobian(const KDL::Chain& chain, const KDL::JntArray& q,
                     KDL::Jacobian& J, int seg_nr = -1)
{
    KDL::ChainJntToJacSolver solver(chain);
    if (int ret = solver.JntToJac(q, J, seg_nr); ret < 0) {
        std::cerr << "雅可比计算失败: " << solver.strError(ret) << std::endl;
        return false;
    }
    return true;
}

/**
 * @brief 计算雅可比的时间导数 \f$\dot J(q,\dot q)\f$
 * @param[in]  chain 运动链
 * @param[in]  q     关节角
 * @param[in]  qd    关节角速度
 * @param[out] Jdot  6×n 雅可比导数
 * @param[in]  seg_nr 计算到哪一段，默认 -1 表示末端
 * @return 成功返回 true；否则返回 false
 */
inline bool jacobianDot(const KDL::Chain& chain, const KDL::JntArray& q,
                        const KDL::JntArray& qd, KDL::Jacobian& Jdot, int seg_nr = -1)
{
    KDL::ChainJntToJacDotSolver solver(chain);
    const KDL::JntArrayVel qv(q, qd);
    if (int ret = solver.JntToJacDot(qv, Jdot, seg_nr); ret < 0) {
        std::cerr << "雅可比导数计算失败: " << solver.strError(ret) << std::endl;
        return false;
    }
    return true;
}

// ============================================================================
// 质量属性与能量
// ============================================================================

/**
 * @brief 整链的质量属性（质量与质心）
 */
struct MassProperties
{
    double mass{};       ///< 总质量 (kg)
    KDL::Vector com;     ///< 基座标系下的质心位置 (m)
};

/**
 * @brief 计算给定构型下整链的总质量与基座标系下的质心位置
 * @param[in] chain 运动链
 * @param[in] q     关节角
 * @return 质量属性结构体；若总质量为 0，质心返回零向量
 * @note KDL 中每段的 RigidBodyInertia 以该段 tip 坐标系为参考；
 *       质心位置由此变换到基座标系：p = frame_tip * I.getCOG()
 */
inline MassProperties massProperties(const KDL::Chain& chain, const KDL::JntArray& q)
{
    const std::vector<KDL::Frame> frames = kdl_utils::linkFrames(chain, q);

    double mass = 0.0;
    KDL::Vector first_moment = KDL::Vector::Zero();
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::RigidBodyInertia I = chain.getSegment(i).getInertia();
        const double m = I.getMass();
        mass += m;
        first_moment = first_moment + m * (frames[i] * I.getCOG());
    }

    MassProperties mp;
    mp.mass = mass;
    mp.com = (mass > kEpsilon) ? first_moment / mass : KDL::Vector::Zero();
    return mp;
}

/**
 * @brief 计算系统势能 \f$U(q) = -\sum_i m_i g^{T} p^{com}_i\f$
 * @param[in] chain   运动链
 * @param[in] q       关节角
 * @param[in] gravity 重力加速度向量
 * @return 势能 (J)
 */
inline double potentialEnergy(const KDL::Chain& chain, const KDL::JntArray& q,
                              const KDL::Vector& gravity = kGravity)
{
    const std::vector<KDL::Frame> frames = kdl_utils::linkFrames(chain, q);

    double U = 0.0;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        const KDL::RigidBodyInertia I = chain.getSegment(i).getInertia();
        U -= I.getMass() * KDL::dot(gravity, frames[i] * I.getCOG());
    }
    return U;
}

/**
 * @brief 计算动能 \f$T = \frac12 \dot q^{T} H(q) \dot q\f$
 * @param[in] H  惯量矩阵
 * @param[in] qd 关节角速度
 * @return 动能 (J)
 */
inline double kineticEnergy(const KDL::JntSpaceInertiaMatrix& H, const KDL::JntArray& qd)
{
    return 0.5 * qd.data.dot(H.data * qd.data);
}

// ============================================================================
// 数值自检
// ============================================================================

/**
 * @brief 惯量矩阵的性质统计
 */
struct MassMatrixInfo
{
    bool symmetric{false};   ///< 是否满足对称性
    double asymmetry{0.0};   ///< 最大非对称偏差 max|H_ij - H_ji|
    double min_eigen{0.0};   ///< 最小特征值，反映正定性
    double max_eigen{0.0};   ///< 最大特征值
    double condition{0.0};   ///< 条件数 |λmax/λmin|
};

/**
 * @brief 检查惯量矩阵的对称性、正定性与条件数
 * @param[in] H       惯量矩阵
 * @param[in] sym_tol 对称性判定阈值
 * @return 性质统计结构体
 * @note λmin > 0 说明 H 正定；条件数过大意味着该构型接近奇异，
 *       逆动力学/基于 H 的控制在该处数值脆弱
 */
inline MassMatrixInfo massMatrixInfo(const KDL::JntSpaceInertiaMatrix& H,
                                     double sym_tol = 1e-9)
{
    MassMatrixInfo info;
    const Eigen::Index n = H.data.rows();

    double max_asym = 0.0;
    for (Eigen::Index i = 0; i < n; ++i) {
        for (Eigen::Index j = 0; j < n; ++j) {
            max_asym = std::max(max_asym, std::fabs(H.data(i, j) - H.data(j, i)));
        }
    }
    info.asymmetry = max_asym;
    info.symmetric = max_asym <= sym_tol;

    const Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(H.data);
    const Eigen::VectorXd ev = solver.eigenvalues();
    info.min_eigen = ev.minCoeff();
    info.max_eigen = ev.maxCoeff();
    info.condition = (std::fabs(info.min_eigen) > kEpsilon)
                         ? std::fabs(info.max_eigen / info.min_eigen)
                         : std::numeric_limits<double>::infinity();
    return info;
}

/**
 * @brief 校验重力力矩与势能梯度的一致性：\f$G \stackrel{?}{=} \partial U/\partial q\f$
 * @param[in]  chain    运动链
 * @param[in]  q        关节角
 * @param[out] max_error 各关节上 |G - ∂U/∂q| 的最大值
 * @param[in]  gravity  重力加速度向量
 * @param[in]  h        中心差分步长
 * @return 计算成功返回 true；否则返回 false
 * @note 这是检验 URDF 动力学参数与 KDL 求解结果是否自洽的最直接手段
 */
inline bool checkGravityEnergy(const KDL::Chain& chain, const KDL::JntArray& q,
                               double& max_error, const KDL::Vector& gravity = kGravity,
                               double h = 1e-6)
{
    KDL::JntArray G(chain.getNrOfJoints());
    if (!gravityTorques(chain, q, G, gravity)) return false;

    max_error = 0.0;
    for (unsigned int i = 0; i < chain.getNrOfJoints(); ++i) {
        KDL::JntArray qp(q), qm(q);
        qp(i) += h;
        qm(i) -= h;
        const double dUdq =
            (potentialEnergy(chain, qp, gravity) - potentialEnergy(chain, qm, gravity)) /
            (2.0 * h);
        max_error = std::max(max_error, std::fabs(G(i) - dUdq));
    }
    return true;
}

/**
 * @brief 校验逆动力学组合结果与 KDL 标准 RNE 求解器是否一致
 * @param[in]  chain     运动链
 * @param[in]  q         关节角
 * @param[in]  qd        关节角速度
 * @param[in]  qdd       关节角加速度
 * @param[out] max_error 各关节力矩的最大绝对偏差
 * @param[in]  gravity   重力加速度向量
 * @return 两者一致（误差小于 tol）返回 true；否则返回 false
 */
inline bool checkInverseDynamics(const KDL::Chain& chain, const KDL::JntArray& q,
                                 const KDL::JntArray& qd, const KDL::JntArray& qdd,
                                 double& max_error, const KDL::Vector& gravity = kGravity,
                                 double tol = 1e-6)
{
    KDL::JntArray tau_id(chain.getNrOfJoints());
    if (!inverseDynamics(chain, q, qd, qdd, tau_id, gravity)) return false;

    KDL::ChainIdSolver_RNE rne(chain, gravity);
    KDL::JntArray tau_rne(chain.getNrOfJoints());
    KDL::Wrenches f_ext(chain.getNrOfSegments(), KDL::Wrench::Zero());
    if (int ret = rne.CartToJnt(q, qd, qdd, f_ext, tau_rne); ret < 0) {
        std::cerr << "RNE 逆动力学失败: " << rne.strError(ret) << std::endl;
        return false;
    }

    max_error = (tau_id.data - tau_rne.data).cwiseAbs().maxCoeff();
    return max_error < tol;
}

/**
 * @brief 检验 \f$\dot H - 2C\f$ 的反对称性（以二阶形式 \f$\dot q^T(\dot H\dot q - 2c) = 0\f$ 验证）
 * @param[in] chain   运动链
 * @param[in] q       关节角
 * @param[in] qd      关节角速度
 * @param[in] gravity 重力加速度向量
 * @param[in] h       中心差分步长，用于沿 \f$\dot q\f$ 方向求 \f$\dot H\f$
 * @return 二阶形式的残差值，接近 0 表示满足反对称性；计算失败返回 NaN
 * @note 该性质是 \f$\dot H - 2C\f$ 反对称的必要条件，常用于验证 C 项实现是否正确
 */
inline double coriolisSkewError(const KDL::Chain& chain, const KDL::JntArray& q,
                                const KDL::JntArray& qd,
                                const KDL::Vector& gravity = kGravity, double h = 1e-6)
{
    const unsigned int n = chain.getNrOfJoints();
    KDL::JntSpaceInertiaMatrix Hp(n), Hm(n);

    KDL::JntArray qp(n), qm(n);
    for (unsigned int i = 0; i < n; ++i) {
        qp(i) = q(i) + h * qd(i);
        qm(i) = q(i) - h * qd(i);
    }
    if (!massMatrix(chain, qp, Hp, gravity)) return std::nan("");
    if (!massMatrix(chain, qm, Hm, gravity)) return std::nan("");

    const Eigen::VectorXd Hdot_qd = ((Hp.data - Hm.data) / (2.0 * h)) * qd.data;

    KDL::JntArray C(n);
    if (!coriolisTorques(chain, q, qd, C, gravity)) return std::nan("");

    return std::fabs(qd.data.dot(Hdot_qd - 2.0 * C.data));
}

// ============================================================================
// 输出工具
// ============================================================================

/**
 * @brief 打印关节量（力矩/角度/速度等）
 * @param[in] name  标签名
 * @param[in] v     待打印的关节数组
 * @param[in] scale 输出时的缩放系数（如弧度转角度用 180/pi）
 * @param[in] unit  单位字符串，为空时不附加单位
 */
inline void printVector(const std::string& name, const KDL::JntArray& v,
                        double scale = 1.0, const std::string& unit = "")
{
    std::cout << "  " << name << ":";
    for (unsigned int i = 0; i < v.rows(); ++i) std::cout << " " << v(i) * scale;
    if (!unit.empty()) std::cout << " (" << unit << ")";
    std::cout << std::endl;
}

/**
 * @brief 打印 n×n 关节空间惯量矩阵
 * @param[in] name 标签名
 * @param[in] H    惯量矩阵
 */
inline void printMatrix(const std::string& name, const KDL::JntSpaceInertiaMatrix& H)
{
    std::cout << "  " << name << " (" << H.data.rows() << "x" << H.data.cols() << "):"
              << std::endl;
    for (Eigen::Index r = 0; r < H.data.rows(); ++r) {
        std::cout << "    [";
        for (Eigen::Index c = 0; c < H.data.cols(); ++c) {
            std::cout << std::setw(12) << H.data(r, c);
        }
        std::cout << " ]" << std::endl;
    }
}

/**
 * @brief 打印任意 Eigen 矩阵
 * @param[in] name 标签名
 * @param[in] M    待打印矩阵
 */
inline void printMatrix(const std::string& name, const Eigen::MatrixXd& M)
{
    std::cout << "  " << name << " (" << M.rows() << "x" << M.cols() << "):" << std::endl;
    for (Eigen::Index r = 0; r < M.rows(); ++r) {
        std::cout << "    [";
        for (Eigen::Index c = 0; c < M.cols(); ++c) {
            std::cout << std::setw(12) << M(r, c);
        }
        std::cout << " ]" << std::endl;
    }
}

/**
 * @brief 打印雅可比矩阵（6×n）
 * @param[in] name 标签名
 * @param[in] J    雅可比矩阵
 */
inline void printJacobian(const std::string& name, const KDL::Jacobian& J)
{
    std::cout << "  " << name << " (" << J.rows() << "x" << J.columns() << "):"
              << std::endl;
    for (unsigned int r = 0; r < J.rows(); ++r) {
        std::cout << "    [";
        for (unsigned int c = 0; c < J.columns(); ++c) {
            std::cout << std::setw(12) << J(r, c);
        }
        std::cout << " ]" << std::endl;
    }
}

}  // namespace dyn_utils
