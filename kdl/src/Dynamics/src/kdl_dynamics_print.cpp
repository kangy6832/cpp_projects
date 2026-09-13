// Copyright (c) 2026, kdl_dynamics authors.
// 教学用途：kdl_dynamics_print.hpp 中声明的函数在这里落地。
//
// 这里的量都是有明确物理意义的（N·m、N、rad/s²、kg·m²），
// 所以一律**原样打印**、不做任何"浮点美化"——数值本身就是要观察的对象。

#include "kdl_dynamics_print.hpp"

#include <ostream>
#include <string>

namespace kdl_dynamics
{
namespace
{

/**
 * @brief 内部辅助：打印一行"标签 + 关节向量"。
 * @param jnt   待打印的关节向量。
 * @param label 标签，例如 "torque (N·m)"。
 * @param os    输出流。
 */
void printJointVector(const KDL::JntArray & jnt, const char * label, std::ostream & os)
{
  os << "  " << label << " = [";
  for (unsigned int i = 0; i < jnt.rows(); ++i) {
    os << jnt(i) << (i + 1 < jnt.rows() ? ", " : "");
  }
  os << "]\n";
}

}  // namespace

void printMassMatrix(const KDL::JntSpaceInertiaMatrix & mass, std::ostream & os)
{
  os << "[MassMatrix] " << mass.rows() << " x " << mass.columns()
     << "  (物理上必然对称且正定)\n";
  for (unsigned int i = 0; i < mass.rows(); ++i) {
    os << "  [";
    for (unsigned int j = 0; j < mass.columns(); ++j) {
      os << mass(i, j) << (j + 1 < mass.columns() ? ", " : "");
    }
    os << "]\n";
  }
}

void printWrench(const KDL::Wrench & wrench, std::ostream & os)
{
  os << "[Wrench]\n";
  os << "  force  = [" << wrench.force.x() << ", " << wrench.force.y() << ", "
     << wrench.force.z() << "]\n";
  os << "  torque = [" << wrench.torque.x() << ", " << wrench.torque.y() << ", "
     << wrench.torque.z() << "]\n";
}

void printIdResult(const IdResult & result, std::ostream & os)
{
  os << "[IdResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  printJointVector(result.torque, "torque (N·m)", os);
}

void printFdResult(const FdResult & result, std::ostream & os)
{
  os << "[FdResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  printJointVector(result.qddot, "qddot (rad/s²)", os);
}

void printMassResult(const MassResult & result, std::ostream & os)
{
  os << "[MassResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  printMassMatrix(result.mass, os);
}

void printCoriolisResult(const CoriolisResult & result, std::ostream & os)
{
  os << "[CoriolisResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  printJointVector(result.torque, "C(q,qdot)*qdot (N·m)", os);
}

void printGravityResult(const GravityResult & result, std::ostream & os)
{
  os << "[GravityResult] " << (result.success() ? "SUCCESS" : "FAILED")
     << "  error_code = " << result.error_code
     << ", message = \"" << result.message << "\"\n";
  printJointVector(result.torque, "G(q) (N·m)", os);
}

}  // namespace kdl_dynamics
