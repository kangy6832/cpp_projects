// Copyright (c) 2026, kdl_tools authors.
// 教学用途：kdl_tools_print.hpp 中声明的打印函数在这里实现。
//
// 只用标准库的 std::ostream，不依赖 ROS 日志，
// 因此这些函数既能直接 cout 到屏幕，也能写进 stringstream 做单元测试。

#include "kdl_tools_print.hpp"

#include <cmath>
#include <iostream>
#include <map>
#include <string>
#include <vector>

namespace kdl_tools
{
namespace
{

/**
 * @brief 把 KDL 的关节类型枚举翻译成人类可读的字符串。
 * @param type KDL::Joint::Type 枚举值。
 * @return 对应的名字，例如 "revolute"、"fixed"。
 */
const char * jointTypeName(KDL::Joint::JointType type)
{
  switch (type) {
    case KDL::Joint::RotAxis:      return "RotAxis(绕轴转)";
    case KDL::Joint::RotX:         return "RotX(绕X转)";
    case KDL::Joint::RotY:         return "RotY(绕Y转)";
    case KDL::Joint::RotZ:         return "RotZ(绕Z转)";
    case KDL::Joint::TransAxis:    return "TransAxis(沿轴平移)";
    case KDL::Joint::TransX:       return "TransX(沿X平移)";
    case KDL::Joint::TransY:       return "TransY(沿Y平移)";
    case KDL::Joint::TransZ:       return "TransZ(沿Z平移)";
    case KDL::Joint::None:         return "None(固定/fixed)";
    default:                       return "Unknown";
  }
}

/// 打印用的短前缀，统一风格：[Joint] / [Segment] / [Chain] / [Tree]。
const char * kJointTag = "[Joint]   ";
const char * kSegmentTag = "[Segment] ";

/**
 * @brief 把浮点噪声清理成漂亮的 0 / ±1，只用于打印展示。
 * @param value 原始数值。
 * @return 接近 0 或 ±1 时返回精确值，否则原样返回。
 *
 * @note URDF 里的 rpy 经过四元数算出来的旋转矩阵常带 1e-6 级别的残差，
 *       直接打印会干扰阅读，所以这里做一次"仅供显示"的取整。
 */
double tidy(double value)
{
  constexpr double kEps = 1e-9;
  if (std::abs(value) < kEps) {
    return 0.0;
  }
  if (std::abs(value - 1.0) < kEps) {
    return 1.0;
  }
  if (std::abs(value + 1.0) < kEps) {
    return -1.0;
  }
  return value;
}

/**
 * @brief 打印树的一个分枝（递归辅助函数）。
 * @param it    指向当前树元素的迭代器；*it->second 即 KDL::TreeElement。
 * @param depth 当前递归深度，用于生成缩进。
 * @param os    输出流。
 *
 * @note KDL::Tree 内部是 std::map<std::string, TreeElement>，
 *       每个 TreeElement 持有本段的 Segment 以及所有子节点的迭代器，
 *       所以"顺着 children 迭代器往下走"即可遍历整棵树。
 */
void printBranch(KDL::SegmentMap::const_iterator it, int depth, std::ostream & os)
{
  const KDL::Segment & segment = it->second.segment;
  const KDL::Joint & joint = segment.getJoint();

  std::string indent(static_cast<std::size_t>(depth) * 2, ' ');
  os << indent << "└─ " << joint.getName()
     << " (" << jointTypeName(joint.getType()) << ")  -> " << segment.getName() << "\n";

  // 递归打印所有子段：children 的元素本身就是指向子节点的迭代器
  //（注意它不是 std::map 的 value_type，因此没有 .second）。
  for (const KDL::SegmentMap::const_iterator & child : it->second.children) {
    printBranch(child, depth + 1, os);
  }
}

}  // namespace

// ---------------------------------------------------------------------------
// 叶子级
// ---------------------------------------------------------------------------

void printJoint(const KDL::Joint & joint, std::ostream & os)
{
  os << kJointTag
     << "name = " << joint.getName()
     << ", type = " << jointTypeName(joint.getType());

  // 固定关节没有关节轴，这里只对可动关节展示旋转轴/平移轴。
  // JointAxis() 返回的轴已经过 scale 归一化，学生看到的即单位方向。
  if (joint.getType() != KDL::Joint::None) {
    const KDL::Vector axis = joint.JointAxis();
    os << ", axis = [" << tidy(axis.x()) << ", " << tidy(axis.y()) << ", "
       << tidy(axis.z()) << "]";
  }
  os << "\n";
}

void printSegment(const KDL::Segment & segment, std::ostream & os)
{
  os << kSegmentTag << "name = " << segment.getName() << "\n";
  printJoint(segment.getJoint(), os);

  // pose(q) 表示"当关节变量为 q 时，从本段坐标系到末端坐标系的变换"。
  // q = 0 时得到的就是该段的静态安装位姿（FrameToTip）。
  const KDL::Frame frame_to_tip = segment.pose(0.0);
  os << "           frame_to_tip(0) origin = ["
     << tidy(frame_to_tip.p.x()) << ", " << tidy(frame_to_tip.p.y()) << ", "
     << tidy(frame_to_tip.p.z()) << "]\n";

  // KDL::Rotation 没有 operator<<，用 operator()(i, j) 逐元素取出 3x3 矩阵。
  os << "           frame_to_tip(0) rotation =\n";
  for (int i = 0; i < 3; ++i) {
    os << "             [";
    for (int j = 0; j < 3; ++j) {
      os << tidy(frame_to_tip.M(i, j)) << (j < 2 ? ", " : "");
    }
    os << "]\n";
  }
}

// ---------------------------------------------------------------------------
// 容器级
// ---------------------------------------------------------------------------

void printChain(const KDL::Chain & chain, std::ostream & os)
{
  os << "===== KDL::Chain: " << chain.getNrOfSegments() << " segments, "
     << chain.getNrOfJoints() << " joints =====\n";

  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    os << "  [" << i << "] ";
    printSegment(chain.getSegment(i), os);
  }

  // 顺便做一个关节类型统计：固定段(None) 与 可动段的分布。
  // 注意：可动关节按顺序排在段列表里，但固定段不占 joint 下标，
  // 所以这里遍历的是"所有段"，再按关节类型判断是否需要计数。
  std::map<std::string, int> type_count;
  for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
    const KDL::Joint & joint = chain.getSegment(i).getJoint();
    if (joint.getType() != KDL::Joint::None) {
      type_count[jointTypeName(joint.getType())]++;
    }
  }
  os << "----- joint type summary -----\n";
  for (const auto & kv : type_count) {
    os << "  " << kv.first << " : " << kv.second << "\n";
  }
  os << "=================================================\n";
}

void printTree(const KDL::Tree & tree, std::ostream & os)
{
  os << "===== KDL::Tree: " << tree.getNrOfSegments() << " segments, "
     << tree.getNrOfJoints() << " joints =====\n";

  KDL::SegmentMap::const_iterator root = tree.getRootSegment();
  if (root == tree.getSegments().end()) {
    os << "  (empty tree)\n";
    return;
  }

  // 根段本身没有父关节，只打印它对应的 link 名（world / base_link）。
  os << "  [root] " << root->second.segment.getName() << "\n";
  for (const KDL::SegmentMap::const_iterator & child : root->second.children) {
    printBranch(child, 1, os);
  }
  os << "=================================================\n";
}

}  // namespace kdl_tools
