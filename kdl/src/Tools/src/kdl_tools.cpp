// Copyright (c) 2026, kdl_tools authors.
// 教学用途：kdl_tools.hpp 中声明的函数在这里落地。
//
// 每个函数都刻意写得很短，只负责"把数据从一种类型搬到另一种类型"，
// 真正的算法都在库内部（urdfdom / kdl_parser / orocos_kdl）。

#include "kdl_tools.hpp"

#include <memory>
#include <string>

#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/logging.hpp>
#include <urdf/model.h>

namespace kdl_tools
{

// ---------------------------------------------------------------------------
// 内部辅助：统一处理 bool 返回值 + 日志，避免在每个函数里重复写 if/else。
// ---------------------------------------------------------------------------
namespace
{

/// 所有日志统一前缀，方便在终端里一眼看出是谁打的。
constexpr const char * kLogTag = "kdl_tools";

/**
 * @brief 根据解析结果打印统一格式的日志。
 * @param ok       解析是否成功。
 * @param what     正在做的事（例如 "parse URDF file"）。
 * @param target   操作对象（例如文件路径）。
 * @return 原样返回 ok，方便调用方直接 `return report(...)`。
 */
bool report(bool ok, const char * what, const std::string & target)
{
  if (ok) {
    RCLCPP_INFO(rclcpp::get_logger(kLogTag), "%s succeeded: %s", what, target.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(kLogTag), "%s failed: %s", what, target.c_str());
  }
  return ok;
}

}  // namespace

// ---------------------------------------------------------------------------
// 一、URDF 文本 -> KDL::Tree
// ---------------------------------------------------------------------------

bool buildTreeFromUrdfModel(
  const urdf::ModelInterface & robot_model, KDL::Tree & tree)
{
  // kdl_parser 对每个 urdf link/joint 调用 addSegment()，层层拼出一棵树。
  const bool ok = kdl_parser::treeFromUrdfModel(robot_model, tree);
  return report(ok, "kdl_parser::treeFromUrdfModel", robot_model.getName());
}

bool buildTreeFromUrdfString(const std::string & urdf_xml, KDL::Tree & tree)
{
  // urdf::Model 继承自 urdf::ModelInterface，用 shared_ptr 持有即可直接
  // 传给 kdl_parser。其 initString() 内部通过 urdf_parser_plugin 解析 XML。
  std::shared_ptr<urdf::Model> model = std::make_shared<urdf::Model>();
  if (!model->initString(urdf_xml)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "urdf::Model::initString failed: invalid URDF xml");
    return false;
  }

  // 第二步：urdf::ModelInterface -> KDL::Tree。
  return buildTreeFromUrdfModel(*model, tree);
}

bool buildTreeFromUrdfFile(const std::string & urdf_file, KDL::Tree & tree)
{
  // 第一步：从磁盘读 XML 并解析成 urdf::Model（继承 ModelInterface）。
  std::shared_ptr<urdf::Model> model = std::make_shared<urdf::Model>();
  if (!model->initFile(urdf_file)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag), "urdf::Model::initFile failed: %s",
      urdf_file.c_str());
    return false;
  }

  // 第二步：urdf::ModelInterface -> KDL::Tree。
  return buildTreeFromUrdfModel(*model, tree);
}

// ---------------------------------------------------------------------------
// 二、Tree -> KDL::Chain
// ---------------------------------------------------------------------------

bool buildChain(
  const KDL::Tree & tree, KDL::Chain & chain,
  const std::string & base_link, const std::string & tip_link)
{
  // getChain 在树中寻找 base_link -> tip_link 的唯一连通路径，
  // 并把沿途的 Segment 依次塞进 chain（含固定段）。
  const bool ok = tree.getChain(base_link, tip_link, chain);
  if (ok) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLogTag), "Chain built: %s -> %s (%u segments, %u joints)",
      base_link.c_str(), tip_link.c_str(),
      static_cast<unsigned>(chain.getNrOfSegments()),
      static_cast<unsigned>(chain.getNrOfJoints()));
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogTag),
      "Tree::getChain failed: no connected path %s -> %s",
      base_link.c_str(), tip_link.c_str());
  }
  return ok;
}

bool buildChainFromUrdfFile(
  const std::string & urdf_file, KDL::Chain & chain,
  const std::string & base_link, const std::string & tip_link)
{
  // 组合拳：两步都成功后 chain 才是有效的。
  KDL::Tree tree("robot");
  if (!buildTreeFromUrdfFile(urdf_file, tree)) {
    return false;
  }
  return buildChain(tree, chain, base_link, tip_link);
}

// ---------------------------------------------------------------------------
// 三、辅助：加载 urdf::Model
// ---------------------------------------------------------------------------

bool loadUrdfModel(const std::string & urdf_file, urdf::Model & model)
{
  // urdf::Model::initFile 是 urdfdom 提供的高层封装，
  // 等价于 parseURDFFile() 再拷贝成具体类型 urdf::Model。
  const bool ok = model.initFile(urdf_file);
  return report(ok, "urdf::Model::initFile", urdf_file);
}

}  // namespace kdl_tools
