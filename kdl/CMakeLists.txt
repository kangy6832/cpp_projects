cmake_minimum_required(VERSION 3.8)
project(kdl_tools)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 教学包使用的都是 C++17（KDL 本身要求 >= 14）。
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdf REQUIRED)        # urdf 包装层: urdf::Model / parseURDFFile
find_package(kdl_parser REQUIRED)        # kdl_parser: urdf -> KDL::Tree
find_package(orocos_kdl_vendor REQUIRED)  # orocos_kdl: KDL::Tree / KDL::Chain 本体

# ---------------------------------------------------------------------------
# 教学库：src/Tools 下的 URDF -> KDL 工具
# ---------------------------------------------------------------------------
add_library(kdl_tools_lib
  src/Tools/src/kdl_tools.cpp
  src/Tools/src/kdl_tools_print.cpp
)

# 本项目头文件规范：每个功能模块各自持有 include/ 与 src/，例如
#   src/Tools/include/kdl_tools.hpp   公共头（该模块的 include/ 即包含根）
#   src/Tools/src/kdl_tools.cpp       实现
# 因此构建期把 src/Tools/include 作为包含根，源码里直接 #include "kdl_tools.hpp"。
# 安装后统一落到 include/kdl_tools/，消费者路径保持不变。
# urdf 包只导出 urdf_INCLUDE_DIRS 变量，而且它是多元素列表，
# 不能包进 $<BUILD_INTERFACE:...>（分号会被当成参数分隔符），因此直接使用。
target_include_directories(kdl_tools_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Tools/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
  ${urdf_INCLUDE_DIRS}
)

target_link_libraries(kdl_tools_lib PUBLIC
  rclcpp::rclcpp
  kdl_parser::kdl_parser
  orocos-kdl
  # urdf 包只导出变量形式：需要显式链接 liburdf.so
  ${urdf_LIBRARIES}
)

# ---------------------------------------------------------------------------
# 教学库：src/Kinematics 下的 FK / 雅可比 / IK
# ---------------------------------------------------------------------------
add_library(kdl_kinematics_lib
  src/Kinematics/src/kdl_fk.cpp
  src/Kinematics/src/kdl_jacobian.cpp
  src/Kinematics/src/kdl_ik.cpp
  src/Kinematics/src/kdl_velocity.cpp
  src/Kinematics/src/kdl_acceleration.cpp
  src/Kinematics/src/kdl_kinematics_print.cpp
)

target_include_directories(kdl_kinematics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Kinematics/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
)

# Kinematics 只依赖 KDL 本身，与 Tools / urdf 完全解耦：
# 它接受的是已经建好的 KDL::Chain，这正是"解耦"在链接层面的体现。
# Eigen 的包含路径由 orocos-kdl 的 INTERFACE 依赖(Eigen3::Eigen)带进来。
target_link_libraries(kdl_kinematics_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
)

# ---------------------------------------------------------------------------
# 教学库：src/Dynamics 下的逆动力学 / 正动力学 / M-C-G 分解
# ---------------------------------------------------------------------------
add_library(kdl_dynamics_lib
  src/Dynamics/src/kdl_idynamics.cpp
  src/Dynamics/src/kdl_dynparam.cpp
  src/Dynamics/src/kdl_fdynamics.cpp
  src/Dynamics/src/kdl_dynamics_print.cpp
)

# 除了本模块的 include/，还需要 Kinematics 的 include/：
# "基座坐标系下的末端外力"必须先做一次正运动学拿到末端姿态，
# 所以 Dynamics 在链接层面依赖 Kinematics（见 kdl_idynamics.cpp 的注释）。
target_include_directories(kdl_dynamics_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Dynamics/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Kinematics/include>
)

target_link_libraries(kdl_dynamics_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
  kdl_kinematics_lib
)

# ---------------------------------------------------------------------------
# 教学库：src/Interpolation 下的插值
#   关节空间：五次多项式（kdl_quintic）
#   笛卡尔空间：位置五次多项式 + 姿态 slerp（kdl_cartesian）
# ---------------------------------------------------------------------------
add_library(kdl_interpolation_lib
  src/Interpolation/src/kdl_quintic.cpp
  src/Interpolation/src/kdl_interpolation_print.cpp
  src/Interpolation/src/kdl_cartesian.cpp
  src/Interpolation/src/kdl_cartesian_print.cpp
)

target_include_directories(kdl_interpolation_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Interpolation/include>
  $<INSTALL_INTERFACE:include/kdl_tools>
)

# 插值只吃"路点 + 段时长"，与 URDF、正逆运动学、动力学都无关：
# 关节空间给的是关节角数组，笛卡尔空间给的是 KDL::Frame 数组，两者都只依赖
# KDL 本身（kdl_cartesian 内部转调 kdl_quintic 的多段求解器，同属本模块，
# 不新增任何依赖）。
target_link_libraries(kdl_interpolation_lib PUBLIC
  rclcpp::rclcpp
  orocos-kdl
)

# ---------------------------------------------------------------------------
# 教学示例可执行文件
# ---------------------------------------------------------------------------
add_executable(build_from_urdf example/build_from_urdf.cpp)
# 把源码里的 model 目录路径编译进程序，示例即可在源码树下直接运行。
target_compile_definitions(build_from_urdf PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(build_from_urdf PRIVATE kdl_tools_lib)

# FK / 雅可比 / IK 示例：既用到 Tools（建链），也用到 Kinematics（求解）。
add_executable(fk_ik_demo example/fk_ik_demo.cpp)
target_compile_definitions(fk_ik_demo PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(fk_ik_demo PRIVATE kdl_tools_lib kdl_kinematics_lib)

# 速度 / 加速度转换示例：演示 J·q̇、J̇·q̇ 与伪逆求逆。
add_executable(vel_acc_demo example/vel_acc_demo.cpp)
target_compile_definitions(vel_acc_demo PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(vel_acc_demo PRIVATE kdl_tools_lib kdl_kinematics_lib)

# 动力学示例：逆解、正解、三明治公式拆分、末端外力与 RK4 仿真。
add_executable(dynamics_demo example/dynamics_demo.cpp)
target_compile_definitions(dynamics_demo PRIVATE
  KDL_TOOLS_MODEL_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/model"
)
target_link_libraries(dynamics_demo PRIVATE kdl_tools_lib kdl_dynamics_lib)

# 五次插值示例：单段系数、多段 C⁴ 拼接、约束校验与 CSV 导出。
# 刻意不定义 KDL_TOOLS_MODEL_DIR：插值只吃关节数组，不需要机器人模型。
add_executable(quintic_demo example/quintic_demo.cpp)
target_link_libraries(quintic_demo PRIVATE kdl_interpolation_lib)

# 笛卡尔插值示例：位置五次多项式 + 姿态 slerp、多段两种路点行为、限位校验与 CSV。
# 同样不定义 KDL_TOOLS_MODEL_DIR：笛卡尔插值只吃位姿数组，不需要机器人模型。
add_executable(cartesian_demo example/cartesian_demo.cpp)
target_link_libraries(cartesian_demo PRIVATE kdl_interpolation_lib)

# ---------------------------------------------------------------------------
# 安装
# ---------------------------------------------------------------------------
install(TARGETS kdl_tools_lib kdl_kinematics_lib kdl_dynamics_lib kdl_interpolation_lib
        build_from_urdf fk_ik_demo vel_acc_demo dynamics_demo quintic_demo cartesian_demo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY src/Tools/include/
  DESTINATION include/kdl_tools
)

install(DIRECTORY src/Kinematics/include/
  DESTINATION include/kdl_tools
)

install(DIRECTORY src/Dynamics/include/
  DESTINATION include/kdl_tools
)

install(DIRECTORY src/Interpolation/include/
  DESTINATION include/kdl_tools
)

# 把 URDF / STL 一起装到 share/kdl_tools/model，示例运行时按此路径查找。
install(DIRECTORY src/model/
  DESTINATION share/${PROJECT_NAME}/model
)

# ---------------------------------------------------------------------------
# 画图脚本：读 quintic_demo 导出的 CSV 画曲线
# ---------------------------------------------------------------------------
# 必须 RENAME 成不带扩展名的 plot_quintic：
# `ros2 run` 在 Linux 下要求 lib/<pkg>/ 里的文件名与命令名**完全一致**，
# 没有 .py 的回退（见 ros2run/api/__init__.py 的 get_executable_path）。
# install(PROGRAMS) 会顺手加上可执行权限，脚本里的 shebang 负责选解释器。
install(PROGRAMS src/Interpolation/scripts/plot_quintic.py
  RENAME plot_quintic
  DESTINATION lib/${PROJECT_NAME}
)

# 笛卡尔插值的画图脚本同理：读 cartesian_demo 导出的 CSV，出总览 + 分张 PNG。
install(PROGRAMS src/Interpolation/scripts/plot_cartesian.py
  RENAME plot_cartesian
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include/kdl_tools)
ament_export_libraries(kdl_tools_lib kdl_kinematics_lib kdl_dynamics_lib kdl_interpolation_lib)
ament_export_dependencies(rclcpp urdf kdl_parser orocos_kdl_vendor)

ament_package()
