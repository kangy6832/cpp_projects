cmake_minimum_required(VERSION 3.16)
project(kdl_demo LANGUAGES CXX)

# ----------------------------------------------------------------------------
# 构建约定：
#   1. 所有中间产物（CMakeCache、对象文件、依赖文件等）放在 <项目根>/bin
#   2. 最终可执行文件放在项目根目录
#   因此请在项目根目录执行：
#       cmake -S . -B bin && cmake --build bin
# ----------------------------------------------------------------------------
if(CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR AND NOT ALLOW_IN_SOURCE_BUILD)
    message(FATAL_ERROR
        "禁止源码内构建。请使用：\n"
        "  cmake -S ${CMAKE_SOURCE_DIR} -B ${CMAKE_SOURCE_DIR}/bin\n"
        "  cmake --build ${CMAKE_SOURCE_DIR}/bin\n"
        "（如确实需要源码内构建，请加 -DALLOW_IN_SOURCE_BUILD=ON）")
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "构建类型" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 可执行文件输出到项目根目录
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
foreach(cfg IN ITEMS DEBUG RELEASE RELWITHDEBINFO MINSIZEREL)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${cfg} ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()

# 让可执行文件正常运行所依赖的动态库能被找到（kdl_parser 等在 /opt/ros/<distro>/lib）
set(CMAKE_SKIP_BUILD_RPATH OFF)
set(CMAKE_BUILD_WITH_INSTALL_RPATH OFF)

# ============================================================================
# 依赖查找
# ============================================================================
find_package(PkgConfig REQUIRED)

# 1) Orocos KDL（含 Eigen 头文件路径）
pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)

# 2) urdfdom（提供 <urdf_parser/urdf_parser.h>）
pkg_check_modules(URDFDOM REQUIRED urdfdom)

# 3) kdl_parser（ROS 2 提供，无 pkg-config，手工查找）
set(_kp_inc_hints "")
set(_kp_lib_hints "")
foreach(_prefix IN LISTS CMAKE_PREFIX_PATH)
    list(APPEND _kp_inc_hints "${_prefix}/include/kdl_parser")
    list(APPEND _kp_lib_hints "${_prefix}/lib")
endforeach()
if(DEFINED ENV{AMENT_PREFIX_PATH})
    string(REPLACE ":" ";" _ament_prefixes "$ENV{AMENT_PREFIX_PATH}")
    foreach(_prefix IN LISTS _ament_prefixes)
        list(APPEND _kp_inc_hints "${_prefix}/include/kdl_parser")
        list(APPEND _kp_lib_hints "${_prefix}/lib")
    endforeach()
endif()
file(GLOB _ros_distros LIST_DIRECTORIES true RELATIVE "/opt/ros" "/opt/ros/*")
foreach(_distro IN LISTS _ros_distros)
    list(APPEND _kp_inc_hints "/opt/ros/${_distro}/include/kdl_parser")
    list(APPEND _kp_lib_hints "/opt/ros/${_distro}/lib")
endforeach()

find_path(KDL_PARSER_INCLUDE_DIR
    NAMES kdl_parser/kdl_parser.hpp
    HINTS ${_kp_inc_hints}
    PATHS /usr/local/include/kdl_parser /usr/include/kdl_parser)

find_library(KDL_PARSER_LIBRARY
    NAMES kdl_parser
    HINTS ${_kp_lib_hints}
    PATHS /usr/local/lib /usr/lib/x86_64-linux-gnu)

if(NOT KDL_PARSER_INCLUDE_DIR OR NOT KDL_PARSER_LIBRARY)
    message(FATAL_ERROR
        "未找到 kdl_parser（ROS 2 包）。请先 source ROS 环境，例如：\n"
        "  source /opt/ros/jazzy/setup.bash")
endif()

get_filename_component(KDL_PARSER_LIB_DIR ${KDL_PARSER_LIBRARY} DIRECTORY)
message(STATUS "kdl_parser include: ${KDL_PARSER_INCLUDE_DIR}")
message(STATUS "kdl_parser library: ${KDL_PARSER_LIBRARY}")

# ============================================================================
# 目标
# ============================================================================
# src 下含 main() 的 .cpp 各自生成一个可执行文件；
# 不含 main() 的 .cpp（公共实现 / 未完成文件）只参与编译，汇入 kdl_common 静态库
file(GLOB ALL_SOURCES CONFIGURE_DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/src/Kinematics/*.cpp
     ${CMAKE_CURRENT_SOURCE_DIR}/src/Dynamics/*.cpp)

if(NOT ALL_SOURCES)
    message(FATAL_ERROR "src 目录下没有找到 .cpp 源文件")
endif()

set(EXE_SOURCES "")
set(COMMON_SOURCES "")
foreach(_src IN LISTS ALL_SOURCES)
    file(STRINGS ${_src} _has_main REGEX "^[ \t]*(int[ \t]+main[ \t]*\\()")
    if(_has_main)
        list(APPEND EXE_SOURCES ${_src})
    else()
        list(APPEND COMMON_SOURCES ${_src})
    endif()
endforeach()

# 统一的依赖配置
function(kdl_target_deps _tgt)
    target_include_directories(${_tgt} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${OROCOS_KDL_INCLUDE_DIRS}
        ${URDFDOM_INCLUDE_DIRS}
        ${KDL_PARSER_INCLUDE_DIR})

    target_link_directories(${_tgt} PRIVATE
        ${OROCOS_KDL_LIBRARY_DIRS}
        ${URDFDOM_LIBRARY_DIRS}
        ${KDL_PARSER_LIB_DIR})

    target_link_libraries(${_tgt} PRIVATE
        ${OROCOS_KDL_LIBRARIES}
        ${URDFDOM_LIBRARIES}
        ${KDL_PARSER_LIBRARY})

    # 默认 URDF 路径：命令行未指定时使用仓库内的 model/robotic_arm.urdf
    target_compile_definitions(${_tgt} PRIVATE
        DEFAULT_URDF_FILE="${CMAKE_CURRENT_SOURCE_DIR}/model/robotic_arm.urdf")
endfunction()

if(COMMON_SOURCES)
    add_library(kdl_common STATIC ${COMMON_SOURCES})
    kdl_target_deps(kdl_common)
endif()

foreach(_src IN LISTS EXE_SOURCES)
    get_filename_component(_target ${_src} NAME_WE)
    add_executable(${_target} ${_src})
    kdl_target_deps(${_target})
    if(COMMON_SOURCES)
        target_link_libraries(${_target} PRIVATE kdl_common)
    endif()
    message(STATUS "可执行文件: ${_target} <- ${_src}")
endforeach()
