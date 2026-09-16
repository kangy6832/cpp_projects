#!/usr/bin/env python3
# Copyright (c) 2026, kdl_interpolation authors.
# 教学用途：把 cartesian_demo 导出的 CSV 画成曲线图（总览 + 分张）。
#
# 运行方式：
#   ros2 run kdl_tools plot_cartesian                        # 读 ./cartesian_trajectory.csv
#   ros2 run kdl_tools plot_cartesian path/to/traj.csv       # 指定 CSV
#   ros2 run kdl_tools plot_cartesian traj.csv --out-dir /tmp/plots
#   ros2 run kdl_tools plot_cartesian traj.csv --show        # 额外弹出交互窗口
#
# 输出（默认写到 CSV 所在目录，<名字> 取自 CSV 文件名）：
#   <名字>_overview.png          2x3 总览：位置 / 姿态 / 线速度 / 线加速度 /
#                                角速度 / 角加速度 + 峰值文字块
#   <名字>_position.png          每个量再单独存一张，方便放大看细节
#   <名字>_orientation.png  ...
#
# 与 plot_quintic.py 的关系：
#   同一个包里的两个画图脚本，结构刻意保持一致（读 CSV → 总览 → 分张）。
#   区别只在数据本身：关节空间画的是 q/qdot/qddot，笛卡尔空间画的是位姿的
#   两半——位置 x/y/z（m）与姿态 RPY（rad）——以及它们对时间的一、二阶导。
#
# 姿态为什么画 RPY 而不是四元数：
#   RPY 三个角能直接看出"姿态在怎么变"，四元数四个分量画出来不直观。
#   CSV 里同时导出了四元数（qx/qy/qz/qw），需要精确复核时用它，不必改脚本。
#   代价是 RPY 在 ±π 附近会跳变——若图上出现竖直跳线，先看是不是这个原因。
#
# 为什么图里的文字用英文：
#   matplotlib 自带的 DejaVu Sans 没有中文字形，中文标签会显示成方框。
#   要在图里写中文，需要另外指定系统中文字体——那属于环境配置，不该由这个
#   教学脚本承担，所以图内一律英文，终端提示保持中文。

import argparse
import os
import sys

import numpy as np
import matplotlib

# 默认切到无窗口后端：这样在没有显示器的环境（ssh、CI、容器）里也能直接跑出 PNG。
# 只有显式传了 --show，才让 matplotlib 自己挑一个能弹窗的后端。
if "--show" not in sys.argv:
    matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402  （必须在选好后端之后再导入 pyplot）

# ---------------------------------------------------------------------------
# 要画的六个量。每项是 (文件后缀, 图标题, 纵轴标签, [(CSV 列名, 图例名), ...], 模长曲线名)
#
# 纵轴标签写成 dp/dt 这种形式而不是 ṗ：一是避免依赖字体里的组合点字符，
# 二是把"这是对时间求导"这件事写明白——角速度与角加速度最容易被看混。
#
# 模长曲线名给 None 就只画分量。速度/加速度之所以要额外叠一条 |v|、|a|，
# 是因为模块里的限位校验判的就是这几个模长，图上直接看到峰值才能对上账。
#
# 脚本只画 CSV 里**真实存在**的列：缺列就整格跳过，而不是直接报错——
# 画图工具没资格要求数据必须长成什么样。
# ---------------------------------------------------------------------------
PANELS = [
    ("position", "TCP position", "p [m]",
     [("x", "x"), ("y", "y"), ("z", "z")], None),
    ("orientation", "TCP orientation (RPY)", "angle [rad]",
     [("roll", "roll"), ("pitch", "pitch"), ("yaw", "yaw")], None),
    ("linear_velocity", "TCP linear velocity", "dp/dt [m/s]",
     [("vx", "vx"), ("vy", "vy"), ("vz", "vz")], "|v|"),
    ("linear_acceleration", "TCP linear acceleration", "d2p/dt2 [m/s^2]",
     [("ax", "ax"), ("ay", "ay"), ("az", "az")], "|a|"),
    ("angular_velocity", "TCP angular velocity", "omega [rad/s]",
     [("wx", "wx"), ("wy", "wy"), ("wz", "wz")], "|omega|"),
    ("angular_acceleration", "TCP angular acceleration", "alpha [rad/s^2]",
     [("alphax", "alphax"), ("alphay", "alphay"), ("alphaz", "alphaz")], "|alpha|"),
]


def parse_args(argv=None):
    """解析命令行参数。"""
    parser = argparse.ArgumentParser(
        description="把 cartesian_demo 导出的 CSV 画成曲线图（总览 + 分张 PNG）。",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "csv",
        nargs="?",
        default="cartesian_trajectory.csv",
        help="cartesian_demo 导出的 CSV 路径",
    )
    parser.add_argument(
        "--out-dir",
        default=None,
        help="图片输出目录（缺省为 CSV 所在目录）",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="除保存 PNG 外再弹出交互窗口（需要图形环境）",
    )
    return parser.parse_args(argv)


def load_csv(path):
    """读入 CSV，返回结构化数组（表头即字段名）。"""
    data = np.genfromtxt(path, delimiter=",", names=True)
    # 只有一行数据时 genfromtxt 会返回 0 维数组，统一拉平，后面就不必到处特判。
    return np.atleast_1d(data)


def available_panels(data):
    """筛出 CSV 里真正有数据的那些量，返回 PANELS 的子集。

    :param data: ``np.genfromtxt(..., names=True)`` 的结果
    :return: PANELS 的子集，且每项的列都保证存在
    """
    names = set(data.dtype.names or ())
    return [
        panel
        for panel in PANELS
        if panel[3] and all(column in names for column, _ in panel[3])
    ]


def panel_columns(data, panel):
    """取出该量在 CSV 里的列名列表（顺序与图例一致）。"""
    return [column for column, _ in panel[3]]


def build_info_text(data, csv_path, panels):
    """拼出总览图最后一格里的文字信息：采样规模 + 各量的峰值。"""
    t = np.asarray(data["t"], dtype=float)
    dt = float(np.mean(np.diff(t))) if t.size > 1 else 0.0

    lines = [
        "file    : {}".format(os.path.basename(csv_path)),
        "samples : {}".format(t.size),
        "span    : {:.4f} s".format(float(t[-1] - t[0])),
        "dt      : {:.4f} s".format(dt),
        "",
    ]
    for key, _, _, columns, magnitude_label in panels:
        values = np.column_stack([data[column] for column, _ in columns])
        # 位置、姿态没有"模长"可言，按分量取最大绝对值；
        # 速度/加速度则按矢量模长报——那才和模块里的限位校验同一口径。
        if magnitude_label:
            peak = float(np.max(np.linalg.norm(values, axis=1)))
        else:
            peak = float(np.max(np.abs(values)))
        lines.append("peak {:<24} = {:>12.5g}".format(key, peak))
    return "\n".join(lines)


def draw_panel(ax, data, panel):
    """在给定坐标轴上画出该量的所有分量（必要时再叠一条模长曲线）。"""
    _, title, ylabel, columns, magnitude_label = panel
    for (column, label) in columns:
        ax.plot(data["t"], data[column], label=label)

    if magnitude_label:
        values = np.column_stack([data[column] for column, _ in columns])
        ax.plot(data["t"], np.linalg.norm(values, axis=1), label=magnitude_label, alpha=0.6)

    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)


def build_overview(data, panels, csv_path):
    """构造 2x3 总览图：前几格画曲线，最后一格放文字信息。"""
    fig, axes = plt.subplots(2, 3, figsize=(16, 8), sharex=True)
    flat = axes.ravel()

    for index, panel in enumerate(panels):
        draw_panel(flat[index], data, panel)
        if index == 0:
            # 图例只在第一格给一次：同一批列名在全部子图里的颜色映射是一致的。
            flat[index].legend(fontsize=8, ncol=2)

    # x 轴共享，所以只在最下面一行标注时间轴。
    for ax in flat[3:6]:
        ax.set_xlabel("t [s]")

    # 剩下没用到的格子：第一格写文字信息，其余直接隐藏。
    for ax in flat[len(panels) + 1:]:
        ax.axis("off")
    if len(panels) < flat.size:
        info_ax = flat[len(panels)]
        info_ax.axis("off")
        info_ax.text(
            0.0, 1.0, build_info_text(data, csv_path, panels), va="top", ha="left",
            family="monospace", fontsize=10,
        )

    fig.suptitle("Cartesian pose interpolation — {}".format(os.path.basename(csv_path)))
    fig.tight_layout()
    return fig


def build_single(data, panel):
    """构造单个量的图，方便放大看细节。"""
    fig, ax = plt.subplots(figsize=(9, 4.5))
    draw_panel(ax, data, panel)
    ax.set_xlabel("t [s]")
    ax.legend(fontsize=8, ncol=2)
    fig.tight_layout()
    return fig


def main(argv=None):
    """读 CSV、画图、保存，并按需弹窗。"""
    args = parse_args(argv)

    if not os.path.isfile(args.csv):
        print("找不到 CSV 文件: {}".format(args.csv))
        print("提示: 先运行 `ros2 run kdl_tools cartesian_demo` 生成它"
              "（CSV 会落在运行时的工作目录里）。")
        return 1

    data = load_csv(args.csv)
    if "t" not in (data.dtype.names or ()):
        print("CSV 里没有 t 列，不像本包导出的轨迹文件: {}".format(args.csv))
        return 1

    panels = available_panels(data)
    if not panels:
        print("CSV 里没有任何可画的列（期望 x/y/z 或 vx/vy/vz 这种命名）: {}".format(args.csv))
        return 1

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv))
    os.makedirs(out_dir, exist_ok=True)

    # 输出文件名前缀取自 CSV 的文件名，这样同一个目录里放多条轨迹也不会互相覆盖。
    stem = os.path.splitext(os.path.basename(args.csv))[0]

    written = []
    figures = []

    overview = build_overview(data, panels, args.csv)
    overview_path = os.path.join(out_dir, "{}_overview.png".format(stem))
    overview.savefig(overview_path, bbox_inches="tight")
    figures.append(overview)
    written.append(overview_path)

    for panel in panels:
        figure = build_single(data, panel)
        path = os.path.join(out_dir, "{}_{}.png".format(stem, panel[0]))
        figure.savefig(path, bbox_inches="tight")
        figures.append(figure)
        written.append(path)

    print("已读取 {} 个采样点、{} 个量:".format(len(data["t"]), len(panels)))
    for path in written:
        print("  {}".format(os.path.abspath(path)))

    if args.show:
        plt.show()

    for figure in figures:
        plt.close(figure)
    return 0


if __name__ == "__main__":
    sys.exit(main())
