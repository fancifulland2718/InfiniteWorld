# -*-coding:utf-8 -*-
import os
import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse


def get_args():
    parser = argparse.ArgumentParser(
        description="Find and visualize planes in a point cloud"
    )
    parser.add_argument("--input", type=str, help="Input PLY file", default="./playroom_3dgs.ply")
    # parser.add_argument("--input", type=str, help="Input PLY file", required=True)
    return parser.parse_args()


def plot_color_legend(colors, equations):
    """
    Plot a color legend with corresponding plane equations.

    Args:
    colors (list): List of RGB triples.
    equations (list): List of strings containing plane equations.
    """
    # Create figure and axis
    fig, ax = plt.subplots(figsize=(6, 6))  # Adjusted figure size for vertical layout
    ax.axis("off")
    num_colors = len(colors)

    # Create a rectangle patch for each color and place it vertically
    for i, (color, eq) in enumerate(zip(colors, equations)):
        rect = patches.Rectangle(
            (0, i / num_colors),
            1,
            1 / num_colors,
            linewidth=1,
            edgecolor="none",
            facecolor=color,
        )
        ax.add_patch(rect)
        # Place text to the right of each color patch
        ax.text(
            1.05,
            (i / num_colors) + 0.5 / num_colors,
            eq,
            ha="left",
            va="center",
            fontsize=10,
            color="black",
        )

    ax.set_ylim(0, 1)  # Ensure the y-axis does not extend beyond the patches
    ax.set_aspect(num_colors)  # Adjust aspect ratio to display rectangles
    plt.show()


def main(args):

    # test_data_dir = "./drjohnson_pgsr"
    # point_cloud_file_name = "tsdf_fusion.ply"
    # point_cloud_file_path = os.path.join(test_data_dir, point_cloud_file_name)
    point_cloud_file_path = args.input

    # 检查文件是否存在
    if not os.path.exists(point_cloud_file_path):
        print("点云文件不存在，请检查路径。")
        exit()

    # 读取点云
    pcd = o3d.io.read_point_cloud(point_cloud_file_path)

    # 初始化剩余点云
    remaining_cloud = pcd
    geometries = []
    alpha = 0.5  # 透明度设置
    num_planes = 5  # 需要查找的平面数量
    planes = num_planes * [None]  # 初始化平面列表

    # 使用matplotlib的colormap
    colormap = plt.get_cmap("tab20")  # 使用tab20颜色映射
    colors = [
        colormap(i / num_planes) for i in range(num_planes)
    ]  # 根据平面数量分割颜色
    plane_equations = []

    # 迭代查找平面
    for i in range(num_planes):
        # 平面分割
        plane_model, inliers = remaining_cloud.segment_plane(
            distance_threshold=0.01, ransac_n=3, num_iterations=3000
        )
        if len(inliers) < 1:
            print("找不到更多平面。")
            break

        # 提取内点和外点
        inlier_cloud = remaining_cloud.select_by_index(inliers)
        original_colors = np.asarray(inlier_cloud.colors)  # 获取原始颜色

        # 使用colormap设置颜色
        color = colors[i]
        # 应用透明度
        # mixed_color = (np.array(color[:3]) * (1 - alpha) + np.array([1, 1, 1]) * alpha).tolist()
        # inlier_cloud.paint_uniform_color(mixed_color)
        mixed_color = original_colors * (1 - alpha) + np.array(color[:3]) * alpha
        inlier_cloud.colors = o3d.utility.Vector3dVector(mixed_color)
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)

        geometries.append(inlier_cloud)
        planes[i] = plane_model

        # 打印平面方程
        [a, b, c, d] = plane_model
        plane_equation = f"{a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0"
        planes[i] = plane_equation
        plane_equations.append(plane_equation)

        print(f"Plane {i + 1}: {plane_equation}")

    plot_color_legend([color[:3] for color in colors], plane_equations)

    # 可视化所有平面
    geometries.append(remaining_cloud)  # 将剩余点云添加到可视化中
    o3d.visualization.draw_geometries(
        geometries,
        zoom=0.8,
        front=[-0.4999, -0.1659, -0.8499],
        lookat=[2.1813, 2.0619, 2.0999],
        up=[0.1204, -0.9852, 0.1215],
    )


if __name__ == "__main__":
    main(get_args())
