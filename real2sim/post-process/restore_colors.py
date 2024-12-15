import numpy as np
from scipy.spatial import cKDTree
from plyfile import PlyData, PlyElement
import argparse

def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--original", type=str, default="playroom_pgsr\playroom_pgsr_pre_o3d_largest_component.ply")
    parser.add_argument("--input", type=str, default="playroom_pgsr\playroom_pgsr_pre_o3d_largest_component_fixed.ply")
    # parser.add_argument("--output", type=str, default="playroom_pgsr\playroom_pgsr_pre_o3d_largest_component_fixed_colored.ply")
    return parser.parse_args()

def read_ply(filename):
    """读取PLY文件并返回顶点坐标、颜色数据和面数据"""
    plydata = PlyData.read(filename)
    vertex = plydata['vertex']
    points = np.array([vertex['x'], vertex['y'], vertex['z']]).transpose()
    try:
        colors = np.array([vertex['red'], vertex['green'], vertex['blue']]).transpose()
    except:
        colors = None
    try:
        faces = np.vstack(plydata['face'].data['vertex_indices'])
    except:
        faces = None
    return points, colors, faces

def write_ply(filename, points, colors, faces):
    """将点云、颜色和面数据以二进制格式写入PLY文件"""
    vertices = np.array(list(zip(points[:, 0], points[:, 1], points[:, 2],
                                 colors[:, 0], colors[:, 1], colors[:, 2])),
                        dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4'),
                               ('red', 'u1'), ('green', 'u1'), ('blue', 'u1')])
    vertex_el = PlyElement.describe(vertices, 'vertex')

    if faces is not None:
        # 处理面数据以确保格式正确
        face_data = np.array([(face,) for face in faces], dtype=[('vertex_indices', 'i4', (3,))])
        face_el = PlyElement.describe(face_data, 'face')
        PlyData([vertex_el, face_el], text=False).write(filename)  # 设置text=False以保存为二进制
    else:
        PlyData([vertex_el], text=False).write(filename)  # 设置text=False以保存为二进制


def colorize_fixed_mesh(original_ply, fixed_ply):
    """使用原始颜色映射颜色到修复后的网格并保持面信息"""
    original_points, original_colors, _ = read_ply(original_ply)
    fixed_points, _, fixed_faces = read_ply(fixed_ply)

    if original_colors is None:
        raise ValueError("Original PLY file does not contain color information.")

    # 使用KD树查找最近邻顶点以映射颜色
    tree = cKDTree(original_points)
    _, indices = tree.query(fixed_points)
    mapped_colors = original_colors[indices]

    output_ply = fixed_ply.replace(".ply", "_colored.ply")

    print("Writing colored fixed mesh to", output_ply)
    # 写入带颜色的修复后的PLY文件，保留面数据
    write_ply(output_ply, fixed_points, mapped_colors, fixed_faces)


def main(args):
    # 使用实例
    # original_ply_path = "playroom_pgsr\playroom_pgsr_pre_o3d_largest_component.ply"
    # fixed_ply_path = "playroom_pgsr\playroom_pgsr_pre_o3d_largest_component_fixed.ply"
    # output_ply_path = (
    #     "playroom_pgsr\playroom_pgsr_pre_o3d_largest_component_fixed_colored.ply"
    # )

    colorize_fixed_mesh(args.original, args.input)

if __name__ == "__main__":
    main(get_args())