import open3d as o3d
import numpy as np
import os
import argparse


def get_args():
    parser = argparse.ArgumentParser(
        description="transformation on mesh to make ground plane near `z=0`"
    )
    parser.add_argument("--input", type=str, help="Input PLY file", required=True)
    parser.add_argument(
        "--plane_arg",
        type=float,
        nargs=4,
        help="Plane equation arguments (a, b, c, d)",
        required=True,
    )
    return parser.parse_args()


def rotation_matrix_from_vectors(vec1, vec2):
    """Find the rotation matrix that aligns vec1 to vec2"""
    a, b = vec1 / np.linalg.norm(vec1), vec2 / np.linalg.norm(vec2)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s**2))
    return rotation_matrix


def main(args):
    # Path settings
    # test_data_dir = "./drjohnson_pgsr"
    # point_cloud_file_name = "tsdf_fusion.ply"
    # point_cloud_file_path = os.path.join(test_data_dir, point_cloud_file_name)
    point_cloud_file_path = args.input

    # Check if the file exists
    if not os.path.exists(point_cloud_file_path):
        assert False, "Point cloud file does not exist, please check the path."

    # Read the point cloud, making sure to load points and faces
    mesh = o3d.io.read_triangle_mesh(point_cloud_file_path)
    if mesh.is_empty():
        assert False, "The loaded file does not contain valid point cloud or mesh data."

    # Check if faces are loaded
    if mesh.triangles:
        print("Face data has been successfully loaded.")
    else:
        print("No face data found.")

    # Processing operations, such as rotation (include rotation logic here)
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices

    # # Use RANSAC to find the ground plane
    # plane_model, inliers = pcd.segment_plane(
    #     distance_threshold=0.12, ransac_n=4, num_iterations=2500
    # )
    [a, b, c, d] = args.plane_arg  # Plane equation: ax + by + cz + d = 0

    normal_vector = np.array([a, b, c])

    # Check the direction of the normal vector
    cnts = 2 * [0]
    for point in np.asarray(pcd.points):
        cnts[float(normal_vector @ point + d) > 0] += 1

    # If there are more points below the ground than above, reverse the normal vector
    if cnts[0] > cnts[1]:
        normal_vector *= -1  # Reverse the normal vector

    # Update the normal vector
    # normal_vector = np.array([a, b, c])

    # Target vector (z-axis)
    target_vector = np.array([0, 0, 1])

    # Compute the rotation matrix
    R = rotation_matrix_from_vectors(normal_vector, target_vector)

    # Create a 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R

    # Rotate the point cloud
    mesh.rotate(R, center=(0, 0, 0))

    # Ensure all points are above z > 0, then apply a small translation offset
    additional_offset = (
        -min(np.asarray(mesh.vertices)[:, 2]) + 0.01  # Add 0.01 to ensure z > 0
    )
    mesh.translate((0, 0, additional_offset))

    # Save the adjusted point cloud/mesh
    # output_file_path = os.path.join(test_data_dir, "tsdf_fusion_pre_o3d.ply")
    output_file_path = point_cloud_file_path.replace(".ply", "_pre_o3d.ply")
    o3d.io.write_triangle_mesh(
        output_file_path, mesh, write_ascii=False, compressed=False
    )

    print(f"Adjusted mesh saved to {output_file_path}")


if __name__ == "__main__":
    main(get_args())
