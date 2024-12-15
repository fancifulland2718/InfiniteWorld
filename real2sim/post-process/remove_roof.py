import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
import argparse


def get_args():
    parser = argparse.ArgumentParser(
        description="Remove the roof from a point cloud mesh"
    )
    parser.add_argument("--input", type=str, help="Input PLY file", required=True)
    parser.add_argument(
        "--num_bins", type=int, help="Number of bins for histogram", default=40
    )
    parser.add_argument(
        "--threshold", type=float, help="Roof threshold as a percentile", default=None
    )
    return parser.parse_args()


def remove_roof_from_point_cloud(
    file_path, num_bins=64, threshold=None
):  # Increased num_bins for better resolution
    # Load the mesh
    mesh = o3d.io.read_triangle_mesh(file_path)
    if mesh.is_empty():
        print("No valid mesh data found.")
        return

    # Extract the point cloud
    points = np.asarray(mesh.vertices)

    # Compute the histogram of z-values
    hist, bin_edges = np.histogram(points[:, 2], bins=num_bins)

    # Find local minima and maxima
    minima = (np.diff(np.sign(np.diff(hist))) > 0).nonzero()[0] + 1
    maxima = (np.diff(np.sign(np.diff(hist))) < 0).nonzero()[0] + 1

    if not maxima.size or not minima.size:
        assert False, "No sufficient local extrema found."

    # Ensure there's a minimum before the last maximum
    last_max_index = maxima[-1]
    minima_before_last_max = minima[minima < last_max_index]
    if not minima_before_last_max.size:
        print("No local minima found before the last maximum.")
        return

    # Select the last local minimum before the last maximum as the roof threshold
    roof_threshold = bin_edges[minima_before_last_max[-1]]
    print(f"Roof threshold: {roof_threshold} from local minimum")
    if threshold is not None:
        roof_threshold = np.percentile(points[:, 2], threshold)
        print(f"Roof threshold: {roof_threshold} from percentile")
    roof_threshold = 0.7  # Hardcoded threshold
    # Determine the indices to keep
    valid_indices = np.where(points[:, 2] < roof_threshold)[0]
    index_map = {
        old_index: new_index for new_index, old_index in enumerate(valid_indices)
    }

    # Update points and optionally colors
    new_points = points[valid_indices]
    if mesh.has_vertex_colors():
        new_colors = np.asarray(mesh.vertex_colors)[valid_indices]

    # Update triangles, removing those referencing deleted vertices
    new_triangles = []
    for triangle in tqdm(np.asarray(mesh.triangles)):
        if all(vertex in index_map for vertex in triangle):
            new_triangles.append([index_map[vertex] for vertex in triangle])

    # Rebuild the mesh
    new_mesh = o3d.geometry.TriangleMesh()
    new_mesh.vertices = o3d.utility.Vector3dVector(new_points)
    new_mesh.triangles = o3d.utility.Vector3iVector(new_triangles)
    if mesh.has_vertex_colors():
        new_mesh.vertex_colors = o3d.utility.Vector3dVector(new_colors)

    # Save the modified mesh
    new_file_path = file_path.replace(".ply", "_no_roof.ply")
    o3d.io.write_triangle_mesh(
        new_file_path, new_mesh, write_ascii=False, compressed=False
    )
    print(f"Mesh without roof saved to {new_file_path}")

    # Visualize the histogram
    plt.bar(
        bin_edges[:-1], hist, width=np.diff(bin_edges), edgecolor="black", align="edge"
    )
    plt.axvline(x=roof_threshold, color="r", linestyle="dashed", linewidth=1)
    plt.title("Histogram of Z Values")
    plt.show()


def main(args):
    # Example usage
    # file_path = "./drjohnson_pgsr/tsdf_fusion_pre_o3d.ply"
    file_path = args.input
    remove_roof_from_point_cloud(
        file_path, num_bins=args.num_bins, threshold=args.threshold
    )


if __name__ == "__main__":
    main(get_args())
