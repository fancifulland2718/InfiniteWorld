import open3d as o3d
import numpy as np
import argparse

def get_args():
    parser = argparse.ArgumentParser(
        description="Remove the noise from a point cloud mesh"
    )
    parser.add_argument("--input", type=str, help="Input PLY file", required=True)
    parser.add_argument("--k", type=int, help="Number of largest components to retain", default=2)
    return parser.parse_args()

def keep_largest_k_connected_components(file_path, k):
    # Load the mesh
    mesh = o3d.io.read_triangle_mesh(file_path)
    if mesh.is_empty():
        raise ValueError("No valid mesh data found.")

    # Check if mesh is watertight
    print("Is the mesh watertight (manifold)?", mesh.is_watertight())

    # Find connected components
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        triangle_clusters, cluster_n_triangles, cluster_area = (
            mesh.cluster_connected_triangles()
        )

    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)

    # Sort clusters by the number of triangles and get the indices of the top k largest components
    largest_components_indices = np.argsort(cluster_n_triangles)[-k:]
    
    # Create a mask to retain only the triangles of the top k largest components
    triangles_to_keep = np.isin(triangle_clusters, largest_components_indices)

    # Remove triangles that are not part of the largest k components
    mesh.remove_triangles_by_mask(~triangles_to_keep)

    # Save the new mesh
    new_file_path = file_path.replace(".ply", f"_{k}_denoised.ply")
    o3d.io.write_triangle_mesh(new_file_path, mesh, write_ascii=False)
    print(f"Top {k} connected components mesh saved to {new_file_path}")

    return new_file_path

def main(args):
    new_mesh_path = keep_largest_k_connected_components(args.input, args.k)

if __name__ == "__main__":
    main(get_args())
