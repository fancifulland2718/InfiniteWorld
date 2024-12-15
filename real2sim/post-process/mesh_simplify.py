import pymeshlab
import argparse


def get_args():
    parser = argparse.ArgumentParser(description="downsampling mesh with texture")
    parser.add_argument("--input", type=str, help="Input OBJ file", required=True)

    return parser.parse_args()


def main(args):
    input_obj = args.input
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(input_obj)
    assert ms.mesh_number() == 1
    num_mesh = ms[0].vertex_number()
    print("Number of vertex: ", ms[0].vertex_number())
    ds_rate = 4e5 / num_mesh if num_mesh > 4e5 else 0.95
    ms.meshing_decimation_quadric_edge_collapse_with_texture(
        targetperc=ds_rate,
        qualitythr=0.500000,
        preservenormal=True,
        planarquadric=True,
    )
    ms.save_current_mesh(input_obj.replace(".obj", "_downsampled.obj"))

if __name__ == "__main__":
    main(get_args())