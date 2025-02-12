import pymeshlab
import os
import argparse

def convert_ply_to_obj(input_path, output_path):
    """Convert a PLY file to an OBJ file using PyMeshLab.

    Args:
        input_path (str): Path to the input PLY file.
        output_path (str): Path to save the output OBJ file.
    """
    ms = pymeshlab.MeshSet()

    try:
        ms.load_new_mesh(input_path)
        print("PLY file loaded successfully.")
    except Exception as e:
        print(f"Error loading PLY file: {e}")
        return

    try:
        ms.save_current_mesh(output_path)
        print("OBJ file saved successfully at", output_path)
    except Exception as e:
        print(f"Error saving OBJ file: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a PLY file to an OBJ file.")
    parser.add_argument("--input", type=str, help="Path to the input PLY file.")
    parser.add_argument("--output", type=str, help="Path to save the output OBJ file.")

    args = parser.parse_args()

    if not os.path.exists(args.input):
        print(f"Input file does not exist: {args.input}")
    else:
        convert_ply_to_obj(args.input, args.output)