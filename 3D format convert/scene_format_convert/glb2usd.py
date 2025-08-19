import bpy
import sys
import os


def convert_glb_to_usd(input_glb: str, output_usd: str):
    """
    Convert a single GLB file to USD format.

    Args:
        input_glb (str): Path to the input GLB file.
        output_usd (str): Path to save the converted USD file.
    """
    # Reset Blender scene to a clean state
    bpy.ops.wm.read_factory_settings(use_empty=True)

    # Import GLB file
    bpy.ops.import_scene.gltf(filepath=input_glb)

    # Export to USD
    bpy.ops.wm.usd_export(filepath=output_usd, export_materials=True)


def process_file(input_glb: str, output_dir: str):
    """
    Process a single GLB file and export it as USD in the given output directory.

    Args:
        input_glb (str): Path to the input GLB file.
        output_dir (str): Directory where the USD file will be saved.
    """
    filename = os.path.splitext(os.path.basename(input_glb))[0] + ".usd"
    output_usd = os.path.join(output_dir, filename)

    print(f"Transforming: {input_glb} -> {output_usd}")
    convert_glb_to_usd(input_glb, output_usd)
    print(f"Conversion complete: {output_usd}")


def process_directory(input_dir: str, output_dir: str):
    """
    Process all GLB files in a directory and convert them to USD.

    Args:
        input_dir (str): Path to the directory containing GLB files.
        output_dir (str): Directory where USD files will be saved.
    """
    os.makedirs(output_dir, exist_ok=True)

    # List all .glb files in the directory
    glb_files = [f for f in os.listdir(input_dir) if f.lower().endswith(".glb")]

    if not glb_files:
        print(f"No .glb files found in: {input_dir}")
        return

    for glb in glb_files:
        process_file(os.path.join(input_dir, glb), output_dir)


def main():
    """
    Entry point of the script.
    Usage:
        blender -b -P glb2usd.py -- <input file/folder> [output folder]

    Arguments:
        input file/folder: Path to a GLB file or a directory containing GLB files.
        output folder (optional): Directory where USD files will be saved.
    """
    if "--" in sys.argv:
        try:
            input_path = sys.argv[sys.argv.index("--") + 1]
        except IndexError:
            print("Error: Missing input file/folder after '--'")
            return
        output_dir = sys.argv[sys.argv.index("--") + 2] if len(sys.argv) > sys.argv.index("--") + 2 else None
    else:
        if len(sys.argv) < 2:
            print("Usage: blender -b -P glb2usd.py -- <input file/folder> [output folder]")
            return
        input_path = sys.argv[1]
        output_dir = sys.argv[2] if len(sys.argv) > 2 else None

    if not os.path.exists(input_path):
        print(f"Error: Input path does not exist: {input_path}")
        return

    if os.path.isdir(input_path):
        output_dir = output_dir or "output"
        process_directory(input_path, output_dir)

    elif os.path.isfile(input_path) and input_path.lower().endswith(".glb"):
        output_dir = output_dir or os.getcwd()
        os.makedirs(output_dir, exist_ok=True)
        process_file(input_path, output_dir)

    else:
        print(f"Error: Invalid input path, not a GLB file or directory: {input_path}")


if __name__ == "__main__":
    main()
