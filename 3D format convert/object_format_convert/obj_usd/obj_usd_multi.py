# blender_python_script.py
import bpy
import os
import argparse
from tqdm import tqdm

def convert_obj_to_usd(dataset_dir, output_dir):
    # 获取所有子目录
    subdirs = [os.path.join(dataset_dir, o) for o in os.listdir(dataset_dir) if os.path.isdir(os.path.join(dataset_dir, o))]
    
    # 使用 tqdm 显示进度条
    for subdir in tqdm(subdirs, desc="Processing directories"):
        relative_path = os.path.relpath(subdir, dataset_dir)
        output_subdir = os.path.join(output_dir, relative_path)

        # 检查输出子目录是否存在
        if os.path.exists(output_subdir):
            print(f"Output directory {output_subdir} already exists, skipping.")
            continue

        os.makedirs(output_subdir, exist_ok=True)

        obj_import_path = os.path.join(subdir, "raw_model.obj")
        usd_export_path = os.path.join(output_subdir, "output.usd")

        if os.path.exists(obj_import_path):
            # 删除当前场景中所有对象
            bpy.ops.object.select_all(action='SELECT')
            bpy.ops.object.delete()

            # 导入OBJ文件
            bpy.ops.wm.obj_import(filepath=obj_import_path)

            # 导出为USD文件
            bpy.ops.wm.usd_export(filepath=usd_export_path)

            print(f"OBJ file has been imported from {obj_import_path} and exported to {usd_export_path}")
        else:
            print(f"No OBJ file found in {subdir}, skipping.")

def parse_args():
    parser = argparse.ArgumentParser(description="Convert OBJ files to USD files and save them in a specified output directory.")
    parser.add_argument('-data_dir', type=str, required=True, help='The directory containing the dataset with OBJ files.')
    parser.add_argument('-output_dir', type=str, required=True, help='The directory to save the converted USD files.')
    args = parser.parse_args()
    return args

if __name__ == "__main__":
    args = parse_args()
    convert_obj_to_usd(args.data_dir, args.output_dir)

