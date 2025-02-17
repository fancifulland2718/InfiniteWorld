input_file=$1
base_name="${input_file%.ply}"
python3 plane_detect.py --input ${input_file}

echo "please choose a plane and input x"
read x
echo "please choose a plane and input y"
read y
echo "please choose a plane and input z"
read z
echo "please choose a plane and input d"
read d

python3 align_ground_plane.py --input ${input_file} --plane_arg ${x} ${y} ${z} ${d}
rotated_base_name="${base_name}_pre_o3d"
python3 mesh_clear.py --input "${rotated_base_name}.ply"
python3 fill_holes.py --input "${rotated_base_name}_denoised.ply"
# python3 remove_roof.py --input "${base_name}_denoised_r.ply"
python3 restore_colors.py --input "${rotated_base_name}_denoised_r.ply" --original ${input_file}
python3 ply2obj_bpy.py --input "${rotated_base_name}_denoised_r_colored.ply" --output "${base_name}.obj"
python3 mesh_simplify.py --input "${base_name}.obj"