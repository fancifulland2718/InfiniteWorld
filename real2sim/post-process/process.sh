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

python3 rot.py --input ${input_file} --plane_arg ${x} ${y} ${z} ${d}
python3 purge_noise.py --input "${base_name}_pre_o3d.ply"
rotated_base_name="${base_name}_pre_o3d"
python3 fill_hole_t.py --input "${rotated_base_name}_largest_component.ply"
# python3 remove_roof.py --input "${base_name}_largest_component_r.ply"
python3 color_after_fix.py --input "${rotated_base_name}_largest_component_r.ply" --original ${input_file}
python3 ply2obj_bpy.py --input "${rotated_base_name}_largest_component_r_colored.ply" --output "${base_name}.obj"
python3 simplification.py --input "${base_name}.obj"