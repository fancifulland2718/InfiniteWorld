import bpy
import numpy as np
import os
from tqdm import tqdm
import argparse

def load_pickled_3d_asset(file_path, export_path):
    import gzip
    import pickle

    # Open the compressed pickled file
    with gzip.open(file_path, 'rb') as f:
        # Load the pickled object
        loaded_object_data = pickle.load(f)

    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

    # Create a new mesh object in Blender
    mesh = bpy.data.meshes.new(name='LoadedMesh')
    obj = bpy.data.objects.new('LoadedObject', mesh)

    # Link the object to the scene
    bpy.context.scene.collection.objects.link(obj)

    # Set the mesh data for the object
    obj.data = mesh

    # Update the mesh with the loaded data
    triangles = np.array(loaded_object_data['triangles']).reshape(-1, 3)
    vertices = [[v['x'], v['z'], v['y']] for v in loaded_object_data['vertices']]

    mesh.from_pydata(vertices, [], triangles)

    uvs = [[uv['x'], uv['y']] for uv in loaded_object_data['uvs']]

    mesh.update()

    # Ensure UV coordinates are stored
    if not mesh.uv_layers:
        mesh.uv_layers.new(name="UVMap")

    uv_layer = mesh.uv_layers["UVMap"]
    for poly in mesh.polygons:
        for loop_index in poly.loop_indices:
            vertex_index = mesh.loops[loop_index].vertex_index
            uv_layer.data[loop_index].uv = uvs[vertex_index]

    material = bpy.data.materials.new(name="AlbedoMaterial")
    obj.data.materials.append(material)

    # Assign albedo color to the material
    material.use_nodes = True
    nodes = material.node_tree.nodes
    principled_bsdf = nodes.get("Principled BSDF")

    texture_node = nodes.new(type='ShaderNodeTexImage')
    albedo_path = os.path.join(os.path.dirname(file_path), 'albedo.jpg')
    image = bpy.data.images.load(albedo_path)
    texture_node.image = image

    # Connect the texture node to the albedo color
    material.node_tree.links.new(
        texture_node.outputs["Color"],
        principled_bsdf.inputs["Base Color"]
    )

    # Add normal map
    normal_path = os.path.join(os.path.dirname(file_path), 'normal.jpg')
    img_normal = bpy.data.images.load(normal_path)
    image_texture_node_normal = material.node_tree.nodes.new(type='ShaderNodeTexImage')
    image_texture_node_normal.image = img_normal
    image_texture_node_normal.image.colorspace_settings.name = 'Non-Color'

    normal_map_node = material.node_tree.nodes.new(type='ShaderNodeNormalMap')

    material.node_tree.links.new(normal_map_node.outputs["Normal"], principled_bsdf.inputs["Normal"])
    material.node_tree.links.new(image_texture_node_normal.outputs["Color"], normal_map_node.inputs["Color"])

    # Assign the material to the object
    obj.data.materials[0] = material

    # Update mesh to apply UV changes
    mesh.update()

    # Export the object
    bpy.ops.wm.obj_export(filepath=export_path)

    return obj

def process_3d_assets(input_path):
    obj_names = os.listdir(input_path)
    for obj_name in tqdm(obj_names, desc="Processing directories"):
        full_path = os.path.join(input_path, obj_name, f"{obj_name}.pkl.gz").replace('\\', '/')
        full_export = os.path.join(input_path, obj_name, "raw_model.obj").replace('\\', '/')

        if not os.path.exists(full_path):
            print(f"File not found: {full_path}")
            continue

        try:
            load_pickled_3d_asset(full_path, full_export)
        except Exception as e:
            print(f"Failed to process {full_path}: {e}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Process 3D assets from pickled files.")
    parser.add_argument("input_directory", type=str, nargs='?', default=os.getcwd(),
                        help="Path to the input directory containing 3D asset files.")

    args = parser.parse_args()
    process_3d_assets(args.input_directory)
