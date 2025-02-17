import bpy
import argparse
import sys
import os
import argparse


def get_args():
    parser = argparse.ArgumentParser(description="Convert PLY to OBJ using PyBlender")
    parser.add_argument("--input", type=str, help="Input PLY file", required=True)
    parser.add_argument("--output", type=str, help="Output OBJ file", required=True)
    parser.add_argument(
        "--width", type=int, help="Texture width, the same as height", default=4096
    )
    return parser.parse_args()


# for i in range(1):
def main(args):
    print(args)
    input_ply = args.input
    output_obj = args.output
    # input_ply = "./playroom_pgsr/playroom_pgsr.ply"
    # output_obj = "./playroom_pgsr/playroom_pgsr_bpy.obj"

    name, _ = os.path.splitext(output_obj)
    output_png = "{}.png".format(name)
    output_mtl = "{}.mtl".format(name)

    print("Input PLY: {}".format(input_ply))
    print("Output OBJ: {}".format(output_obj))
    print("Output PNG: {}".format(output_png))
    print("Output MTL: {}".format(output_mtl))

    bpy.ops.object.select_all(action="DESELECT")
    bpy.ops.object.select_by_type(type="MESH")
    bpy.ops.object.delete(use_global=False)

    # # https://docs.blender.org/api/current/bpy.data.html
    # print('Remove default cube mesh')
    # if "Cube" in bpy.data.meshes:
    #     mesh = bpy.data.meshes["Cube"]
    #     bpy.data.meshes.remove(mesh)

    print("Import PLY")
    bpy.ops.import_mesh.ply(filepath=input_ply)

    print("Toggle edit mode")
    bpy.ops.object.editmode_toggle()

    print("UV smart project")
    bpy.ops.uv.smart_project()

    # https://blender.stackexchange.com/questions/5668/add-nodes-to-material-with-python
    print("Add shading material")
    material = bpy.data.materials.new("SomeMaterial")
    material.use_nodes = True
    nodes = material.node_tree.nodes

    print("Toggle edit mode")
    bpy.ops.object.editmode_toggle()

    print("Add input vertex color")
    input_node = nodes.new("ShaderNodeVertexColor")
    bsdf_node = nodes.get("Principled BSDF")

    print("Link vertex color to bsdf")
    material.node_tree.links.new(bsdf_node.inputs[0], input_node.outputs[0])

    print("Add texture image")
    texture_node = nodes.new("ShaderNodeTexImage")

    print("Create empty image")
    image = bpy.data.images.new(name="SomeImage", width=args.width, height=args.width)

    print("Assign image to node")
    texture_node.image = image

    print("Switch to CYCLES render engine")
    bpy.context.scene.render.engine = "CYCLES"

    print("Select active material")
    bpy.context.active_object.active_material = material

    print("Bake image")
    bpy.context.view_layer.objects.active = bpy.context.active_object
    bpy.ops.object.bake(type="DIFFUSE", pass_filter={"COLOR"}, use_clear=True)

    print("Save image")
    image.save_render(output_png)

    # set map_Kd correctly in mtl file
    print("Set image path")
    image.filepath = os.path.basename(output_png)

    print("Connect texture node to bsdf")
    material.node_tree.links.new(bsdf_node.inputs[0], texture_node.outputs[0])

    print("Export OBJ")
    bpy.ops.export_scene.obj(filepath=output_obj)


if __name__ == "__main__":
    main(get_args())
