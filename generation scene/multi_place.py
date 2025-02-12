from omni.isaac.kit import SimulationApp
import math
from scipy.spatial.transform import Rotation as R
import json

# Initialize the Simulation App
simulation_app = SimulationApp()

import omni
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.objects import DynamicCuboid
from omni.kit.commands import execute
import numpy as np

def convert_ai2thor_to_isaacsim(position, rotation):
    """
    Convert AI2-THOR position and rotation to Isaac Sim position and rotation.
    
    Args:
        position : A tuple with 'x', 'y', 'z' representing AI2-THOR position.
        rotation : A tuple with 'x', 'y', 'z' representing AI2-THOR rotation (pitch, yaw, roll).
    
    Returns:
        tuple: 
            - position : A tuple with 'x', 'y', 'z' representing Isaac Sim position.
            - rotation : A tuple with 'x', 'y', 'z' representing Isaac Sim rotation (pitch, yaw, roll).
    """
    # Convert position
    # converted_position = (position[0],  position[2],  position[1])
    converted_position = (position[0],  position[2],  0)

    # Convert rotation
    # AI2-THOR uses (pitch, yaw, roll)
    # Isaac Sim also uses (pitch, yaw, roll), but we need to account for the different axes
    ai2thor_rotation = R.from_euler('xyz', [rotation[0], rotation[1], rotation[2]], degrees=True)
    
    # Convert the rotation to the Isaac Sim coordinate system
    # Rotate around the y-axis by 90 degrees to align with Isaac Sim's coordinate system
    align_rotation = R.from_euler('x', 90, degrees=True)
    
    # Apply the alignment rotation
    isaacsim_rotation = align_rotation * ai2thor_rotation
    
    # Convert back to euler angles
    converted_rotation = isaacsim_rotation.as_euler('xyz', degrees=True)
    
    # return converted_position, (math.radians(converted_rotation[0]), math.radians(converted_rotation[1]), math.radians(converted_rotation[2]))
    return converted_position, (converted_rotation[0], converted_rotation[1], converted_rotation[2])

def create_wall(stage, path, translation, rotation, scale):
    wall_prim = stage.DefinePrim(path, 'Cube')
    if wall_prim.IsValid():
        omni.kit.commands.execute('TransformPrimSRT', path=path,
                                new_translation=translation,
                                new_rotation_euler=rotation,
                                new_scale=scale)
    else:
        print(f"Invalid prim path: {path}")

def import_objv(usd_context, path, file_path, translation, rotation, scale):
    omni.kit.commands.execute('CreateReference', usd_context=usd_context, 
                              path_to=path, asset_path=file_path)
    omni.kit.commands.execute('TransformPrimSRT', path=path,
                              new_translation=translation,
                              new_rotation_euler=rotation,
                              new_scale=scale)

def calculate_height(points):
    height = max(points[:,2]) - min(points[:,2])
    return height

def import_door(usd_context, floor_path, door_pos, wall_polygon, exterior):
    wall_height = calculate_height(wall_polygon)
    door_height = door_pos[1]["y"]
    if wall_polygon[0][0] - wall_polygon[2][0] != 0:
        wall_min = min(wall_polygon[:,0])
        wall_max = max(wall_polygon[:,0])
        wall_y = wall_polygon[0][1]
        import_wall(usd_context, floor_path, [[wall_min,wall_y,0], [wall_min,wall_y,wall_height], [wall_min+door_pos[0]["x"],wall_y,wall_height], [wall_min+door_pos[0]["x"],wall_y,0]], exterior)
        import_wall(usd_context, floor_path, [[wall_min+door_pos[1]["x"],wall_y,wall_height], [wall_min+door_pos[1]["x"],wall_y,0], [wall_max,wall_y,0], [wall_max,wall_y,wall_height]], exterior)
        import_wall(usd_context, floor_path, [[wall_min+door_pos[0]["x"],wall_y,door_height], [wall_min+door_pos[0]["x"],wall_y,wall_height], [wall_min+door_pos[1]["x"],wall_y,wall_height], [wall_min+door_pos[1]["x"],wall_y,door_height]], exterior)
    
    elif wall_polygon[0][1] - wall_polygon[2][1] != 0:
        wall_min = min(wall_polygon[:,1])
        wall_max = max(wall_polygon[:,1])
        wall_x = wall_polygon[0][0]
        import_wall(usd_context, floor_path, [[wall_x,wall_min,0], [wall_x,wall_min,wall_height], [wall_x,wall_min+door_pos[0]["x"],wall_height], [wall_x,wall_min+door_pos[0]["x"],0]], exterior)
        import_wall(usd_context, floor_path, [[wall_x,wall_min+door_pos[1]["x"],wall_height], [wall_x,wall_min+door_pos[1]["x"],0], [wall_x,wall_max,0], [wall_x,wall_max,wall_height]], exterior)
        import_wall(usd_context, floor_path, [[wall_x,wall_min+door_pos[0]["x"],door_height], [wall_x,wall_min+door_pos[0]["x"],wall_height], [wall_x,wall_min+door_pos[1]["x"],wall_height], [wall_x,wall_min+door_pos[1]["x"],door_height]], exterior)

def import_wall(usd_context, floor_path, wall_polygon, exterior):
    global count
    wall_polygon = np.array(wall_polygon)
    wall_height = calculate_height(wall_polygon)
    wall_width = np.linalg.norm(wall_polygon[1] - wall_polygon[0]) if np.linalg.norm(wall_polygon[1] - wall_polygon[0]) != wall_height else np.linalg.norm(wall_polygon[3] - wall_polygon[0])
    wall_depth = 0.005
    wall_center = np.mean(wall_polygon, axis=0)

    delt_x = 0
    delt_y = 0
    if wall_polygon[0][0] - wall_polygon[2][0] == 0:
        delt_x = 0.05
        scale_x = wall_depth
        scale_y = wall_width/2
    else: 
        delt_y = 0.05
        scale_x = wall_width/2
        scale_y = wall_depth

    if exterior:
        import_objv(usd_context, "/World/obj_" + str(count), floor_path, (wall_center[0], wall_center[1], wall_center[2] - wall_depth/2), (0,0,0), (scale_x, scale_y, wall_height/2))
    else:
        import_objv(usd_context, "/World/obj_" + str(count), floor_path, (wall_center[0] + delt_x, wall_center[1]+ delt_y, wall_center[2] - wall_depth/2), (0,0,0), (scale_x, scale_y, wall_height/2))
    count+=1

def generate_scene(scene_data, usd_file_path, obj_data_usd):
    world = World(stage_units_in_meters=1.0)
    stage = world.stage
    usd_context = omni.usd.get_context()
    stage.DefinePrim("/World", "Xform")

    floor_objects = scene_data["floor_objects"]
    count = 0
    for floor_object in floor_objects:
        assetId = floor_object["assetId"]
        path = obj_data_usd +assetId+ "/raw_model.usd"
        position = (floor_object["position"]["x"], floor_object["position"]["y"], floor_object["position"]["z"])
        rotation = (floor_object["rotation"]["x"], floor_object["rotation"]["y"], floor_object["rotation"]["z"])
        pos, rot = convert_ai2thor_to_isaacsim(position, rotation)
        import_objv(usd_context, "/World/obj_" + str(count), path, pos, rot, (1,1,1))
        count+=1

    small_objects = scene_data["small_objects"]
    for small_object in small_objects:
        assetId = small_object["assetId"]
        path = obj_data_usd +assetId+ "/raw_model.usd"
        position = (small_object["position"]["x"], small_object["position"]["y"], small_object["position"]["z"])
        rotation = (small_object["rotation"]["x"], small_object["rotation"]["y"], small_object["rotation"]["z"])
        pos, rot = convert_ai2thor_to_isaacsim(position, rotation)
        pos[2] = small_object["position"]["y"] - small_object["height"]/2
        import_objv(usd_context, "/World/obj_" + str(count), path, pos, rot, (1,1,1))
        count+=1

    doors = scene_data["doors"]
    walls = scene_data["walls"]

    for wall in walls:
        mat_id = wall["material"]["name"]
        floor_path = obj_data_usd + mat_id
        ids = wall["id"]
        door_mark = -1
        wall_polygon = wall["polygon"]
        wall_polygon = np.array([[p["x"], p["z"], p["y"]] for p in wall_polygon])
        
        exterior = True if wall_polygon[1][0] + wall_polygon[1][1] > wall_polygon[2][0] + wall_polygon[2][1] else False

        for door in doors:
            if door["wall0"] == ids or door["wall1"] == ids:
                door_pos = door["holePolygon"]
                import_door(usd_context,floor_path,door_pos, wall_polygon, exterior)
                door_mark = 1
                break
        if door_mark == -1:
            import_wall(usd_context,floor_path, wall_polygon, exterior)
        
    rooms = scene_data["rooms"]
    for room in rooms:
        mat_id = room["floorMaterial"]["name"]
        floor_path = obj_data_usd + mat_id
        polygon = room["vertices"]
        polygon = np.array(polygon)
        floor_w = max(polygon[:,0]) - min(polygon[:,0])
        floor_l = max(polygon[:,1]) - min(polygon[:,1])
        floor_center = np.mean(polygon, axis=0)
        print(floor_w, floor_l)
        print(floor_center)
        import_objv(usd_context, "/World/obj_" + str(count), floor_path, (floor_center[0], floor_center[1], -0.005), (0,0,0), (floor_w/2, floor_l/2, 0.005))
        count+=1

    omni.usd.get_context().save_as_stage(usd_file_path)

    # Reset and start the simulation
    world.reset()
    simulation_app.update()

scene_files = []  # scene json file path
output_files = []  # output path
obj_data_usd = ""

for scene_file, output_file in zip(scene_files, output_files):
    with open(scene_file, 'r', encoding='utf-8') as file:
        scene_data = json.load(file)
    generate_scene(scene_data, output_file, obj_data_usd)

# Close the simulation app
simulation_app.close()