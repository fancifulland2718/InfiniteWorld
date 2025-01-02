from navigate_python.map_show import *
from navigate_python.navigate import *
from navigate_python.dstar_lite import *
from navigate_python.discretize_map import *
from arguments import get_args

def main(json_file):
    '''
    :param json_file: task json file
    '''

    import os
    os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES" # Set environment variable to accept OMNI Kit EULA (End User License Agreement)
    from omni.isaac.kit import SimulationApp
    config = {
    "width": "1280",
    "height": "720",
    "headless": False,
    }
    simulation_app = SimulationApp(config) # Configure simulation application, specify window size and whether to run in headless mode

    import numpy as np
    import json
    from omni.isaac.core import World
    from omni.isaac.core.prims import GeometryPrim
    import omni.isaac.core.utils.prims as prim_utils


    from BaseControl_position import BaseControl
    from some_function import init_robot_pos, add_boundary_walls
 
    args = get_args()
    # Import scene
    scene_root_path = args.scene_root_path

    # Extract target scene ID
    with open(json_file, 'r', encoding='utf-8') as file:
        json_data = json.load(file)
    
    scene_id = json_data['Scene']
    scene_path = scene_root_path + scene_id + "/" + scene_id + ".usd"
    
    # Create world
    my_world = World(stage_units_in_meters=1.0)
    reset_needed = False

    # Add default ground plane to the scene
    add_boundary_walls()

     # Initialize robot position using random points from the map
    map_path = args.occupancy_map_root_path + scene_id
    img_folder = os.listdir(map_path)
    png_files = [file for file in img_folder if file.endswith('.png')]
    map_name = png_files[0]

    map_file=map_path+"/"+map_name
    stretch_pos = init_robot_pos(map_file=map_file)

    # Create robot
    stretch_path = args.robot_usd_path
    prim_utils.create_prim(
    prim_type="Xform",
    prim_path = "/World/stretch",
    usd_path=stretch_path,
    translation = np.array([stretch_pos[0],stretch_pos[1],0.0])
    )


    prim_utils.create_prim(
        prim_type="Xform",
        prim_path = "/World/Scene",
        usd_path = scene_path,
        scale=[1, 1, 1],
        translation=np.array([0, 0, 0]),
        orientation=[1,0,0,0]
    )
    GeometryPrim("/World/Scene", collision=True)


    # Add lighting
    prim_utils.create_prim(
        "/World/Light_3",
        "DomeLight",
        position=np.array([0.0, 0.0, 0.0]),
        attributes={
            "inputs:intensity": 1e3,
            "inputs:color": (1.0, 1.0, 1.0)
        }
    )

    robot_path = args.robot_path

    while simulation_app.is_running():
        my_world.step(render=True)
        if my_world.is_stopped() and not reset_needed:
            reset_needed = True
        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                reset_needed = False

                # Define movement and grasping class
                base_control = BaseControl(robot=robot_path,obj_prim=None,next_obj_prim=None)   
                base_control.move_by_keyboard(world=my_world)  
                my_world.step(render=True)
    simulation_app.close()


args = get_args()
json_path = args.keyboard_json_path
main(json_path)

