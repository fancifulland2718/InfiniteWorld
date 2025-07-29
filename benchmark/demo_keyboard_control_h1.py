from navigate_python.map_show import *
from navigate_python.navigate import *
from navigate_python.dstar_lite import *
from navigate_python.discretize_map import *
from arguments import get_args
import sys
sys.path.append("You should input your isaac-sim's path" + "/exts/omni.isaac.core/")
sys.path.append("You should input your isaac-sim's path" + "/exts/omni.isaac.examples/")


def main(json_file):
    '''
    H1 Humanoid keyboard control demo - Fixed version
    :param json_file: task json file
    '''

    import os
    os.environ["OMNI_KIT_ACCEPT_EULA"] = "YES"
    from omni.isaac.kit import SimulationApp
    config = {
        "width": "1280",
        "height": "720",
        "headless": False,
    }
    simulation_app = SimulationApp(config)

    import omni.isaac.core.utils.prims as prim_utils
    import numpy as np
    import json
    from omni.isaac.core import World
    from omni.isaac.core.prims import GeometryPrim
    import carb
    import omni.appwindow
    import omni.timeline

 
    # Import H1 humanoid controller
    from H1KeyboardController import H1KeyboardController
    from benchmark_utils import init_robot_pos, add_boundary_walls

    args = get_args()
    scene_root_path = args.scene_root_path

    # Extract target scene ID
    with open(json_file, 'r', encoding='utf-8') as file:
        json_data = json.load(file)
    
    scene_id = json_data['Scene']
    scene_path = scene_root_path + scene_id + "/" + scene_id + ".usd"
    
    # Create world with proper physics settings
    my_world = World(
        stage_units_in_meters=1.0,
        physics_dt=1.0 / 200.0,  # 5ms physics step - CRITICAL for H1 stability
        rendering_dt=8.0 / 200.0  # 40ms render step - matches original
    )
    
    reset_needed = False

    # Initialize robot position using random points from the map
    map_path = args.occupancy_map_root_path + scene_id
    img_folder = os.listdir(map_path)
    png_files = [file for file in img_folder if file.endswith('.png')]
    map_name = png_files[0]

    map_file = map_path + "/" + map_name
    h1_pos = init_robot_pos(map_file=map_file)

    # Create H1 humanoid robot with fixed controller
    h1_controller = H1KeyboardController(
        prim_path="/World/H1",
        usd_path="../robot/h1.usd",
        name="H1", 
        position=np.array([h1_pos[0], h1_pos[1], 1.05])  # Use standard height
    )

    # Add scene
    prim_utils.create_prim(
        prim_type="Xform",
        prim_path="/World/Scene",
        usd_path=scene_path,
        scale=[1, 1, 1],
        translation=np.array([0, 0, 0]),
        orientation=[1, 0, 0, 0]
    )
    GeometryPrim("/World/Scene", collision=True)

    # Add proper ground plane with realistic friction
    my_world.scene.add_default_ground_plane(
        z_position=0,
        name="default_ground_plane", 
        prim_path="/World/defaultGroundPlane",
        static_friction=0.2,   # Realistic friction values
        dynamic_friction=0.2,
        restitution=0.01,
    )

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

    # Setup keyboard input properly
    appwindow = omni.appwindow.get_default_app_window()
    input_interface = carb.input.acquire_input_interface()
    keyboard = appwindow.get_keyboard()

    physics_ready = False

    def on_physics_step(step_size):
        nonlocal physics_ready
        if physics_ready:
            h1_controller.advance(step_size)
        else:
            physics_ready = True

    def sub_keyboard_event(event, *args, **kwargs):
        return h1_controller.handle_keyboard_event(event, *args, **kwargs)

    # Subscribe to keyboard events
    keyboard_sub = input_interface.subscribe_to_keyboard_events(keyboard, sub_keyboard_event)
    
    # Add physics callback
    my_world.add_physics_callback("physics_step", callback_fn=on_physics_step)
    
    # Setup timeline event handling for proper reset
    timeline = omni.timeline.get_timeline_interface()
    
    def timeline_callback(event):
        if h1_controller:
            h1_controller.post_reset()
            
    event_timer_callback = timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
        int(omni.timeline.TimelineEventType.STOP), timeline_callback
    )

    print("H1 Humanoid Keyboard Control Demo")
    print("Controls:")
    print("  NUMPAD 8/UP: Move forward")
    print("  NUMPAD 4/LEFT: Turn left") 
    print("  NUMPAD 6/RIGHT: Turn right")
    print("  ESC: Exit")

    async def setup_async():
        nonlocal physics_ready
        physics_ready = False
        await my_world.play_async()
        h1_controller.initialize()
        physics_ready = True

    # Initialize the world and robot
    import asyncio
    asyncio.ensure_future(setup_async())

    while simulation_app.is_running():
        my_world.step(render=True)
        if my_world.is_stopped() and not reset_needed:
            reset_needed = True
        if my_world.is_playing():
            if reset_needed:
                my_world.reset()
                reset_needed = False
                physics_ready = False
                # Re-initialize after reset
                asyncio.ensure_future(setup_async())

    # Cleanup
    if my_world.physics_callback_exists("physics_step"):
        my_world.remove_physics_callback("physics_step")
    event_timer_callback = None
    
    simulation_app.close()


if __name__ == "__main__":
    args = get_args()
    json_path = args.keyboard_json_path # you should specify the json file path
    main(json_path)