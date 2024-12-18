from navigate_python.map_show import *
from navigate_python.navigate import *
from navigate_python.dstar_lite import *
from navigate_python.discretize_map import *
import random
import math
import pandas as pd
import os
import omni.isaac.core.utils.prims as prim_utils

def init_robot_pos(map_file,scene_offset=None):
    '''
    Initialize the robot's position by randomly selecting a location from the map.
    '''
    hm = HeightMap(map_file,bias=scene_offset) # Define the map class
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1) # Define the navigator class
    navigator.planner.compute_cost_map()
    # Randomly obtain the initial position of the robot
    random_x = random.randint(math.floor(hm.X) -1, math.floor(hm.X + hm.height * hm.resolution))
    random_y = random.randint(math.floor(hm.Y) -1, math.floor(hm.Y + hm.width * hm.resolution))
    robot_map_pos = navigator.planner.real2map([random_x,random_y])
    robot_real_pos = navigator.planner.map2real(robot_map_pos)
    print("Robot initial position", robot_real_pos)
    return robot_real_pos


def check_wall_block(img_path,obj_world_pos,robot_world_pos,scene_offset=None):
    '''
    Check if there are any walls blocking the path between two positions.
    Walls are represented by a gray value of 2 in the map.
    If the line connecting the two points passes through a gray value of 2, it indicates a wall, and the check fails.
    '''
    hm = HeightMap(img_path,bias=scene_offset) # Define the map class
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1)  # Define the navigator class
    navigator.planner.compute_cost_map()

    # Get the map coordinates of the object and the robot
    obj_map_pos = navigator.planner.real2map(obj_world_pos,reachable_assurance=False)
    robot_map_pos = navigator.planner.real2map(robot_world_pos,reachable_assurance=False)
    print("obj_map_pos",obj_map_pos)
    print("robot_map_pos",robot_map_pos)
    # Calculate whether the line connecting the two points on the map passes through a gray region
    x0, y0 = obj_map_pos
    x1, y1 = robot_map_pos

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
  
    x, y = x0, y0  # Starting point
    sx = -1 if x0 > x1 else 1 # Increment step
    sy = -1 if y0 > y1 else 1

    if dx > dy: # Determine the main axis
        err = dx / 2.0
        while x != x1:
            if hm_map[x][y] == 2:  # Check if it is a gray region
                print("Wall detected,fail,x,y",x,y)  
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            if hm_map[x][y] == 2:
                print("Wall detected,fail,x,y",x,y)  
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    return True



def check_path_and_table_placement(img_path,obj_world_pos,robot_world_pos,next_obj_range,scene_offset=None):
    '''
    Check if there are any walls blocking the path between two positions and if the object is placed on the table.
    Walls are represented by a gray value of 2 in the map.
    If the line connecting the two points passes through a gray value of 2, it indicates a wall, and the check fails.
    '''
    hm = HeightMap(img_path,bias=scene_offset)
    xy_range = hm.compute_range()
    hm.make_map()
    hm_map = hm.get_map()
    navigator = Navigator(area_range=xy_range, map=hm_map, scale_ratio=1)
    navigator.planner.compute_cost_map()

     # Get the map coordinates of the object
    obj_map_pos = navigator.planner.real2map(obj_world_pos,reachable_assurance=False)

    # Get the range of the table
    minx,miny,maxx,maxy = next_obj_range

    # Calculate whether the line connecting the two points on the map passes through a gray region
    robot_map_pos = navigator.planner.real2map(robot_world_pos,reachable_assurance=False)
    x0, y0 = obj_map_pos
    x1, y1 = robot_map_pos

    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
  
    x, y = x0, y0 # Starting point
    sx = -1 if x0 > x1 else 1 # Increment step
    sy = -1 if y0 > y1 else 1

    if dx > dy: # Determine the main axis
        err = dx / 2.0
        while x != x1:
            if hm_map[x][y] == 2:  # Check if it is a gray region
                print("fail,x,y",x,y)  
                return False
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            if hm_map[x][y] == 2:
                print("fail,x,y",x,y)  
                return False
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    
    # check if the object is on the table plane
    if minx <= obj_world_pos[0] <= maxx and  miny <= obj_world_pos[1] <= maxy:
        return True
    else:
        print("Out of range, x:",minx,maxx,"y:",miny,maxy)
        print("Real-world coordinates of the object",obj_world_pos)
        return False


def calculate_path_length(path):
    '''
    Calculate the total distance of a series of path points.
    '''
    total_distance = 0.0
    for i in range(1, len(path)):
        x1, y1 = path[i - 1]
        x2, y2 = path[i]
        
        distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
        total_distance += distance
    
    return total_distance


def append_to_csv(file_path, json_file_path, is_success, path_length, NE):
    '''
    Append results to a CSV file, including navigation and manipulation data.
    '''
    new_data = pd.DataFrame([{
        "json_file_path": json_file_path,
        "is_success": is_success,
        "path_length": path_length,
        "NE": NE
    }])
    
    # Read the CSV file, or create a new DataFrame if it does not exist
    try:
        df = pd.read_csv(file_path)
        df = pd.concat([df, new_data], ignore_index=True)
        
        # Write the updated DataFrame back to the CSV file
        df.to_csv(file_path, index=False)
        print("Successfully added data to the file:", file_path)
    except FileNotFoundError:
        # If the file does not exist, create a new file and write the data
        new_data.to_csv(file_path, index=False)
        print("File does not exist, created a new file and wrote the data:", file_path)


def append_to_csv2(file_path, json_file_path, is_success, path_length1, path_length2,NE1,NE2,is_success1,is_success2):
    '''
    Append detailed results to a CSV file.
    '''
    # Create a dictionary for the new row of data
    new_data = pd.DataFrame([{
        "json_file_path": json_file_path,
        "is_success": is_success,
        "path_length1": path_length1,
        "path_length2": path_length2,
        "NE1": NE1,
        "NE2": NE2,
        "is_success1": is_success1,
        "is_success2": is_success2
    }])
    
    try:
        df = pd.read_csv(file_path)
        df = pd.concat([df, new_data], ignore_index=True)
        
        df.to_csv(file_path, index=False)
        print("Successfully added data to the file:", file_path)
    except FileNotFoundError:
        # If the file does not exist, create a new file and write the data
        new_data.to_csv(file_path, index=False)
        print("File does not exist, created a new file and wrote the data:", file_path)


def create_imgs_folder(json_file_path):
    '''
    Record the image data corresponding to the task and create the corresponding folder.
    '''
    json_file_dir = os.path.dirname(json_file_path)
    parent_folder = os.path.dirname(json_file_dir)
    folder_number = os.path.basename(json_file_dir)

    new_folder_name = f"{folder_number}_imgs"
    new_folder_path = os.path.join(parent_folder, new_folder_name)

    os.makedirs(new_folder_path, exist_ok=True)

    return new_folder_path


    
def add_boundary_walls(width=80, height=60, wall_height=5, wall_thickness=1, center=(0, 0)):
    """
    Add boundary walls and ground around the scene, and adjust the center position of the walls.

    Args:
    - width (float): The boundary size of the scene in the X direction, which is the length of the horizontal wall.
    - height (float): The boundary size of the scene in the Y direction, which is the length of the vertical wall.
    - wall_height (float): The height of the walls.
    - wall_thickness (float): The thickness of the walls.
    - center (tuple): The center position of the walls, formatted as (x, y).
    """
    # Wall dimensions
    wall_x_size = width + 2 * wall_thickness  # Length of the wall in the X, Y direction
    wall_y_size = height + 2 * wall_thickness

    # Coordinates of the center position
    center_x, center_y = center

    # Create the top wall (horizontal cube)
    prim_utils.create_prim(
        prim_path="/World/Wall_Top",
        prim_type="Cube",
        translation=(center_x, center_y + height / 2 + wall_thickness / 2, wall_height / 2),
        scale=(wall_x_size, wall_thickness, wall_height)
    )

    # Create the bottom wall (horizontal cube)
    prim_utils.create_prim(
        prim_path="/World/Wall_Bottom",
        prim_type="Cube",
        translation=(center_x, center_y - height / 2 - wall_thickness / 2, wall_height / 2),
        scale=(wall_x_size, wall_thickness, wall_height)
    )

    # Create the left wall (vertical cube)
    prim_utils.create_prim(
        prim_path="/World/Wall_Left",
        prim_type="Cube",
        translation=(center_x - width / 2 - wall_thickness / 2, center_y, wall_height / 2),
        scale=(wall_thickness, wall_y_size, wall_height)
    )

    # Create the right wall (vertical cube)
    prim_utils.create_prim(
        prim_path="/World/Wall_Right",
        prim_type="Cube",
        translation=(center_x + width / 2 + wall_thickness / 2, center_y, wall_height / 2),
        scale=(wall_thickness, wall_y_size, wall_height)
    )

    # Create the ground
    prim_utils.create_prim(
        prim_path="/World/Wall_Bottom_Surface",
        prim_type="Cube",
        translation=(center_x, center_y, -1),
        scale=(wall_x_size, wall_y_size, wall_thickness)
    )
    
    
