import argparse
def get_args():
    parser = argparse.ArgumentParser("GT")

    parser.add_argument("--scene_root_path",type=str,default="/data1/lfwj/hssd_scenes/final_selected_usd/")
    parser.add_argument("--occupancy_map_root_path",type=str,default="/data1/lfwj/hssd_scenes/final_selected_usd/")
    parser.add_argument("--robot_usd_path",type=str,default="robot/stretch.usd")
    
    parser.add_argument("--obj_in_isaac_path",type=str,default="/World/Scene/floorplan/furniture/")
    parser.add_argument("--robot_path",type=str,default="/World/stretch")
    parser.add_argument("--lift_path",type=str,default="/World/stretch/link_mast")
    parser.add_argument("--arm1_path",type=str,default="/World/stretch/link_arm_l1")
    parser.add_argument("--arm2_path",type=str,default="/World/stretch/link_arm_l2")
    parser.add_argument("--arm3_path",type=str,default="/World/stretch/link_arm_l3")
    parser.add_argument("--arm4_path",type=str,default="/World/stretch/link_arm_l4")
    parser.add_argument("--grasp_path",type=str,default="/World/stretch/link_gripper_s3_body")
    parser.add_argument("--camera_path",type=str,default="/World/stretch/camera_color_frame/rgb_camera") 

    args = parser.parse_args([])

    return args