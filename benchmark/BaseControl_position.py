from omni.isaac.dynamic_control import _dynamic_control
from transformations import euler_from_quaternion,quaternion_from_euler
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.robots import Robot
import math
import time
import numpy as np
import matplotlib.pyplot as plt

class BaseControl:
    def __init__(self,robot,obj_prim=None,next_obj_prim=None):
        """
        Initialize the BaseControl class.
        :param robot: Path to the robot USD.
        :param obj_prim: Object to be grasped.
        :param next_obj_prim: Object to release.
        """
        self.robot = robot
        self.prim = XFormPrim(self.robot)

        # Default linear and angular velocities
        self.linear_velocity = 0.02
        self.angular_velocity = 0.02
        
        # Error tolerances
        self.error_dis = 0.02
        self.error_ang = 0.02

        pos_static, _, _, _ = self.trans_pos()
        self.z = pos_static[2] + 0.02 # Set height to original height + 0.02 to avoid ground collision
        self.obj = obj_prim
        self.next_obj = next_obj_prim
        self.grasp_pos = "/World/stretch/link_gripper_s3_body/joint_grasp_center"
    
    
    def trans_pos(self):
        """
        Get the current position and orientation of the robot.
        :return: Position (x, y, z) and Euler angles (roll, pitch, yaw).
        """
        stretch_baselink_pos = Robot(self.robot)
        position, quaternion  = stretch_baselink_pos.get_world_pose()
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return position, roll, pitch, yaw


    def trans2pi(self,num):
        """
        Convert angle difference to the range [-pi, pi].
        :param num: Angle difference.
        :return: Angle within the range [-pi, pi].
        """
        while num < -math.pi or num > math.pi:
            if num < -math.pi:
                num += 2*math.pi
            else:
                num -= 2*math.pi
        return num


    def move_by_target(self, target, world,img_index,is_grasp=False,orientation=True,is_record = False, camera=None,process_img_folder=None):
        """
        Move the robot to a target position and orientation.
        :param target: Target position (x, y) and orientation (w) in radians.
        :param world: Simulation world.
        :param img_index: Index for image saving.
        :param is_grasp: Whether the robot is currently grasping an object.
        :param orientation: Whether to adjust the final orientation.
        :param is_record: Whether to record images.
        :param camera: Camera object for image capture.
        :param process_img_folder: Folder path to save images.
        """
        # get current position and yaw
        pos, roll, pitch, yaw = self.trans_pos()
        x1, y1, _ = pos
        x2, y2, w2 = target
        angle_to_target = math.atan2(y2 - y1, x2 - x1)
        angle_diff = self.trans2pi(angle_to_target - yaw)
        
        # Turn
        while abs(angle_diff) > self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw + self.angular_velocity * angle_diff / abs(angle_diff) )
            self.prim.set_world_pose(orientation=np.array(euler2q),position=np.array([x1, y1, self.z]))
            # get process image
            if is_record:
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())


            # check if object is grasped
            if is_grasp:
                grasp_pos = Robot(self.grasp_pos)
                grasp_position, quaternion  = grasp_pos.get_world_pose()
                
                _,ori_obj = self.obj.get_world_pose()
                roll_obj, pitch_obj, yaw_obj = euler_from_quaternion(ori_obj)
                euler2q_obj = quaternion_from_euler(roll_obj, pitch_obj, yaw_obj+self.angular_velocity*angle_diff/abs(angle_diff))
                self.obj.set_world_pose(position=np.array(grasp_position),orientation=euler2q_obj)
            
            _, _, _, yaw = self.trans_pos()
            angle_diff = self.trans2pi(angle_to_target - yaw)
            world.step(render=True)

        
        # Move 
        while math.sqrt((x2 - x1)**2 + (y2 - y1)**2) > self.error_dis: 
            dx = x2 - x1
            dy = y2 - y1
            self.prim.set_world_pose(position=np.array([x1 + self.linear_velocity*dx/math.sqrt(dx**2 + dy**2),y1 + self.linear_velocity*dy/math.sqrt(dx**2 + dy**2), self.z]))
            
            if is_record:
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())

            if is_grasp:
                grasp_pos = Robot(self.grasp_pos)
                grasp_position, _  = grasp_pos.get_world_pose()
                self.obj.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)
        
            pos, roll, pitch, yaw = self.trans_pos()
            x1, y1 = pos[0], pos[1]


        # Final turn to target orientation
        if orientation:
            final_angle_diff = w2 - yaw
            final_angle_diff = self.trans2pi(final_angle_diff)
    
            while abs(final_angle_diff) >self.error_ang:
                euler2q = quaternion_from_euler(roll, pitch, yaw+self.angular_velocity*final_angle_diff/abs(final_angle_diff))
                self.prim.set_world_pose(orientation=np.array(euler2q))
                
                if is_record:
                    img_index[0] += 1
                    plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())

                if is_grasp:
                    grasp_pos = Robot(self.grasp_pos)
                    grasp_position, _  = grasp_pos.get_world_pose()
                    
                    _,ori_obj = self.obj.get_world_pose()
                    roll_obj, pitch_obj, yaw_obj = euler_from_quaternion(ori_obj)
                    euler2q_obj = quaternion_from_euler(roll_obj, pitch_obj, yaw_obj+self.angular_velocity*angle_diff/abs(angle_diff))
                    self.obj.set_world_pose(position=np.array(grasp_position),orientation=euler2q_obj)
                
                pos, roll, pitch, yaw = self.trans_pos()
                final_angle_diff = self.trans2pi(w2 - yaw)
                world.step(render=True)
                
        print("current_pos:",pos)

    def turn_by_mode(self, target, world,img_index,is_grasp = False,is_record = False, camera=None,process_img_folder=None):
        """
        Turn the robot by a specified angle.
        :param target: Target angle in radians. Right is positive and eft is negative.
        :param world: Simulation world.
        :param img_index: Index for image saving.
        :param is_grasp: Whether the robot is currently grasping an object.
        :param is_record: Whether to record images.
        :param camera: Camera object for image capture.
        :param process_img_folder: Folder path to save images.
        """
        # get current position and yaw
        pos, roll, pitch, yaw = self.trans_pos()
        newpos=[pos[0],pos[1],self.z]
        prim = XFormPrim("/World/stretch", name="stretch")
        
        # Turn
        final_angle_diff = self.trans2pi(target)

        while abs(final_angle_diff) >self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw + self.angular_velocity * final_angle_diff/abs(final_angle_diff))
            prim.set_world_pose(orientation=np.array(euler2q),position=newpos)
            
            if is_record:
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
                    
            if is_grasp:
                grasp_pos = Robot(self.grasp_pos)
                grasp_position, _  = grasp_pos.get_world_pose()
                self.obj.set_world_pose(position=np.array(grasp_position))
            
            pos, roll, pitch, yaw = self.trans_pos()
            final_angle_diff = self.trans2pi(final_angle_diff - self.angular_velocity * final_angle_diff/abs(final_angle_diff))
            world.step(render=True)
    

    def turn4grasp(self,world,img_index,is_record = False, camera=None,process_img_folder=None):
        """
        Turn the robot to a suitable position for grasping an object.
        :param world: Simulation world.
        :param img_index: Index for image saving.
        :param is_record: Whether to record images.
        :param camera: Camera object for image capture.
        :param process_img_folder: Folder path to save images.
        """
        # get current position and yaw
        pos, roll, pitch, yaw = self.trans_pos()
        x1, y1, _ = pos
        target, _ = self.obj.get_world_pose()
        x2, y2, _ = target
        
        dx = x2 - x1
        dy = y2 - y1
        
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.trans2pi(angle_to_target - yaw + math.pi/2) # +pi/2 is to make the grasp turn to the object


        # Turn
        while abs(angle_diff) > self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw + self.angular_velocity*angle_diff/abs(angle_diff))
            self.prim.set_world_pose(orientation=np.array(euler2q)) 
            
            if is_record:
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())

            _, _, _, yaw = self.trans_pos()
            angle_diff = self.trans2pi(angle_to_target - yaw + math.pi/2)
            world.step(render=True)


    def turn4release(self,world,img_index,is_grasp=True,obj_prim=None,is_record=False, camera=None,process_img_folder=None):
        """
        Turn the robot to a suitable position for releasing an object.
        :param world: Simulation world.
        :param img_index: Index for image saving.
        :param is_grasp: Whether the robot is currently grasping an object.
        :param obj_prim: Object to release.
        :param is_record: Whether to record images.
        :param camera: Camera object for image capture.
        :param process_img_folder: Folder path to save images.
        """
        # get current position and yaw
        pos, roll, pitch, yaw = self.trans_pos()
        x1, y1, _ = pos
        target, _ = obj_prim.get_world_pose()
        x2, y2, _ = target
        angle_to_target = math.atan2(y2 - y1, x2 - x1)
        angle_diff = self.trans2pi(angle_to_target - yaw + math.pi/2) # +pi/2 is to make the grasp turn to the object


        # Turn
        while abs(angle_diff) > self.error_ang:
            euler2q = quaternion_from_euler(roll, pitch, yaw+self.angular_velocity*angle_diff/abs(angle_diff))
            self.prim.set_world_pose(orientation=np.array(euler2q))
            if is_record:
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)
            # Check if the object is being grasped
            if is_grasp:
                grasp_pos = Robot(self.grasp_pos)
                grasp_position, quaternion  = grasp_pos.get_world_pose()
                _,ori_obj = self.obj.get_world_pose()
                roll_obj, pitch_obj, yaw_obj = euler_from_quaternion(ori_obj)
                euler2q_obj = quaternion_from_euler(roll_obj, pitch_obj, yaw_obj+self.angular_velocity*angle_diff/abs(angle_diff))
                self.obj.set_world_pose(position=np.array(grasp_position),orientation=euler2q_obj)

            _, _, _, yaw = self.trans_pos()
            angle_diff = self.trans2pi(angle_to_target - yaw + math.pi/2)
            world.step(render=True)



            
