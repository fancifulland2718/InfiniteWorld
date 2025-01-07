import numpy as np
from scipy.spatial.transform import Rotation as R
from omni.isaac.dynamic_control import _dynamic_control
import math
from omni.isaac.core.robots import Robot
from transformations import euler_from_quaternion
import matplotlib.pyplot as plt

class BaseGrasp:
    def __init__(self, arm1, arm2, arm3, arm4, lift, grasp, obj_prim, robot):
        self.arm = 0 # Length of the robotic arm
        self.lift = 0 # Height of the robotic arm
        self.grasp = 0 # Angle of the robotic arm's gripper
        self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift) # Offset from the end of the robotic arm to the center of the vehicle in the vehicle coordinate system

        self.dc = _dynamic_control.acquire_dynamic_control_interface()

        # Define lift joint
        self.articulation_lift = self.dc.get_articulation(lift)
        self.dof_lift = self.dc.find_articulation_dof(self.articulation_lift, "joint_lift")

        # Define arm joint
        self.articulation_arm1 = self.dc.get_articulation(arm1)
        self.dof_arm1 = self.dc.find_articulation_dof(self.articulation_arm1, "joint_arm_l0")

        self.articulation_arm2 = self.dc.get_articulation(arm2)
        self.dof_arm2 = self.dc.find_articulation_dof(self.articulation_arm2, "joint_arm_l1")

        self.articulation_arm3 = self.dc.get_articulation(arm3)
        self.dof_arm3 = self.dc.find_articulation_dof(self.articulation_arm3, "joint_arm_l2")

        self.articulation_arm4 = self.dc.get_articulation(arm4)
        self.dof_arm4 = self.dc.find_articulation_dof(self.articulation_arm4, "joint_arm_l3")


        # Define grasp joints
        self.articulation_grasp_left = self.dc.get_articulation(grasp)
        self.dof_grasp_left = self.dc.find_articulation_dof(self.articulation_grasp_left, "joint_gripper_finger_left")

        self.articulation_grasp_right = self.dc.get_articulation(grasp)
        self.dof_grasp_right = self.dc.find_articulation_dof(self.articulation_grasp_right, "joint_gripper_finger_right")

        # Grasping status
        self.is_grasping = False
        # Prim path of the object being grasped
        self.obj_prim = obj_prim
        self.robot = robot
        self.arm_speed = 0.01
        self.lift_speed = 0.01

    def transform_to_world(self, pos, orient):
        """
        Transform a 3D coordinate from the vehicle coordinate system to the world coordinate system.
        :param pos: 3D coordinate in the vehicle coordinate system (x, y, z)
        :param orient: Quaternion representing the orientation of the vehicle relative to the world coordinate system (w, x, y, z)
        :return: 3D coordinate in the world coordinate system (x, y, z)
        """
        
        rotation = R.from_quat(orient) # Convert quaternion to rotation matrix
        rotation_matrix = rotation.as_matrix() 
        world_pos = np.dot(rotation_matrix, self.d) + pos

        return world_pos

    def transform_to_base(self, pos, orient, target):
        """
        Calculate the robotic arm parameters based on the target position (requires moving to the correct position first).
        :param pos: Coordinate in the vehicle coordinate system (x, y, z)
        :param orient: Quaternion representing the orientation of the vehicle relative to the world coordinate system (w, x, y, z)
        :param target: Target position in the world coordinate system (x, y, z)
        :return: Parameters for the robotic arm [length, height difference]
        """

        arm_length = 0.52
        rotation = R.from_quat(orient)
        rotation_matrix = rotation.as_matrix() 
        pos_transformed = np.dot(rotation_matrix.T, (target - pos))
        length = abs(-0.4 - pos_transformed[1])
        if length > arm_length:
            length = arm_length
        arg4grasp = [length,abs(pos_transformed[2] - 0.06)]
        return arg4grasp



    def rpy2R(self,rpy): 
        """
        Convert the roll-pitch-yaw angles of the Stretch robot to a rotation matrix.
        :param rpy: [roll, pitch, yaw] in radians
        :return: Rotation matrix
        """
        rot_x = np.array([[1, 0, 0],
                        [0, math.cos(rpy[0]), -math.sin(rpy[0])],
                        [0, math.sin(rpy[0]), math.cos(rpy[0])]])
        rot_y = np.array([[math.cos(rpy[1]), 0, math.sin(rpy[1])],
                        [0, 1, 0],
                        [-math.sin(rpy[1]), 0, math.cos(rpy[1])]])
        rot_z = np.array([[math.cos(rpy[2]), -math.sin(rpy[2]), 0],
                        [math.sin(rpy[2]), math.cos(rpy[2]), 0],
                        [0, 0, 1]])
        R = np.dot(rot_z, np.dot(rot_y, rot_x))
        return R



    def grasp_by_target(self, target, world, camera=None, img_index=None, is_record=False, process_img_folder=None):
        """
        Move the robotic arm to the target position and grasp the object.
        :param target: Target position [length, height]
        :param world: Simulation world object
        """

        time_step = int(target[1]/self.lift_speed)
        for i in range(time_step):
            self.dc.set_dof_position_target(self.dof_lift, self.lift_speed * i)
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)  
        # self.dc.set_dof_position_target(self.dof_lift, target[1]) # Lift the robotic arm to the specified height
        self.lift = target[1] # Update lift height
        
        for i in range(0,101):
            self.dc.set_dof_position_target(self.dof_grasp_left,i * 0.1) # Open the gripper
            self.dc.set_dof_position_target(self.dof_grasp_right,i * 0.1)
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)


        time_step = int(target[0]/4 / self.arm_speed)
        dof_arms = [self.dof_arm1, self.dof_arm2, self.dof_arm3, self.dof_arm4]
        for i in range(len(dof_arms)):
            for j in range(time_step):
                self.dc.set_dof_position_target(dof_arms[i],self.arm_speed * j)
                if is_record and camera.get_rgb().any():
                    img_index[0] += 1
                    plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
                world.step(render=True)

        # self.dc.set_dof_position_target(self.dof_arm1,target[0]/4)
        # self.dc.set_dof_position_target(self.dof_arm2,target[0]/4)
        # self.dc.set_dof_position_target(self.dof_arm3,target[0]/4)
        # self.dc.set_dof_position_target(self.dof_arm4,target[0]/4)
        self.arm = target[0] # Update arm length


        for i in range(100,0,-1):
            self.dc.set_dof_position_target(self.dof_grasp_left,i * 0.1) # Close the gripper
            self.dc.set_dof_position_target(self.dof_grasp_right,i * 0.1)
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)
        
        self.is_grasping = True
        
        tmp_length = target[0] / 4
        while tmp_length > 0:
            self.dc.set_dof_position_target(self.dof_arm1,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm2,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm3,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm4,tmp_length)
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)
            tmp_length -= self.arm_speed
            if tmp_length <= 0:
                tmp_length = 0
            self.arm = tmp_length

            # Attach the object to the robotic arm's gripper
            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos()
            
            R = self.rpy2R([roll,pitch,yaw]) # Obtain the rotation matrix between the world and vehicle coordinates
            R = np.linalg.inv(R)
            # Multiply d by the rotation matrix and add the current position to get the world coordinates of the gripper
            self.d = np.dot(self.d,R)
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)    

    
    def release_by_target(self, target, world, is_record=False, camera=None, process_img_folder=None, img_index=None):
        '''
        Move the robotic arm to the target position and release the object.
        :param target: Target position [length, height]
        :param world: Simulation world object
        '''
        tmp_height = target[1]
        while abs(tmp_height - self.lift) > 0.01 and self.lift <= 1:
            self.dc.set_dof_position_target(self.dof_lift,self.lift + self.lift_speed * (tmp_height - self.lift)/abs(tmp_height - self.lift))
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)
            
            self.lift = self.lift + self.lift_speed * (tmp_height - self.lift)/abs(tmp_height - self.lift)

            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos()
            
            R = self.rpy2R([roll,pitch,yaw])
            R = np.linalg.inv(R)
            self.d = np.dot(self.d,R)
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)    


        tmp_length = 0
        while tmp_length < target[0] / 4:
            self.dc.set_dof_position_target(self.dof_arm1,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm2,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm3,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm4,tmp_length)
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)
            
            tmp_length += self.arm_speed
            world.step(render=True)
            self.arm = tmp_length*4

            self.d = (-0.02, -0.4 - self.arm, 0.06 + self.lift)
            cur_position,roll,pitch,yaw = self.trans_pos()
            
            R = self.rpy2R([roll,pitch,yaw])
            R = np.linalg.inv(R)
            self.d = np.dot(self.d,R)
            grasp_position = cur_position + self.d
            self.obj_prim.set_world_pose(position=np.array(grasp_position))
            world.step(render=True)    


        for i in range(0,101):
            self.dc.set_dof_position_target(self.dof_grasp_left,i/10) # Open the gripper
            self.dc.set_dof_position_target(self.dof_grasp_right,i/10) 
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)

        self.is_grasping = False  # Set the grasping status to false 

        tmp_length = target[0] / 4
        while tmp_length > 0:
            self.dc.set_dof_position_target(self.dof_arm1,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm2,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm3,tmp_length)
            self.dc.set_dof_position_target(self.dof_arm4,tmp_length)
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
            world.step(render=True)
            tmp_length -= self.arm_speed
            if tmp_length <= 0:
                tmp_length = 0
            self.arm = tmp_length # 更新arm长度
            
        # self.dc.set_dof_position_target(self.dof_arm4,0)
        # self.dc.set_dof_position_target(self.dof_arm3,0)
        # self.dc.set_dof_position_target(self.dof_arm2,0)
        # self.dc.set_dof_position_target(self.dof_arm1,0)
        # self.arm = 0 # Update arm length
        world.step(render=True)

        for i in range(100,0,-1):
            self.dc.set_dof_position_target(self.dof_grasp_left,i * 0.1) # Close the gripper
            self.dc.set_dof_position_target(self.dof_grasp_right,i * 0.1) 
            if is_record and camera.get_rgb().any():
                img_index[0] += 1
                plt.imsave(process_img_folder + "/"+f"frame_{img_index[0]:04d}.png",camera.get_rgb())
                    
            world.step(render=True)


    def trans_pos(self):
        """
        Get the position and orientation of the Stretch robot's base link in the world coordinate system.
        :return: Position (x, y, z), Roll, Pitch, Yaw
        """
        stretch_baselink_pos = Robot(self.robot)
        position, quaternion  = stretch_baselink_pos.get_world_pose()
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        return position, roll, pitch , yaw
        

