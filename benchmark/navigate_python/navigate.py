#!/usr/bin/env python3
# -*- encoding: utf-8 -*-
import math
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np

from .dstar_lite import DStarLite, euclidean_distance

class Navigator:
    '''
    Navigation class
    '''

    def __init__(self,
                 area_range,
                 map,
                 scale_ratio=1,
                 resolution=0.05,
                 react_radius=50,
                 vision_radius=math.pi*3/7,
                 max_iteration=100):
        self.area_range = area_range        # Actual coordinate range of the map [xmin, xmax, ymin, ymax]
        self.map = map                      # Scaled and discretized map array (X, Y)
        self.scale_ratio = scale_ratio      # Scale ratio of the map
        self.resolution = resolution
        self.max_iteration = max_iteration  # Maximum number of planning iterations

        self.planner = DStarLite(area_range=area_range, map=map, scale_ratio=scale_ratio, react_radius=react_radius, resolution=resolution, vision_radius=vision_radius)
        self.yaw = None

    def validate_goal(self, goal):
        '''
            Validate and normalize the goal position.
        '''
        return self.planner.map2real(self.planner.real2map(goal))

    def navigate(self, goal: (float, float), pos: (float, float), animation=True):
        '''
            Single navigation from start to goal, both in world coordinates.
        '''
        goal = np.array(self.validate_goal(goal))  # Validate and normalize the goal position
        pos = np.array(pos)   # Current position and orientation of the robot
        self.yaw = None
        print('------------------navigation_start----------------------')
        print("goal",goal)

        path = self.planner.planning(pos, goal)

        self.planner.reset()  # Reset variables after one round of navigation
        map_path = []
        for p in path:
            map_path.append(self.planner.real2map(p))
        return path, map_path
    
    def navigate_(self, goal: (int, int), pos: (float, float), animation=True):
        '''
            Single navigation from start to goal, start in world coordinates and goal in map coordinates.
        '''
        goal = np.array(self.planner.map2real(goal))
        pos = np.array(pos)  
        self.yaw = None
        print('------------------navigation_start----------------------')

        path = self.planner.planning(pos, goal)

        self.planner.reset()
        map_path = []
        for p in path:
            map_path.append(self.planner.real2map(p))
        return path, map_path

    def _navigate_(self, goal: (int, int), pos: (int, int), animation=True):
        '''
            Single navigation from start to goal, both in map coordinates.
        '''
        goal = np.array(self.planner.map2real(goal))
        pos = np.array(self.planner.map2real(pos))  
        self.yaw = None
        print('------------------navigation_start----------------------')

        path = self.planner.planning(pos, goal)

        self.planner.reset()
        map_path = []
        for p in path:
            map_path.append(self.planner.real2map(p))
        return path, map_path

