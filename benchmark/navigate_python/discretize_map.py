# !/usr/bin/env python3
# -*- encoding: utf-8 -*-

import matplotlib.pyplot as plt
import numpy as np
import pickle
import os

from scipy.ndimage import binary_dilation


def draw_grid_map(grid_map, path=None, goal=None, save_path=None):
    """
    Draw a grid map and optionally plot a path and goal position.
    
    Parameters:
    - grid_map: 2D array representing the grid map.
    - path: List of coordinate pairs (x, y) representing the path.
    - goal: Coordinate pair (x, y) representing the goal position.
    - save_path: String representing the file path to save the image.
    """
    # Generate a new map image
    plt.imshow(grid_map, cmap='binary', alpha=0.5, origin='lower')  

    # Plot axes
    plt.xlabel('y')
    plt.ylabel('x')

    # Display grid lines
    plt.grid(color='black', linestyle='-', linewidth=0.5)

    if path:
        px = []
        py = []
        for p in path:
            px.append(p[0])
            py.append(p[1])
        plt.plot(py[:-1], px[:-1])
        if goal:
            plt.scatter(py[0], px[0], s=10, c='red')
            plt.scatter(goal[1], goal[0], s=10, c='blue')
        print("path plot")

    plt.show()
    if save_path:
        plt.savefig(save_path)


def discretize_map(scene, scale_ratio):
    """
    Discretizes a given scene into a grid map based on the specified scale ratio.
    Each cell in the grid map is marked as reachable or unreachable based on the scene's reachability check.

    Parameters:
    - scene: An object representing the scene with a method `reachable_check(x, y, Yaw)` to check reachability.
    - scale_ratio: A float representing the scaling factor to convert scene coordinates to grid coordinates.
    """
    X = int(950 / scale_ratio)  # Number of sampling points 
    Y = int(1850 / scale_ratio)
    map = np.zeros((X, Y))

    for x in range(X):
        for y in range(Y):
            if not scene.reachable_check(x * scale_ratio - 350, y * scale_ratio - 400, Yaw=0):
                map[x, y] = 1
                print(x, y)

    file_name = 'map_'+str(scale_ratio)+'.pkl'
    if not os.path.exists(file_name):
        open(file_name, 'w').close()
    with open(file_name, 'wb') as file:
        pickle.dump(map, file)
    print('Save successful')


def expand_obstacles(scale_ratio, expand_range=1):
    '''
    Expand the edges of obstacles in the map.
    '''
    file_name = 'map_'+str(scale_ratio)+'.pkl'
    dilated_file_name = 'map_'+str(scale_ratio)+'_e'+str(expand_range)+'.pkl'

    if os.path.exists(file_name):
        with open(file_name, 'rb') as file:
            map = pickle.load(file)

    dilated_map = binary_dilation(map, iterations=expand_range)

    if not os.path.exists(dilated_file_name):
        open(dilated_file_name, 'w').close()
    with open(dilated_file_name, 'wb') as file:
        pickle.dump(dilated_map, file)
    print('Save successful')


def show_map(file_name, path=None):
    """
    Load and display a grid map from a pickle file. Optionally, overlay a path on the map.
    """
    if os.path.exists(file_name):
        with open(file_name, 'rb') as file:
            map = pickle.load(file)
            draw_grid_map(map, path)


def show_map_(map, path=None, goal=None,save_path=None):
    draw_grid_map(map, path, goal, save_path=save_path)


