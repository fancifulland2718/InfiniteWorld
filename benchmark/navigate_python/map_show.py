import os
import pickle
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import re


def read_bin(file):
    if os.path.exists(file):
        raw_map = np.loadtxt(file)
    else:
        print("file doesn't exit!")
        return 0
    return raw_map


class HeightMap:
    def __init__(self, file,bias = None):
        if bias:
            bias_x ,bias_y = bias
        else:
            bias_x ,bias_y = [0,0]
        self.file = file
        self.raw_map = Image.open(self.file).convert('RGB')
        self.map = None
        split_file = re.split('X_|Y_|.png', self.file)
        self.width = self.raw_map.width           # Width of the height map
        self.height = self.raw_map.height           # Height of the height map
        self.resolution = 0.05     # Map resolution
        self.X = float(split_file[1]) + bias_x               # X displacement of the map's origin relative to world coordinates
        self.Y = float(split_file[2]) + bias_y               # Y displacement of the map's origin relative to world coordinates
        # Actual coordinate range of the map: xmin, xmax, ymin, ymax
        self.area_range = [self.X-self.width*self.resolution, self.X+self.width*self.resolution,
                           self.Y-self.height*self.resolution, self.Y+self.height*self.resolution]


    def make_map(self):
        '''
        Convert the raw image map into a numpy array representation.
        '''
        image_array = np.array(self.raw_map)
        self.map = np.zeros((int(self.height), int(self.width)), dtype=int)
        for i in range(image_array.shape[0]):
            for j in range(image_array.shape[1]):
                # Get pixel value
                pixel = image_array[i, j]
                # Check if the pixel is black (1 represents obstacle)
                if np.array_equal(pixel, [0, 0, 0]):
                    self.map[i, j] = 1
                # Check if the pixel is white (0 represents walkable area)
                elif np.array_equal(pixel, [255, 255, 255]):
                    self.map[i, j] = 0
                else:   # (2 represents unobserved area)
                    self.map[i, j] = 2
        return self.map

    def de_noising(self):
        '''
        Remove noise from the height map by setting unobserved areas based on their surroundings.
        '''
        if self.map is None:
            print("please reconstruct the map first!")
            return 0
        noise_list = np.argwhere(self.map == 2)     # Get all unobserved points' coordinates
        for coord in noise_list:
            if coord[0]*coord[1] == 0 or coord[1] == self.height-1 or coord[0] == self.width-1:
                continue                            # Skip edge points
            else:                                   # Get surrounding values
                around = np.array([self.map[coord[0]-1][coord[1]-1], self.map[coord[0]][coord[1]-1],
                                   self.map[coord[0]+1][coord[1]-1], self.map[coord[0]-1][coord[1]],
                                   self.map[coord[0]+1][coord[1]], self.map[coord[0]-1][coord[1]+1],
                                   self.map[coord[0]][coord[1]+1], self.map[coord[0]+1][coord[1]+1]])

                if np.count_nonzero(around == 1) > 4:   # Unobserved point surrounded by obstacles
                    self.map[coord[0]][coord[1]] = 1
                elif np.count_nonzero(around == 0) > 4:    # Unobserved point surrounded by walkable areas
                    self.map[coord[0]][coord[1]] = 0
                else:
                    pass

    def map_plot(self):
        '''
        Plot the height map using matplotlib.
        '''
        if self.map is None:
            print("please reconstruct the map first!")
            return 0
        fig, ax = plt.subplots()
        im = ax.imshow(self.map, cmap='hot_r', interpolation='nearest')
        ax.set_xticks(np.arange(self.map.shape[1]))
        ax.set_yticks(np.arange(self.map.shape[0]))
        ax.set_xticklabels(np.arange(1, self.map.shape[1] + 1))
        ax.set_yticklabels(np.arange(1, self.map.shape[0] + 1))
        plt.colorbar(im)
        plt.show()

    def show_info(self):
        print("width: %f, height: %f" % (self.width, self.height))
        print("resolution: %f" % self.resolution)
        print("X: %f, Y: %f" % (self.X, self.Y))

    def save_pkl(self, k):
        if self.map is None:
            print("please reconstruct the map first!")
            return 0
        file_name = 'height_map'+str(k)+'.pkl'
        if not os.path.exists(file_name):
            open(file_name, 'w').close()
        with open(file_name, 'wb') as file:
            pickle.dump(self.map, file)

    def compute_range(self):
        '''
        Compute the actual coordinate range of the height map.

        Returns:
            List containing the minimum and maximum x and y coordinates of the map.
        '''
        min_x = self.X
        max_x = self.X + self.height*self.resolution
        min_y = self.Y
        max_y = self.Y + self.width*self.resolution
        print("min x: %f, max x: %f" % (min_x, max_x))
        print("min y: %f, max y: %f" % (min_y, max_y))

        return [min_x, max_x, min_y, max_y]

    def get_map(self):
        return self.map


