import numpy as np
import pandas as pd
from mpl_toolkits.mplot3d import Axes3D
# import octomap

import matplotlib.pyplot as plt

def read_octomap(file_path):
    return pd.read_csv(file_path).values
def read_frontier_points(file_path):
    return pd.read_csv(file_path).values

def visualize(frontier_points, occupied_points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Plot occupied points
    ax.scatter(occupied_points[:, 0], occupied_points[:, 1], occupied_points[:, 2], c='b', marker='o', s=1, label='Occupied Points')
    
    # Plot frontier points
    ax.scatter(frontier_points[:, 0], frontier_points[:, 1], frontier_points[:, 2], c='r', marker='o', s=10, label='Frontier Points')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.legend()
    plt.show()

if __name__ == "__main__":
    octomap_file = 'frontier_detector/src/frontier_detection_ros/map_resource/occupancy.csv'
    frontier_points_file = 'frontier_detector/src/frontier_detection_ros/map_resource/frontier_points.csv'
    
    occupied_points = read_octomap(octomap_file)
    frontier_points = read_frontier_points(frontier_points_file)
    
    visualize(frontier_points, occupied_points)