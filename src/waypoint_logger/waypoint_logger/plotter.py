#!/usr/bin/env python3

'''
Displays the map of waypoints and additional data from another CSV
'''

# Imports
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
import csv
import os
import rospkg
from rospkg import RosPack
from nav_msgs.msg import Odometry
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA
import matplotlib.pyplot as plt
from matplotlib import patches

# GLOBAL VARIABLES 
xc = 0
yc = 0
yaw = 0 
idx = 0
waypoints = []

def read_points(file_name):
    # CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
    file_path = file_name
    with open(file_path) as f:
        path_points = np.loadtxt(file_path, delimiter=',')
    return path_points

if __name__=='__main__':
    # Read waypoints from the first CSV file
    waypoints = read_points('wp_file.csv')  # Update this to your first CSV file path
    # Read additional data from the second CSV file
    additional_data = read_points('wp_file_stanly.csv')  # Update this to your second CSV file path

    plt.cla()
    # PURE PURSUIT CODE 
    cx = []; cy = []
    for point in waypoints:
        cx.append(float(point[0]))
        cy.append(float(point[1]))
    
    plt.plot(cx, cy, "-r", label="Waypoints")

    # Plotting the additional data
    additional_x = []; additional_y = []
    for point in additional_data:
        additional_x.append(float(point[0]))
        additional_y.append(float(point[1]))

    plt.plot(additional_x, additional_y, "-b", label="Stanely")  # Change color and label as needed

    plt.axis("equal")
    plt.grid(True)
    plt.title("Path Tracking Survey")
    plt.legend()  # Add legend to differentiate between plots
    plt.show()
