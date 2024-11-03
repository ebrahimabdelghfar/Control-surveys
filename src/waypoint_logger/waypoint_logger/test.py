import numpy as np
def read_points():
    """
    CHANGE THIS PATH TO WHERE YOU HAVE SAVED YOUR CSV FILES
    """
    file_name = 'wp_file.csv' #'racecar_walker.csv'
    file_path = file_name
    with open(file_path) as f:
        path_points = np.loadtxt(file_path, delimiter = ',')
    return path_points



waypoints = read_points()

waypoints_xy = waypoints[:, :2]
print(waypoints_xy)