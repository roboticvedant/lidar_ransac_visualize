import numpy as np
import open3d as o3d


def rotz(t):
    """ Rotation about the z-axis. """
    c = np.cos(t)
    s = np.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def box2corners(box, length_of_car=5.0, width_of_car=2.0, height_of_car=2.0, rear_axle_from_center=2.0, height_of_rear_axle=0.5):
    """
    Converts car box parameters [x, y, z, yaw] to 3D corners with given car dimensions,
    where (x, y, z) is the position of the rear axle.

    Parameters:
        box (list): Box parameters [x_rear_axle, y_rear_axle, z_rear_axle, yaw].
        length_of_car (float): Length of the car (dx).
        width_of_car (float): Width of the car (dy).
        height_of_car (float): Height of the car (dz).
        rear_axle_from_center (float): Distance from the center to the rear axle.
        height_of_rear_axle (float): Height offset of the rear axle from the base.

    Returns:
        np.ndarray: 3D coordinates of the box corners, shape (8, 3).
    """
    # Extract box parameters
    x_rear_axle, y_rear_axle, z_rear_axle = box[:3]
    yaw = box[3]

    l = length_of_car  # Length (dx)
    w = width_of_car   # Width (dy)
    h = height_of_car  # Height (dz)

    # Calculate the center of the car based on the rear axle position and yaw angle
    x_center = x_rear_axle + rear_axle_from_center * np.cos(yaw)
    y_center = y_rear_axle + rear_axle_from_center * np.sin(yaw)
    z_center = z_rear_axle + height_of_rear_axle

    # Create rotation matrix for the yaw angle around the z-axis
    R = rotz(yaw)

    # Define the local 3D corners centered at the origin (before rotation and translation)
    Box = np.array([
        [-l / 2, -w / 2, 0],  # Rear-left bottom
        [l / 2, -w / 2, 0],   # Rear-right bottom
        [l / 2, w / 2, 0],    # Front-right bottom
        [-l / 2, w / 2, 0],   # Front-left bottom
        [-l / 2, -w / 2, h],  # Rear-left top
        [l / 2, -w / 2, h],   # Rear-right top
        [l / 2, w / 2, h],    # Front-right top
        [-l / 2, w / 2, h]    # Front-left top
    ])

    # Rotate and translate the corners to the correct position
    rotated_corners = np.dot(Box, R.T)
    rotated_corners[:, 0] += x_center  # Translate x
    rotated_corners[:, 1] += y_center  # Translate y
    rotated_corners[:, 2] += z_center  # Translate z

    return rotated_corners


def create_box_from_corners(corners, color=None):
    """
	corners: 8 corners(x, y, z)
	corners: array = 8*3
	#         7 -------- 6
	#        /|         /|
	#       4 -------- 5 .
	#       | |        | |
	#       . 3 -------- 2
	#       |/         |/
	#       0 -------- 1
	"""
    # 12 lines in a box
    lines = [
        [0, 1],
        [1, 2],
        [2, 3],
        [3, 0],
        [4, 5],
        [5, 6],
        [6, 7],
        [7, 4],
        [0, 4],
        [1, 5],
        [2, 6],
        [3, 7],
    ]

    colors = [color for i in range(len(lines))]
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


def create_box(box, color=None, length_of_car=5.0, width_of_car=2.0, height_of_car=2.0, rear_axle_from_center=2.0, height_of_rear_axle=0.5):
    """
    box: list(8) [ x, y, z, dx, dy, dz, yaw]
    """
    box_corners = box2corners(box, length_of_car, width_of_car, height_of_car, rear_axle_from_center, height_of_rear_axle)
    box = create_box_from_corners(box_corners, color)
    return box