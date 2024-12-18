import open3d as o3d
import numpy as np


from .open3d_coordinate import create_coordinate
from .open3d_arrow import create_arrow
from .open3d_box import create_box
from .ransac_visualization import *
from .cluster_visualization import *

def create_bounding_box_line_set(centroid, yaw, dx=2.0, dy=1.5, dz=1.0):
    """
    Create a line set for a bounding box from position (x,y,z), yaw angle, and dimensions.
    """
    # Define the 8 vertices of the box relative to center (before rotation)

    l, w, h = dx/2, dy/2, dz/2
    vertices_base = np.array([
        [ l,  w,  h], # 0
        [-l,  w,  h], # 1
        [-l, -w,  h], # 2
        [ l, -w,  h], # 3
        [ l,  w, -h], # 4
        [-l,  w, -h], # 5
        [-l, -w, -h], # 6
        [ l, -w, -h]  # 7
    ])
    
    # Create rotation matrix around z-axis
    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    
    # Rotate vertices
    vertices_rotated = np.dot(vertices_base, rotation_matrix.T)
    
    # Translate vertices to final position
    vertices = vertices_rotated + np.array(centroid)
    
    # Define the 12 edges of the box using vertex indices
    lines = np.array([
        [0, 1], [1, 2], [2, 3], [3, 0],  # Top face
        [4, 5], [5, 6], [6, 7], [7, 4],  # Bottom face
        [0, 4], [1, 5], [2, 6], [3, 7]   # Connecting edges
    ])
    
    # Create LineSet
    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(vertices)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    
    # Set color for all lines (green for the new box)
    colors = np.array([[0, 1, 0] for _ in range(len(lines))])
    line_set.colors = o3d.utility.Vector3dVector(colors)
    
    return line_set

def extract_yaw_from_line_set(line_set):
    """
    Extract the yaw angle from a line set by analyzing the orientation of the box.
    """
    vertices = np.asarray(line_set.points)
    # Get front face vertices (assuming first four points are the top face)
    front_edge = vertices[1] - vertices[0]  # Vector along front edge
    yaw = np.arctan2(front_edge[1], front_edge[0])
    return yaw

def cluster_visualization(vis, boxes, x_center, y_center):
    return draw_box_from_cluster(vis, boxes, x_center, y_center)


def create_box_with_arrow(box, length_of_car, width_of_car, height_of_car, rear_axle_from_center, height_of_rear_axle, color=None,):
    """
    box: list(8) [ x, y, z, dx, dy, dz, yaw]
    """

    box_o3d = create_box(box, color, length_of_car, width_of_car, height_of_car, rear_axle_from_center, height_of_rear_axle)
    x = box[0]
    y = box[1]
    z = box[2]
    print('Value of z', z)
    l = length_of_car
    h = height_of_car
    yaw = box[3]
    # get direction arrow
    dir_x = l * np.cos(yaw)
    dir_y = l * np.sin(yaw)

    arrow_origin = [x, y , h/2]
    arrow_end = [x + dir_x, y + dir_y, h/2]
    arrow = create_arrow(arrow_origin, arrow_end, color)

    return box_o3d, arrow


# def draw_clouds_with_boxes(cloud , boxes):
#     """
#     cloud: (N, 4)  [x, y, z, intensity]
#     boxes: (n,7) np.array = n*7  ( x, y, z, dx, dy, dz, yaw) 
#     """
#     vis = o3d.visualization.Visualizer()
#     vis.create_window()
#     # --------------------------------------------------------------
#     # create point cloud
#     # --------------------------------------------------------------
#     points_color = [[0.5, 0.5, 0.5]]  * cloud.shape[0]
#     pc = o3d.geometry.PointCloud()
#     pc.points = o3d.utility.Vector3dVector(cloud[:,:3])
#     pc.colors = o3d.utility.Vector3dVector(points_color)
#     vis.add_geometry(pc)

#     # --------------------------------------------------------------
#     # create boxes with colors with arrow
#     # --------------------------------------------------------------
#     boxes_o3d = []

#     cur_box_color = [1, 0, 0]

#     # create boxes
#     for box in boxes:
#         box_o3d, arrow = create_box_with_arrow(box, cur_box_color)
#         boxes_o3d.append(box_o3d)
#         boxes_o3d.append(arrow)
#     # add_geometry fro boxes
#     [vis.add_geometry(element) for element in boxes_o3d]

#     # --------------------------------------------------------------
#     # coordinate frame
#     # --------------------------------------------------------------
#     coordinate_frame = create_coordinate(size=2.0, origin=[0, 0, 0])
#     vis.add_geometry(coordinate_frame)

#     # --------------------------------------------------------------
#     # drop the window
#     # --------------------------------------------------------------
#     vis.get_render_option().point_size = 2
#     vis.run()
#     vis.destroy_window()


import open3d as o3d
import numpy as np

def draw_clouds_with_boxes(vis, cloud, boxes, length=5, width=2, height=1.5, rear_axle_from_center=2, height_of_rear_axle=0.5):
    """
    vis: open3d.visualization.Visualizer
    cloud: (N, 4)  [x, y, z, intensity]
    boxes: (n, 7) np.array = n*7  ( x, y, z, dx, dy, dz, yaw) 
    """
    # --------------------------------------------------------------
    # create point cloud
    # --------------------------------------------------------------
    points_color = [[0.5, 0.5, 0.5]] * cloud.shape[0]
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(cloud[:, :3])
    pc.colors = o3d.utility.Vector3dVector(points_color)
    vis.add_geometry(pc)

    # --------------------------------------------------------------
    # create boxes with colors with arrow
    # --------------------------------------------------------------
    boxes_o3d = []

    cur_box_color = [1, 0, 0]

    # create boxes
    for box in boxes:
        box_o3d, arrow = create_box_with_arrow(box, length, width, height, rear_axle_from_center, height_of_rear_axle, cur_box_color)
        boxes_o3d.append(box_o3d)
        boxes_o3d.append(arrow)
    
    # add_geometry for boxes
    [vis.add_geometry(element) for element in boxes_o3d]

    # --------------------------------------------------------------
    # coordinate frame
    # --------------------------------------------------------------
    coordinate_frame = create_coordinate(size=2.0, origin=[0, 0, 0])
    vis.add_geometry(coordinate_frame)

    # --------------------------------------------------------------
    # create and add grid
    # --------------------------------------------------------------
    # grid = create_grid(size=30, step=2.5)
    # vis.add_geometry(grid)

    # Set point size
    vis.get_render_option().point_size = 2

    # Set camera view for top-down view
    # ctr = vis.get_view_control()
    # ctr.set_lookat([0, 0, 0])  # Look at the origin
    # ctr.set_front([0, 0, -1])   # Look down along the negative z-axis
    # ctr.set_up([0, 1, 0])       # Y-axis as up
    # ctr.set_zoom(0.5)           # Adjust zoom level

    # Run the visualization
    

    # Save the screenshot
    # vis.capture_screen_image("pointcloud_top_view.png")  # Save the image as a PNG file

    # Clean up and close the window

def draw_fixed_boxes(vis, boxes, plane_model):
    
    boxes_o3d = []
    cur_box_color = [0, 1, 1]

    # create boxes
    for box in boxes:
        box_o3d, arrow = create_box_with_arrow(box, cur_box_color)
        boxes_o3d.append(box_o3d)
        boxes_o3d.append(arrow)
    
    # add_geometry for boxes
    [vis.add_geometry(element) for element in boxes_o3d]

def create_grid(size=10, step=1.0):
    """
    Create a grid in the XY plane (horizontal grid).
    size: the overall grid span (will be -size to +size)
    step: spacing between the grid lines
    """
    lines = []
    points = []

    # Create grid points along X and Y axes
    for i in np.arange(-size, size + step, step):
        # Along X axis
        points.append([i, -size, 0])  # Y-axis changes, X stays constant
        points.append([i, size, 0])
        # Along Y axis
        points.append([-size, i, 0])  # X-axis changes, Y stays constant
        points.append([size, i, 0])

    # Create line connections
    for i in range(0, len(points), 2):
        lines.append([i, i + 1])

    # Convert points and lines to Open3D LineSet
    points = o3d.utility.Vector3dVector(points)
    lines = o3d.utility.Vector2iVector(lines)

    line_set = o3d.geometry.LineSet(points=points, lines=lines)

    # Optional: Set grid line color (e.g., gray)
    colors = [[0.1, 0.5, 0.1]] * len(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)

    return line_set


# Example function to create a coordinate frame (you can adjust this if needed)
def create_coordinate(size=1.0, origin=[0, 0, 0]):
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=origin)


def draw_ransac_road(vis, cloud, boxes, expansion_ratio, distance_threshold, ransac_n, num_iterations, length, width, height, rear_axle_from_center, height_of_rear_axle):
    try:
         _,opponent_fixed_box = visualize_with_ransac_plane(vis, cloud, boxes,expansion_ratio, distance_threshold, ransac_n, num_iterations, length, width, height, rear_axle_from_center, height_of_rear_axle)
    except:
        print('Error in RANSAC')
        return None
    return opponent_fixed_box

def visualize_fixed_data(vis, idx):
    visualize_corrected_data(vis, idx);
    draw_clouds_with_boxes(vis,cloud, boxes)
    