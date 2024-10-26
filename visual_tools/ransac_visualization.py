import numpy as np
import open3d as o3d
from .open3d_box import create_box_from_corners

def fit_plane_with_ransac(points, distance_threshold, ransac_n, num_iterations):
    """
    Fit a plane to the point cloud using RANSAC and return the plane model and inlier points.
    
    Parameters:
        points (np.ndarray): The point cloud data.
        distance_threshold (float): Maximum distance from the plane for a point to be considered an inlier.
        ransac_n (int): Number of points to sample to estimate a plane.
        num_iterations (int): Number of iterations for RANSAC.

    Returns:
        o3d.geometry.PointCloud: The points that lie on the plane (inliers).
        List[float]: The plane model [a, b, c, d] for the equation ax + by + cz + d = 0.
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Apply RANSAC to find the best-fitting plane in the point cloud
    plane_model, inliers = pcd.segment_plane(distance_threshold=distance_threshold,
                                             ransac_n=ransac_n,
                                             num_iterations=num_iterations)
    inlier_points = pcd.select_by_index(inliers)
    return inlier_points, plane_model

def extract_border_points(cloud, box, expansion_ratio, length=5, width=2, height=2, axle_from_center=2):
    """
    Extract points on the expanded border around the bounding box based on [x, y, z, yaw].
    
    Parameters:
        cloud (np.ndarray): The point cloud data.
        box (list): [x, y, z, yaw].
        expansion_ratio (float): Factor to expand beyond the car's width for the border region.
        length (float): Length of the car.
        width (float): Width of the car.
        height (float): Height of the bounding box.
        axle_from_center (float): Distance from center to the rear axle.
    
    Returns:
        tuple: 
            np.ndarray: Points within the expanded bounding box.
            np.ndarray: Points within the original bounding box (expansion_ratio = 0).
            np.ndarray: Rotated corners of the expanded bounding box.
            np.ndarray: Rotated corners of the original bounding box (expansion_ratio = 0).
    """
    # Unpack the box, treating x_rear_axle, y_rear_axle, z_rear_axle as the rear axle center coordinates
    x_rear_axle, y_rear_axle, _, yaw = box

    # Calculate the actual center of the bounding box by shifting forward by axle_from_center
    # The rear axle is behind the center, so subtract axle_from_center
    x_center = x_rear_axle + axle_from_center * np.cos(yaw)
    y_center = y_rear_axle + axle_from_center * np.sin(yaw)

    # Rotation matrix to account for yaw (rotation around the z-axis)
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])

    # Define the corners of the original box (expansion_ratio = 0) in local coordinates (XY plane)
    local_corners_original = np.array([
        [-length/2, -width/2],  # Rear-left corner
        [ length/2, -width/2],  # Front-left corner
        [ length/2,  width/2],  # Front-right corner
        [-length/2,  width/2]   # Rear-right corner
    ])
    
    # Rotate the corners around the center using the yaw angle
    rotated_corners_original = np.dot(local_corners_original, rotation_matrix.T)
    rotated_corners_original[:, 0] += x_center
    rotated_corners_original[:, 1] += y_center

    # Define the corners of the expanded box in local coordinates (XY plane)
    expanded_w = width * (1 + expansion_ratio)  # Expand by expansion_ratio
    expanded_l = length * (1 + expansion_ratio)  # Expand by expansion_ratio
    local_corners_expanded = np.array([
        [-expanded_l/2, -expanded_w/2],  # Rear-left corner
        [ expanded_l/2, -expanded_w/2],  # Front-left corner
        [ expanded_l/2,  expanded_w/2],  # Front-right corner
        [-expanded_l/2,  expanded_w/2]   # Rear-right corner
    ])
    
    # Rotate the corners around the center using the yaw angle for expanded box
    rotated_corners_expanded = np.dot(local_corners_expanded, rotation_matrix.T)
    rotated_corners_expanded[:, 0] += x_center
    rotated_corners_expanded[:, 1] += y_center

    # Extract points within the original bounding box region (expansion_ratio = 0)
    x_min_orig, y_min_orig = np.min(rotated_corners_original, axis=0)
    x_max_orig, y_max_orig = np.max(rotated_corners_original, axis=0)

    points_in_original_box = cloud[(cloud[:, 0] >= x_min_orig) & (cloud[:, 0] <= x_max_orig) &
                                   (cloud[:, 1] >= y_min_orig) & (cloud[:, 1] <= y_max_orig)]

    # Extract points within the expanded bounding box region in XY space
    x_min_exp, y_min_exp = np.min(rotated_corners_expanded, axis=0)
    x_max_exp, y_max_exp = np.max(rotated_corners_expanded, axis=0)

    points_in_expanded_box = cloud[(cloud[:, 0] >= x_min_exp) & (cloud[:, 0] <= x_max_exp) &
                                   (cloud[:, 1] >= y_min_exp) & (cloud[:, 1] <= y_max_exp)]
    
    # Return both the original and expanded box points along with their corners
    return points_in_expanded_box, rotated_corners_original


def create_ransac_plane_visualization(rotated_corners_original, plane_model):
    """
    Create a solid plane based on the RANSAC plane model using the rotated corners from extract_border_points.

    Parameters:
        rotated_corners_original (np.ndarray): Rotated corners of the original bounding box (4x2 array).
        plane_model (List[float]): Plane equation coefficients [a, b, c, d] for the equation ax + by + cz + d = 0.

    Returns:
        o3d.geometry.TriangleMesh: A solid plane mesh based on the bounding box's rotated corners.
    """
    # Unpack the plane model coefficients: ax + by + cz + d = 0
    a, b, c, d = plane_model

    # Extract the X, Y coordinates from rotated_corners_original
    rear_left_x, rear_left_y = rotated_corners_original[0]
    rear_right_x, rear_right_y = rotated_corners_original[1]
    front_right_x, front_right_y = rotated_corners_original[2]
    front_left_x, front_left_y = rotated_corners_original[3]

    # Calculate Z values for each corner using the plane equation: z = -(a * x + b * y + d) / c
    z_rear_left = -(a * rear_left_x + b * rear_left_y + d) / c
    z_rear_right = -(a * rear_right_x + b * rear_right_y + d) / c
    z_front_right = -(a * front_right_x + b * front_right_y + d) / c
    z_front_left = -(a * front_left_x + b * front_left_y + d) / c

    # Create the 3D corners of the plane
    corners = np.array([
        [rear_left_x, rear_left_y, z_rear_left],   # Rear left
        [rear_right_x, rear_right_y, z_rear_right], # Rear right
        [front_right_x, front_right_y, z_front_right], # Front right
        [front_left_x, front_left_y, z_front_left]  # Front left
    ])

    # Create a mesh from the 4 corners
    plane_mesh = o3d.geometry.TriangleMesh()
    plane_mesh.vertices = o3d.utility.Vector3dVector(corners)
    plane_mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2], [0, 2, 3]])  # Two triangles forming a rectangle

    # Set the color to solid green for visualization
    plane_mesh.paint_uniform_color([0.0, 1.0, 0.0])  # Green color to represent the plane
    box = create_full_box_from_plane(corners)
    
    return plane_mesh, box

def create_full_box_from_plane(plane_corners, height=1.5, color=[0, 1, 1]):

    top_plane_corners = plane_corners.copy()
    top_plane_corners[:, 2] += height  # Increase Z values by the box height

    full_box_corners = np.vstack([plane_corners, top_plane_corners])

    return create_box_from_corners(full_box_corners, color)

def visualize_with_ransac_plane(vis, cloud, boxes, expansion_ratio, distance_threshold, ransac_n, num_iterations):
    """
    Visualize the point cloud with a RANSAC-fitted solid plane on the expanded bounding box border.
    
    Parameters:
        cloud (np.ndarray): The point cloud data.
        boxes (np.ndarray): The bounding boxes data [x, y, z, yaw].
    """
    # Process each bounding box
    for box in boxes:
        # Extract points in the expanded bounding box border patch
        points_in_border_patch, rotated_corners_original = extract_border_points(cloud, box, expansion_ratio)
        if points_in_border_patch.shape[0] > 0:  # Ensure there are points in the patch
            # Fit a plane to the points using RANSAC
            inliers, plane_model = fit_plane_with_ransac(points_in_border_patch, distance_threshold, ransac_n, num_iterations)
            # Create the solid RANSAC plane
            ransac_plane, box = create_ransac_plane_visualization(rotated_corners_original, plane_model)
            # Add the RANSAC plane to the visualization
            vis.add_geometry(ransac_plane)
            vis.add_geometry(box)
            return plane_model
