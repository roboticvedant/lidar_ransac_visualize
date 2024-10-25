import numpy as np
import open3d as o3d

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

def extract_border_points(cloud, box, expansion_ratio):
    """
    Extract points on the expanded border around the bounding box based on [x, y, z, yaw].
    
    Parameters:
        cloud (np.ndarray): The point cloud data.
        box (list): [x, y, z, yaw].
        expansion_ratio (float): Factor to expand beyond the car's width for the border region.
    
    Returns:
        np.ndarray: Points on the expanded bounding box border region.
    """
    x_center, y_center, z_center, yaw = box  # Unpack the box
    l = 5  # Length of the car (adjust as needed)
    w = 2  # Width of the car (adjust as needed)
    
    # Expand the width to cover the border area
    expanded_w = w * (1 + expansion_ratio)  # Expand by 0.5 times the car width
    
    # Rotate the box corners based on the yaw angle
    rotation_matrix = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])

    # Define the corners of the expanded box in local coordinates
    local_corners = np.array([
        [-l/2, -expanded_w/2],
        [ l/2, -expanded_w/2],
        [ l/2,  expanded_w/2],
        [-l/2,  expanded_w/2]
    ])

    # Rotate the corners around the center using the yaw angle
    rotated_corners = np.dot(local_corners, rotation_matrix.T)
    rotated_corners[:, 0] += x_center
    rotated_corners[:, 1] += y_center

    # Extract points within the expanded bounding box region in XY space
    x_min, y_min = np.min(rotated_corners, axis=0)
    x_max, y_max = np.max(rotated_corners, axis=0)

    points_in_border_patch = cloud[(cloud[:, 0] >= x_min) & (cloud[:, 0] <= x_max) &
                                   (cloud[:, 1] >= y_min) & (cloud[:, 1] <= y_max)]
    
    return points_in_border_patch

def create_ransac_plane_visualization(points_in_patch, plane_model):
    """
    Create a solid plane based on the RANSAC plane model.
    
    Parameters:
        points_in_patch (np.ndarray): Points that were used for RANSAC.
        plane_model (List[float]): Plane equation coefficients [a, b, c, d].

    Returns:
        o3d.geometry.TriangleMesh: A solid plane mesh.
    """
    # Unpack the plane model coefficients
    a, b, c, d = plane_model

    # Use the points' XY bounds to create a solid plane
    x_min, y_min = np.min(points_in_patch[:, :2], axis=0)
    x_max, y_max = np.max(points_in_patch[:, :2], axis=0)

    # Calculate the Z-values of the corners based on the plane equation: z = -(a*x + b*y + d)/c
    z1 = -(a * x_min + b * y_min + d) / c
    z2 = -(a * x_max + b * y_min + d) / c
    z3 = -(a * x_max + b * y_max + d) / c
    z4 = -(a * x_min + b * y_max + d) / c

    # Create the four corners of the plane
    corners = np.array([[x_min, y_min, z1],
                        [x_max, y_min, z2],
                        [x_max, y_max, z3],
                        [x_min, y_max, z4]])

    # Create a mesh from the 4 corners
    plane_mesh = o3d.geometry.TriangleMesh()
    plane_mesh.vertices = o3d.utility.Vector3dVector(corners)
    plane_mesh.triangles = o3d.utility.Vector3iVector([[0, 1, 2], [0, 2, 3]])  # Two triangles forming a rectangle
    # Set the color to solid green
    plane_mesh.paint_uniform_color([0.0, 1.0, 0.0])  # Green patch representing the road

    return plane_mesh

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
        points_in_border_patch = extract_border_points(cloud, box, expansion_ratio)
        if points_in_border_patch.shape[0] > 0:  # Ensure there are points in the patch
            # Fit a plane to the points using RANSAC
            inliers, plane_model = fit_plane_with_ransac(points_in_border_patch, distance_threshold, ransac_n, num_iterations)
            # Create the solid RANSAC plane
            ransac_plane = create_ransac_plane_visualization(points_in_border_patch, plane_model)
            # Add the RANSAC plane to the visualization
            vis.add_geometry(ransac_plane)
