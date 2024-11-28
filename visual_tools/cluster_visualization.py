from .open3d_box import create_box
from .ransac_visualization import extract_box_params_from_lineset
import numpy as np
import open3d as o3d

def draw_box_from_cluster(vis, bounding_box_init, x_center, y_center):
    """
    Draws and updates a bounding box in the Open3D visualizer by shifting its center to x_center, y_center.
    
    Args:
        vis (open3d.visualization.Visualizer): Open3D visualizer.
        bounding_box_init (open3d.geometry.LineSet): LineSet representing the bounding box.
        x_center (float): Target x-coordinate of the new center.
        y_center (float): Target y-coordinate of the new center.

    Returns:
        open3d.geometry.LineSet: Updated LineSet for the bounding box.
    """
    # Extract the corners of the LineSet (bounding box vertices)
    corners = np.asarray(bounding_box_init.points)
    
    # Compute the current center of the bounding box
    current_center = np.mean(corners, axis=0)
    
    # Compute the translation vector to shift the box center
    translation = np.array([x_center - current_center[0], 
                            y_center - current_center[1], 
                            0])  # No change in z-axis for 2D operations
    
    # Apply the translation to all points of the LineSet
    shifted_corners = corners + translation
    
    # Update the points of the bounding box with the shifted coordinates
    bounding_box_init.points = o3d.utility.Vector3dVector(shifted_corners)
    
    # Set the color of the bounding box (yellow)
    bounding_box_init.colors = o3d.utility.Vector3dVector([[0, 0, 0] for _ in range(len(bounding_box_init.lines))])
    
    # Add the updated LineSet to the visualizer
    vis.add_geometry(bounding_box_init, reset_bounding_box=False)

    return bounding_box_init


