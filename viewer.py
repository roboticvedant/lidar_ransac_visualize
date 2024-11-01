import time
import numpy as np
import open3d as o3d
import os
from visual_tools import *  # Assuming this is your existing function
import shutil

class VisualizerSequence:
    def __init__(self, cloud_dir, label_dir, start_idx=0, debug_flag=True,generating_data= False, frame_interval=200):
        self.cloud_dir = cloud_dir
        self.label_dir = label_dir
        self.idx = start_idx
        self.debug_flag = debug_flag
        self.generating_data = generating_data
        self.frame_interval = frame_interval  # Time in milliseconds
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.zoom_factor = 1.0  # Default zoom level
        self.is_side_view = False  # Track whether the current view is side or top

    def load_bin_as_pcd(self, bin_file):
        """Load point cloud from a .bin file (x, y, z, intensity)."""
        point_cloud_data = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)
        points = point_cloud_data[:, :3]  # Extract x, y, z coordinates
        return points
    
    def extract_numbers_from_file(self, file_path):
        """Extract the bounding boxes from a .txt file, returning them as a numpy array."""
        numbers_list = []
        with open(file_path, 'r') as file:
            for line in file:
                parts = line.strip().split(',')
                numbers = list(map(float, parts))  # Convert to float, skipping label
                numbers_list.append(numbers)
        return np.array(numbers_list)

    def dataloader(self, cloud_path, boxes_path):
        """Load both the point cloud and bounding boxes."""
        cloud = self.load_bin_as_pcd(cloud_path)
        boxes = self.extract_numbers_from_file(boxes_path).reshape(-1, 4)
        return cloud, boxes

    def apply_zoom(self):
        """Apply the current zoom level to the view control."""
        ctr = self.vis.get_view_control()
        ctr.set_zoom(self.zoom_factor)

    def set_side_view(self):
        """Set the camera to a side view."""
        ctr = self.vis.get_view_control()
        lookat = [0, 0, 0]  # Adjust the point of interest based on your scene
        front = [1, 0, 0]   # Looking from the side along the X-axis
        up = [0, 0, 1]      # Z-axis as "up" direction

        ctr.set_lookat(lookat)
        ctr.set_front(front)
        ctr.set_up(up)
        ctr.set_zoom(self.zoom_factor)  # Apply the current zoom level

    def set_top_view(self):
        """Set the camera to a top-down view."""
        ctr = self.vis.get_view_control()
        
        # lookat = [0, 0, 0]  # Adjust the point of interest based on your scene
        lookat = [self.opponent_pos[0], self.opponent_pos[1], self.opponent_pos[2]] 
        front = [0, 1, 0.25]  
        up = [0, 0, 1]    

        ctr.set_lookat(lookat)
        ctr.set_front(front)
        ctr.set_up(up)
        ctr.set_zoom(self.zoom_factor)  # Apply the current zoom level

    def toggle_view(self):
        """Toggle between side and top view."""
        if self.is_side_view:
            self.set_top_view()
        else:
            self.set_side_view()
        self.is_side_view = not self.is_side_view  # Toggle the flag

    def next_scene(self):
        """Load and display the next frame."""
        self.vis.clear_geometries()

        cloud_file = f'{self.cloud_dir}/{self.idx:06d}.bin'
        label_file = f'{self.label_dir}/{self.idx:06d}.txt'

        if not (os.path.exists(cloud_file) and os.path.exists(label_file)):
            print(f"No more files found at index {self.idx:06d}")
            return False

        print(f"Processing: {cloud_file}")
        cloud, boxes = self.dataloader(cloud_file, label_file)

        self.opponent_pos = boxes[0][:3]

        if self.generating_data:
            # Draw the point cloud and bounding boxes
            draw_clouds_with_boxes(self.vis, cloud, boxes)
            self.fixedOpponentBox= draw_ransac_road(self.vis, cloud, boxes, expansion_ratio=1, distance_threshold=0.1, ransac_n=3, num_iterations=1000)
        else:
            self.ShowCorrectData(self.vis, self.idx)

        # Apply the zoom level and persist the current view (side or top)
        self.apply_zoom()

        if self.is_side_view:
            self.set_side_view()
        else:
            self.set_top_view()

        if self.debug_flag:
            print(f"Debugging: Full animation and RANSAC applied for frame {self.idx}")
        else:
            print(f"Frame {self.idx}: Moving without running full animation")

        self.idx += 1
        self.vis.poll_events()
        self.vis.update_renderer()
        return True

    def key_callback(self, vis):
        """Key callback to handle 'N' key presses for the next frame."""
        return self.next_scene()

    def auto_update_callback(self, vis):
        """Automatically update the frames at the specified interval."""
        if self.next_scene():
            time.sleep(self.frame_interval / 1000.0)  # Convert milliseconds to seconds
        return True

    def zoom_in(self, vis):
        """Zoom into the scene and store the zoom level."""
        ctr = self.vis.get_view_control()
        current_zoom = self.zoom_factor  # Get the current zoom level
        new_zoom = current_zoom * 0.9  # Zoom in by reducing the zoom value
        ctr.set_zoom(new_zoom)  # Set the new zoom level
        self.zoom_factor = new_zoom  # Store the updated zoom factor
        return True

    def zoom_out(self, vis):
        """Zoom out of the scene and store the zoom level."""
        ctr = self.vis.get_view_control()
        current_zoom = self.zoom_factor  # Get the current zoom level
        new_zoom = current_zoom * 1.1  # Zoom out by increasing the zoom value
        ctr.set_zoom(new_zoom)  # Set the new zoom level
        self.zoom_factor = new_zoom  # Store the updated zoom factor
        return True
    
    def SaveCorrectedBox(self):
        # Save the corrected box parameters as a text file
        save_path_txt = os.path.join("CorrectedData", "labels", f'{self.idx - 1:06d}.txt')
        exportCorrectedBox(self.fixedOpponentBox, save_path_txt)
        
        # Copy the original .bin file to the corrected data directory
        original_bin_file = os.path.join(self.cloud_dir, f'{self.idx - 1:06d}.bin')
        corrected_bin_file = os.path.join("CorrectedData", "cloud", f'{self.idx - 1:06d}.bin')
        os.makedirs(os.path.dirname(corrected_bin_file), exist_ok=True)

        if os.path.exists(original_bin_file):
            shutil.copyfile(original_bin_file, corrected_bin_file)
            print(f"Copied {original_bin_file} to {corrected_bin_file}")
        else:
            print(f"Original .bin file {original_bin_file} not found")
            return False

        return True

    def load_boxes_from_corrected_data(self, corrected_txt_file):
        """Load box parameters from a .txt file in CorrectedData."""
        if not os.path.exists(corrected_txt_file):
            print(f"File {corrected_txt_file} not found")
            return None

        boxes = []
        with open(corrected_txt_file, 'r') as file:
            for line in file:
                parts = list(map(float, line.strip().split()))
                boxes.append(parts)  # Append box in format [x, y, z, dx, dy, dz, yaw]
        return np.array(boxes)

    def ShowCorrectData(self, vis, idx):
        """Visualize point cloud and boxes from CorrectedData."""
        visualize_fixed_data(vis, idx)
        return True

    def run(self):
        """Set up the visualizer and start the sequence."""
        self.vis.create_window()
        self.vis.register_key_callback(ord('N'), self.key_callback)

        # Register additional key callbacks for zoom and toggling view
        self.vis.register_key_callback(ord('I'), self.zoom_in)
        self.vis.register_key_callback(ord('O'), self.zoom_out)
        self.vis.register_key_callback(ord('V'), lambda vis: self.toggle_view())  # Toggle view on 'V'
        self.vis.register_key_callback(ord('S'), lambda vis: self.SaveCorrectedBox())  # Save corrected box on 'S'

        if not self.debug_flag:
            # Register automatic frame progression if debug_flag is False
            self.vis.register_animation_callback(self.auto_update_callback)

        # Start the first frame
        self.next_scene()

        # Start the Open3D visualizer event loop
        self.vis.run()
        self.vis.destroy_window()

if __name__ == "__main__":
    # Directory paths to cloud and label files
    cloud_directory = 'RACECAR_DATA/data/cloud'
    label_directory = 'RACECAR_DATA/data/labels'

    # Create the visualizer sequence object and run it with automatic frame progression
    visualizer = VisualizerSequence(cloud_directory, label_directory, start_idx=0, debug_flag=True, frame_interval=10)
    visualizer.run()
