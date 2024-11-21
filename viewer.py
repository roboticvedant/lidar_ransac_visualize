import time
import numpy as np
import open3d as o3d
import os
from visual_tools import *  # Assuming this is your existing function
import shutil
import argparse

class VisualizerSequence:
    def __init__(self, raw_data_dir, fixed_data_dir, start_idx, play_animation, generating_data, 
                ransac_visualize_flag, frame_interval, length_of_car, width_of_car,
                height_of_car, axle_from_center, height_of_rear_axle):

        self.fixed_data_dir = fixed_data_dir
        self.raw_data_dir = raw_data_dir

        if generating_data:
            self.cloud_dir = os.path.join(raw_data_dir, "cloud")
            self.label_dir = os.path.join(raw_data_dir, "labels")
        else:
            self.cloud_dir = os.path.join(fixed_data_dir, "cloud")
            self.label_dir = os.path.join(fixed_data_dir, "labels")

        self.ransac_visualize_flag = ransac_visualize_flag
        self.idx = start_idx
        self.play_animation = play_animation # Flag to play the animation
        self.generating_data = generating_data
        self.frame_interval = frame_interval  # Time in milliseconds
        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self.zoom_factor = 1.0  # Default zoom level
        self.is_side_view = False  # Track whether the current view is side or top

        self.height_of_car = height_of_car
        self.width_of_car = width_of_car
        self.length_of_car = length_of_car
        self.axle_from_center = axle_from_center # along the length of the car from the center
        self.height_of_rear_axle = height_of_rear_axle

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
        # lookat = [0, 0, 0]  # Adjust the point of interest based on your scene
        lookat = [self.opponent_pos[0], self.opponent_pos[1], self.opponent_pos[2]] 
        front = [1, -1.0, -0.025]   # Looking from the side along the X-axis
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
            if not self.generating_data:
                    try:
                        self.idx += 1
                        print(f"Skipping to next index {self.idx:06d}")
                        if self.idx > 99999: # arbitrary limit to prevent issue with infinite loop
                            print("End of data reached")
                            self.vis.destroy_window()
                            return True
                        return self.next_scene()
                    except:
                        print("End of data reached, caught by exception") # end if it gets in infinite loop, not ideal but good enoguh for now
                        self.vis.destroy_window()
                        return True
            else:
                print(f"Exiting at index {self.idx:06d}")
                self.vis.destroy_window()
                return False

        print(f"Processing: {cloud_file}")
        cloud, boxes = self.dataloader(cloud_file, label_file)

        self.opponent_pos = boxes[0][:3]

        if self.generating_data:
            # Draw the point cloud and bounding boxes
            draw_clouds_with_boxes(self.vis, cloud, boxes, self.length_of_car, self.width_of_car, 
                                    self.height_of_car, self.axle_from_center, self.height_of_rear_axle)
            if self.ransac_visualize_flag:
                try:
                    self.fixedOpponentBox= draw_ransac_road(self.vis, cloud, boxes, expansion_ratio=1, 
                                                        distance_threshold=0.1, ransac_n=3, num_iterations=1000, 
                                                        length=self.length_of_car, width=self.width_of_car, 
                                                        height=self.height_of_car,
                                                        rear_axle_from_center=self.axle_from_center, 
                                                        height_of_rear_axle=self.height_of_rear_axle)
                except:
                    print('Error in RANSAC')
                    self.idx += 1
                    return
        else:
            draw_clouds_with_boxes(self.vis, cloud, boxes)

        # Apply the zoom level and persist the current view (side or top)
        self.apply_zoom()

        if self.is_side_view:
            self.set_side_view()
        else:
            self.set_top_view()

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
        if not self.generating_data:
            print("Not generating data, skipping SaveCorrectedBox")
            return False
        if not self.ransac_visualize_flag:
            print("RANSAC visualization flag is off, skipping SaveCorrectedBox")
            return False

        # Save the corrected box parameters as a text file
        save_path_txt = os.path.join(self.fixed_data_dir,"labels", f'{self.idx - 1:06d}.txt')
        exportCorrectedBox(self.fixedOpponentBox, save_path_txt)
        
        # Copy the original .bin file to the corrected data directory
        original_bin_file = os.path.join(self.cloud_dir, f'{self.idx - 1:06d}.bin')
        corrected_bin_file = os.path.join(self.fixed_data_dir,"cloud", f'{self.idx - 1:06d}.bin')
        os.makedirs(os.path.dirname(corrected_bin_file), exist_ok=True)

        if os.path.exists(original_bin_file):
            shutil.copyfile(original_bin_file, corrected_bin_file)
            print(f"Copied {original_bin_file} to {corrected_bin_file}")
        else:
            print(f"Original .bin file {original_bin_file} not found")
            return False

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

        if self.play_animation:
            self.vis.register_animation_callback(self.auto_update_callback)

        # Start the first frame
        self.next_scene()

        # Start the Open3D visualizer event loop
        self.vis.run()
        self.vis.destroy_window()

if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description='Visualize the point cloud data with bounding boxes')

    # Paths
    argparser.add_argument('-r', '--raw_data_dir', type=str,default='RACECAR_DATA/data/', help='Path to the raw data directory')
    argparser.add_argument('-f', '--fixed_data_dir', type=str, default='RACECAR_DATA/correctedData/', help='Path to the fixed data directory')

   # Flags
    argparser.add_argument('-p', '--play_animation', action='store_true', help='Enable debug mode')
    argparser.add_argument('-g', '--generate', action='store_true', help='Generate corrected data')
    argparser.add_argument('-n', '--no_ransac', action='store_false', help='Disable RANSAC visualization')

    # Car parameters
    argparser.add_argument('-s', '--start_idx', type=int, default=290, help='Start index for visualization')
    argparser.add_argument('-i', '--frame_interval', type=int, default=200, help='Time in milliseconds between frames')
    argparser.add_argument('-l', '--length_of_car', type=float, default=5.0, help='Length of the car')
    argparser.add_argument('-w', '--width_of_car', type=float, default=2.0, help='Width of the car')
    argparser.add_argument('-hc', '--height_of_car', type=float, default=2.0, help='Height of the car')
    argparser.add_argument('-a', '--axle_from_center', type=float, default=2.0, help='Distance from center to rear axle')
    argparser.add_argument('-hr', '--height_of_rear_axle', type=float, default=0.5, help='Height of the rear axle from the ground')

    args = argparser.parse_args()

    # Create the visualizer sequence object and run it with automatic frame progression
    visualizer = VisualizerSequence(args.raw_data_dir, args.fixed_data_dir, 
                                    args.start_idx, args.play_animation, args.generate,
                                    args.no_ransac, args.frame_interval, 
                                    args.length_of_car, args.width_of_car, 
                                    args.height_of_car, args.axle_from_center, 
                                    args.height_of_rear_axle)
    visualizer.run()
