# VisualizerSequence Tool

This tool visualizes point cloud data and bounding boxes from `.bin` and `.txt` files using Open3D. It supports zoom controls, frame-by-frame navigation, and automatic frame progression.

## How to Run

1. **Directory Setup:**
   - Ensure your point cloud files (`.bin`) and bounding box files (`.txt`) are stored in separate directories.
   - Example directory structure:
     ```
     RACECAR_DATA/
       └── data/
           ├── cloud/    # Contains .bin point cloud files
           └── labels/   # Contains .txt label files
     ```

2. **Run the Script:**
   - Use the following command to run the visualizer:
     ```bash
     python viewer.py
     ```

3. **Parameters:**
   - `cloud_directory`: Path to the directory containing point cloud (`.bin`) files.
   - `label_directory`: Path to the directory containing bounding box (`.txt`) files.
   - `start_idx`: The starting index of the frames to load (e.g., `000230.bin` and `000230.txt`).
   - `debug_flag`: Set `True` for debugging mode, which enables detailed logging of frames and the RANSAC plane fit process.
   - `frame_interval`: Time interval between frames in milliseconds when automatic frame progression is enabled.

4. **Example:**
   - To visualize frames starting from frame `230` with automatic progression every 10 milliseconds:
     ```bash
     python viewer.py
     ```

## Key Commands

- **N**: Load the next frame manually.
- **I**: Zoom into the scene.
- **O**: Zoom out of the scene.
- **Q**: To quit the scene.

## Flags

- **debug_flag (bool)**: 
  - Set to `True` to enable full debugging information and disable automatic frame progression.
  - Default: `False`.
  
- **frame_interval (int)**:
  - Time (in milliseconds) between each frame when automatic progression is enabled.
  - Default: `200` milliseconds.

## Features

- **Manual Frame Navigation**: Press `N` to manually load the next point cloud and bounding box.
- **Automatic Frame Progression**: When `debug_flag=False`, frames advance automatically according to the set `frame_interval`.
- **Zoom Controls**: 
  - Press `I` to zoom into the scene.
  - Press `O` to zoom out of the scene.

## Example Usage

```python
# Create the visualizer sequence object and run it with automatic frame progression
visualizer = VisualizerSequence(
    cloud_dir="RACECAR_DATA/data/cloud", 
    label_dir="RACECAR_DATA/data/labels", 
    start_idx=230, 
    debug_flag=False, 
    frame_interval=10
)
visualizer.run()
```
