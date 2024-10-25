# VisualizerSequence Tool

This tool visualizes point cloud data and bounding boxes from `.bin` and `.txt` files using Open3D. It supports zoom controls, frame-by-frame navigation, and automatic frame progression.

## Prerequisites

Ensure that you have **Conda** installed on your machine. You can install [Miniconda](https://docs.conda.io/en/latest/miniconda.html) if you do not have Conda.

## Setting Up the Environment

To ensure you have the correct dependencies and packages, you can recreate the conda environment by following these steps:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/roboticvedant/lidar_ransac_visualize
   cd lidar_ransac_visualize
   ```

2. **Create the Conda Environment**:
   - The environment is defined in the `environment.yml` file. You can create the environment using:
     ```bash
     conda env create -f environment.yml
     ```

3. **Activate the Environment**:
   ```bash
   conda activate racecarenv
   ```

4. **Verify Installation**:
   - You can verify the environment by checking the installed packages:
     ```bash
     conda list
     ```

## Running the Visualizer

1. **Prepare Data**: 
   - Ensure your point cloud files (`.bin`) and bounding box files (`.txt`) are stored in separate directories.
   - Example directory structure:
     ```
     RACECAR_DATA/
       └── data/
           ├── cloud/    # Contains .bin point cloud files
           └── labels/   # Contains .txt label files
     ```

2. **Run the Script**:
   - Use the following command to run the visualizer:
     ```bash
     python viewer.py
     ```

3. **Script Parameters**:
   - `cloud_directory`: Path to the directory containing point cloud (`.bin`) files.
   - `label_directory`: Path to the directory containing bounding box (`.txt`) files.
   - `start_idx`: The starting index of the frames to load (e.g., `000230.bin` and `000230.txt`).
   - `debug_flag`: Set `True` for debugging mode, which enables detailed logging of frames and the RANSAC plane fit process.
   - `frame_interval`: Time interval between frames in milliseconds when automatic frame progression is enabled.

4. **Example**:
   - To visualize frames starting from frame `230` with automatic progression every 10 milliseconds:
     ```bash
     python viewer.py
     ```

## Key Commands

- **N**: Load the next frame manually.
- **I**: Zoom into the scene.
- **O**: Zoom out of the scene.

## Flags

- **debug_flag (bool)**: 
  - Set to `True` to enable full debugging information and disable automatic frame progression.
  - Default: `False`.
  
- **frame_interval (int)**:
  - Time (in milliseconds) between each frame when automatic progression is enabled.
  - Default: `200` milliseconds.

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