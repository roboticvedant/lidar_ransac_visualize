# PointCloud Visualizer & Correction Tool

This tool visualizes point cloud data and bounding boxes from `.bin` and `.txt` files using Open3D. It supports features such as zoom controls, frame-by-frame navigation, automatic frame progression, and saving corrected data with RANSAC plane fitting.

## Prerequisites

Ensure that you have **Conda** installed on your machine. You can install [Miniconda](https://docs.conda.io/en/latest/miniconda.html) if you do not have Conda.

## Setting Up the Environment

To set up the environment with the required dependencies, follow these steps:

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
   - Verify the environment by checking the installed packages:
     ```bash
     conda list
     ```

## Running the Visualizer

### Command-Line Arguments

- **`-r`, `--raw_data_dir`** (default: `RACECAR_DATA/data/`): Path to the directory containing raw point cloud data.
- **`-f`, `--fixed_data_dir`** (default: `RACECAR_DATA/correctedData/`): Path to the directory where corrected data will be stored.
- **`-p`, `--play_animation`**: Enable automatic frame progression.
- **`-g`, `--generate`**: Generate corrected data and save it.
- **`-n`, `--no_ransac`**: Disable RANSAC visualization.
- **`-s`, `--start_idx`** (default: `0`): The starting index for visualization.
- **`-i`, `--frame_interval`** (default: `200` ms): Time in milliseconds between frames for automatic progression.
- **`-l`, `--length_of_car`** (default: `5.0`): Length of the car.
- **`-w`, `--width_of_car`** (default: `2.0`): Width of the car.
- **`-hc`, `--height_of_car`** (default: `2.0`): Height of the car.
- **`-a`, `--axle_from_center`** (default: `2.0`): Distance from the center to the rear axle.
- **`-hr`, `--height_of_rear_axle`** (default: `0.5`): Height of the rear axle from the ground.

### Example Command

To run the script with default parameters:

```bash
python viewer.py -r path/to/raw_data -f path/to/fixed_data -p -g -s 100 -i 100
```

### Key Commands During Visualization

- **N**: Load the next frame manually.
- **I**: Zoom in.
- **O**: Zoom out.
- **V**: Toggle between side and top views.
- **S**: Save the corrected bounding box to a text file.

## Directory Structure

Ensure your data is structured as follows:

```
RACECAR_DATA/
  └── data/
      ├── cloud/    # Contains .bin point cloud files
      └── labels/   # Contains .txt label files
```

## Example Usage in Python

```python
visualizer = VisualizerSequence(
    raw_data_dir="RACECAR_DATA/data/",
    fixed_data_dir="RACECAR_DATA/correctedData/",
    start_idx=230,
    play_animation=True,
    generating_data=True,
    ransac_visualize_flag=True,
    frame_interval=200,
    length_of_car=5.0,
    width_of_car=2.0,
    height_of_car=2.0,
    axle_from_center=2.0,
    height_of_rear_axle=0.5
)
visualizer.run()
```
