# KISS-ICP Point Cloud Alignment Documentation

## Overview

KISS-ICP (Keep It Small and Simple - Iterative Closest Point) is a LiDAR odometry pipeline that aligns sequential point clouds to create a map and estimate the trajectory of a moving sensor. This document covers the complete process from setup to execution.

## What is Point Cloud Alignment?

Point cloud alignment is the process of:
1. **Registration**: Finding the transformation (rotation + translation) between consecutive point cloud frames
2. **Mapping**: Building a cumulative map by combining aligned point clouds
3. **Odometry**: Estimating the sensor's trajectory through space

## Installation and Setup

### Prerequisites
```bash
# Create a virtual environment
python -m venv kiss-env
cd kiss-env
Scripts\activate  # On Windows

# Install KISS-ICP
pip install kiss-icp
```

### Common Issues and Solutions

#### Polyscope Compatibility Error
If you encounter `TypeError: cannot unpack non-iterable PickResult object`:

1. Edit the visualizer file:
   ```
   kiss-env\Lib\site-packages\kiss_icp\tools\visualizer.py
   ```

2. Replace the `_trajectory_pick_callback` method (around line 270):
   ```python
   def _trajectory_pick_callback(self):
       if self._gui.GetIO().MouseClicked[0]:
           pick_result = self._ps.get_selection()
           # Handle both older tuple format and newer PickResult object
           try:
               # Try newer format first
               if hasattr(pick_result, 'is_hit') and pick_result.is_hit:
                   name = pick_result.structure_name
                   idx = pick_result.local_index
               else:
                   # Fall back to older tuple unpacking format
                   name, idx = pick_result
           except (ValueError, TypeError):
               return  # Selection failed or returned unexpected format
           
           if name == "trajectory" and self._ps.has_point_cloud(name):
               pose = self._trajectory[idx]
               self._selected_pose = f"x: {pose[0]:7.3f}, y: {pose[1]:7.3f}, z: {pose[2]:7.3f}"
   ```

#### Windows Symlink Permission Error
If you get `OSError: [WinError 1314] A required privilege is not held by the client`:

1. Edit the pipeline file:
   ```
   kiss-env\Lib\site-packages\kiss_icp\pipeline.py
   ```

2. Replace the `_get_results_dir` method (around line 245-257):
   ```python
   def _get_results_dir(self, out_dir):
       """Create the output dir if it doesn't exist yet."""
       timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
       results_dir = os.path.join(os.path.realpath(out_dir), timestamp)
       latest_dir = os.path.join(os.path.realpath(out_dir), "latest")
       os.makedirs(results_dir, exist_ok=True)
       
       # Remove existing "latest" link/directory if it exists
       if os.path.exists(latest_dir) or os.path.islink(latest_dir):
           try:
               os.unlink(latest_dir)
           except OSError:
               if os.path.isdir(latest_dir):
                   import shutil
                   shutil.rmtree(latest_dir)
       
       # Try to create a symlink, but fall back to a simple file with the path on Windows
       try:
           os.symlink(results_dir, latest_dir)
       except OSError:
           # On Windows without admin privileges, create a text file instead
           with open(latest_dir + ".txt", "w") as f:
               f.write(f"Latest results directory: {results_dir}")
           print(f"Created pointer file at {latest_dir}.txt instead of symlink")
       
       return results_dir
   ```

## Data Preparation

### Supported Formats
- **Generic**: Any directory containing point cloud files (.pcd, .ply, .txt, etc.)
- **KITTI**: KITTI odometry dataset format
- **ROS Bag**: ROS bag files with PointCloud2 messages
- **Custom formats**: Various LiDAR datasets (Apollo, Boreas, etc.)

### Directory Structure
```
data/
└── PointClouds/
    ├── cloud_001.pcd
    ├── cloud_002.pcd
    ├── cloud_003.pcd
    └── ...
```

## Running the Alignment Pipeline

### Basic Usage
```bash
# Navigate to your project directory
cd C:\Users\YourName\YourProject

# Activate the environment
kiss-env\Scripts\activate

# Run the pipeline with visualization
kiss_icp_pipeline --visualize "data\PointClouds"

# Run without visualization (faster)
kiss_icp_pipeline "data\PointClouds"
```

### Command Line Options
```bash
kiss_icp_pipeline [OPTIONS] DATA_PATH

Options:
  --visualize          Enable real-time visualization
  --n-scans INTEGER    Number of scans to process (-1 for all)
  --jump INTEGER       Skip frames at the beginning
  --max-range FLOAT    Maximum range for point filtering
  --deskew             Enable motion deskewing
  --config PATH        Path to custom configuration file
```

### Example Commands
```bash
# Process only first 100 scans with visualization
kiss_icp_pipeline --visualize --n-scans 100 "data\PointClouds"

# Skip first 50 frames and process the rest
kiss_icp_pipeline --jump 50 "data\PointClouds"

# Use custom maximum range
kiss_icp_pipeline --max-range 50.0 "data\PointClouds"
```

## The Alignment Process

### Step-by-Step Process

1. **Data Loading**
   - Loads point clouds sequentially from the specified directory
   - Applies preprocessing (filtering, downsampling)

2. **Feature Extraction**
   - Extracts keypoints from each point cloud
   - Uses adaptive thresholding for robust feature selection

3. **Registration**
   - Aligns current frame with the accumulated map
   - Uses ICP (Iterative Closest Point) algorithm
   - Estimates 6-DOF pose (3D position + 3D orientation)

4. **Map Update**
   - Adds aligned points to the `VoxelHashMap`
   - Maintains efficient spatial indexing
   - Removes old/redundant points to control memory usage

5. **Trajectory Estimation**
   - Tracks the sensor's path through space
   - Outputs poses as 4x4 transformation matrices

### Key Components

#### VoxelHashMap
- Efficient 3D spatial data structure
- Stores points in voxel grid for fast nearest neighbor queries
- Configurable voxel size (default: 0.1m)

#### Adaptive Thresholding
- Dynamically adjusts parameters based on point cloud density
- Ensures robust registration across different environments

#### Motion Compensation
- Optional deskewing for rotating LiDAR sensors
- Corrects for sensor motion during scan acquisition

## Output Files

After running the pipeline, you'll find these files in the `results` directory:

```
results/
├── latest/  (or latest.txt on Windows)
└── 2025-11-10_14-30-45/
    ├── poses_kitti.txt          # Estimated trajectory in KITTI format
    ├── poses_tum.txt            # Estimated trajectory in TUM format
    ├── gt_poses_kitti.txt       # Ground truth poses (if available)
    ├── gt_poses_tum.txt         # Ground truth poses (if available)
    ├── config.yaml              # Configuration used
    ├── log.txt                  # Processing log
    └── complete_map.txt         # Final point cloud map (if added)
```

### Pose Formats

**KITTI Format** (poses_kitti.txt):
```
r11 r12 r13 tx r21 r22 r23 ty r31 r32 r33 tz
```

**TUM Format** (poses_tum.txt):
```
timestamp tx ty tz qx qy qz qw
```

## Saving Complete Maps

### Method 1: Built-in Map Extraction

Add this method to `kiss-env\Lib\site-packages\kiss_icp\pipeline.py`:

```python
def _save_complete_map(self):
    """Save the complete point cloud map."""
    print("Saving the complete map...")
    if not self.results_dir:
        return
        
    complete_map = self.odometry.local_map.point_cloud()
    print(f"Complete map has {complete_map.shape[0]} points")

    # Save as TXT
    map_file = os.path.join(self.results_dir, "complete_map.txt")
    np.savetxt(map_file, complete_map)
    print(f"Saved complete map to {map_file}")
    
    # Optionally save as PCD
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(complete_map)
        o3d.io.write_point_cloud(os.path.join(self.results_dir, "complete_map.pcd"), pcd)
    except ImportError:
        pass

# Add to run() method:
def run(self):
    # ... existing code ...
    self._save_complete_map()  # Add this line before return
    return self.results
```

**Note:** This method saves only the final voxelized map without preserving point attributes (intensity, labels).

### Method 2: Full Reconstruction with Attributes (saving_map.py)

For preserving all point attributes (intensity, labels, etc.), use the `saving_map.py` script after running the pipeline:

**Purpose:** Reconstructs the complete map by transforming all original point clouds using estimated poses, preserving all attributes.

**Usage:**
```bash
# 1. Run KISS-ICP pipeline first
kiss_icp_pipeline "data\PointClouds"

# 2. Run the map reconstruction script
python saving_map.py
```

**What it does:**
- Loads estimated poses from `results\latest\*_poses.npy`
- Transforms each original point cloud to global coordinates
- Preserves intensity and label attributes
- Applies voxel downsampling and outlier removal
- Outputs high-quality merged map as `results\latest\complete_map.pcd`

**Key advantages:**
- Preserves all point attributes (intensity, labels, colors)
- Higher quality than voxel hash map extraction
- Allows custom post-processing (filtering, downsampling)

**Configuration in saving_map.py:**
```python
# Adjust these parameters as needed:
voxel_size = 0.05              # Downsample resolution (meters)
nb_neighbors = 20              # Outlier removal: neighbor count
std_ratio = 2.0                # Outlier removal: std threshold
```

## Visualization Controls

When using `--visualize`, you can interact with the 3D viewer:

- **Mouse**: Rotate, pan, zoom the 3D view
- **Space**: Pause/resume processing
- **R**: Reset camera view
- **Click trajectory**: Select specific poses for inspection

The visualization shows:
- **Green points**: Current frame being processed
- **Blue points**: Accumulated map
- **Red line**: Estimated trajectory
- **Yellow points**: Keypoints used for registration

## Troubleshooting

### Common Issues

1. **No point clouds found**
   - Check file formats are supported
   - Verify directory path is correct

2. **Poor alignment results**
   - Adjust `max_range` parameter
   - Check point cloud quality and density
   - Verify coordinate system consistency

3. **Memory issues with large datasets**
   - Use `--n-scans` to process smaller batches
   - Reduce voxel map size in configuration

4. **Slow processing**
   - Disable visualization for faster processing
   - Use appropriate `max_range` to filter distant points

### Performance Tips

- Use SSD storage for faster file I/O
- Adjust voxel size based on point cloud density
- Filter out ground points if not needed
- Use appropriate downsampling for dense point clouds

## Configuration

Create a custom configuration file (config.yaml):

```yaml
data:
  max_range: 100.0
  min_range: 5.0
  deskew: false

mapping:
  voxel_size: 0.1
  max_points_per_voxel: 20
  max_distance: 100.0

adaptive_threshold:
  initial_threshold: 2.0
  min_motion_th: 0.1
```

Use it with:
```bash
kiss_icp_pipeline --config config.yaml "data\PointClouds"
```

This documentation should help you remember the complete process for next time!
