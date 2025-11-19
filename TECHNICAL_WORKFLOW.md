# Technical Workflow: Camera-LiDAR Calibration and Point Cloud Colorization

## Overview

This workflow performs camera-LiDAR calibration, point cloud alignment, and RGB colorization for multi-camera LiDAR datasets. The pipeline produces a globally aligned, RGB-colored point cloud by fusing data from two cameras.

## Data Structure

```
walkley data/
├── PointCloudsIntensity/    # LiDAR scans (*.pcd)
│   ├── 00000.pcd
│   ├── 00001.pcd
│   └── ...
├── primaryImages/           # Primary camera frames (*.png)
│   ├── 00000.png
│   ├── 00001.png
│   └── ...
└── secondaryImages/         # Secondary camera frames (*.png)
    ├── 00000.png
    ├── 00001.png
    └── ...

calibration/                 # Optional: Checkerboard calibration data
├── PointClouds/
├── primaryImages/
└── secondaryImages/
```

## Pipeline Stages

### Stage 1: Camera-LiDAR Calibration

**Tool:** `calibrate_point_picking.py`

Interactive point-picking tool to establish 3D-2D correspondences between LiDAR points and camera pixels.

**Method:**
- Manual selection of corresponding points in point cloud and image
- Solves Perspective-n-Point (PnP) problem to compute extrinsic parameters (R, t)
- Uses known camera intrinsics (K matrix)

**Primary Camera Calibration:**
```bash
python calibrate_point_picking.py --camera primary --frame 0 --use_checkerboard
```

**Secondary Camera Calibration:**
```bash
python calibrate_point_picking.py \
    --data_dir "C:\Users\Abdul-AzizAlNajjar\CU\aligning_points\data\walkley\walkley data" \
    --camera secondary \
    --initial_calib walkley_secondary_calibration.txt \
    --frame 1
```

**Interactive Controls:**
- `P` - Pick point in 3D point cloud (Shift+Click in viewer)
- Click - Select corresponding point in 2D image
- `C` - Compute calibration (requires ≥4 point pairs)
- `U` - Undo last point pair
- `R` - Reset all points
- `S` - Save calibration file
- `Q` - Quit

**Output:** Calibration files containing:
- Camera intrinsics (K): 3×3 matrix
- Rotation matrix (R): 3×3 matrix (LiDAR → camera frame)
- Translation vector (t): 3×1 vector
- Euler angles (roll, pitch, yaw)

### Stage 2: Calibration Verification

**Tool:** `visualize_walkley_calibration.py`

Projects LiDAR points onto camera images using calibration to verify alignment quality.

**Primary Camera:**
```bash
python visualize_walkley_calibration.py \
    --camera primary \
    --calib_file walkley_primary_calibration.txt \
    --output_dir walkley_calibration_check_primary
```

**Secondary Camera:**
```bash
python visualize_walkley_calibration.py \
    --camera secondary \
    --calib_file walkley_secondary_calibration.txt \
    --output_dir walkley_calibration_check_secondary
```

**Video Generation:**
```bash
python create_video_from_frames.py \
    --input_dir walkley_calibration_check_primary \
    --output calibration_check_primary.gif \
    --fps 5
```

**Visualization:** Points colored by depth (blue=near, red=far)

### Stage 3: Point Cloud Alignment

**Tool:** KISS-ICP (Keep It Small and Simple ICP)

Performs LiDAR odometry to compute per-frame poses and create a globally consistent map.

```bash
kiss_icp_pipeline --visualize "C:\Users\Abdul-AzizAlNajjar\CU\aligning_points\data\walkley\walkley data\PointCloudsIntensity"
```

**Output Location:** `results/latest/`
- `PointCloudsIntensity_poses.npy` - 4×4 transformation matrices (local → global)
- `complete_map.pcd` - Aligned point cloud (may lack intensity field)
- `config.yml` - Algorithm parameters

### Stage 4: Complete Map Generation

**Tool:** `saving_map.py`

Rebuilds complete map with all fields (positions, intensity, scalar_label) preserved.

```bash
python saving_map.py
```

**Process:**
1. Load poses from `PointCloudsIntensity_poses.npy`
2. Transform each frame to global coordinates using 4×4 pose matrices
3. Concatenate all transformed points
4. Preserve intensity and scalar_label fields

**Output:** `results/latest/Walkley_complete_full.pcd`

### Stage 5: Point Cloud Colorization

**Tool:** `colorize_pointcloud.py`

Projects aligned point cloud back to camera frames to extract RGB colors.

**Dual-Camera Fusion (Recommended):**
```bash
python colorize_pointcloud.py \
    --no-include_all_points \
    --saturation_boost 2.5 \
    --gamma 0.75 \
    --use_both_cameras
```

**Single Camera:**
```bash
python colorize_pointcloud.py \
    --camera primary \
    --no-include_all_points \
    --saturation_boost 2.5 \
    --gamma 0.75
```

**Algorithm:**
1. **Load aligned map** - Reads complete point cloud with all fields
2. **Per-frame projection:**
   - For each frame, transform global points back to local frame
   - Project local points to camera image using calibration (K, R, t)
   - Extract RGB values at projected pixel coordinates
   - Apply color enhancement (saturation boost, gamma correction)
3. **Color averaging** - Points visible in multiple frames/cameras are averaged
4. **Post-processing:**
   - Statistical outlier removal (20 neighbors, 2.0 std ratio)
   - Optional voxel downsampling (currently disabled)

**Parameters:**
- `--use_both_cameras` - Fuse colors from primary and secondary cameras
- `--saturation_boost 2.5` - Increase color vibrancy (default: 2.0)
- `--gamma 0.75` - Brighten colors (< 1.0 brightens, > 1.0 darkens)
- `--no-include_all_points` - Only save points visible in camera frames

**Output:** `results/latest/walkley_colored_map.pcd`
- RGB colors (0-255, converted to 0-1 range)
- Original intensity field preserved
- Original scalar_label field preserved

## Calibration File Format

```
# Walkley Dataset Calibration - primary camera
# Frame used: 1

# Camera Intrinsics (K)
2018.1000000000017 0.0 960.0
0.0 2018.1000000000017 600.0
0.0 0.0 1.0

# Rotation Matrix (R)
-0.011553342828645775 -0.9997804802942524 0.017478887037728572
-0.010602642995305752 -0.017356585586652026 -0.9997931450546593
0.9998770445564784 -0.011736275361762766 -0.010399788922617953

# Translation Vector (t)
0.2515451343777162 -0.053525501067545826 -0.009094872369760159

# Euler Angles (degrees)
Roll: -131.54490567620832
Pitch: -89.10150438773171
Yaw: -137.45701971206634
```

## Mathematical Framework

### Projection Equation

For a 3D point **P**_lidar in LiDAR frame, projection to pixel **p** in camera image:

1. **Transform to camera frame:**
   ```
   P_cam = R × P_lidar + t
   ```

2. **Project to image plane:**
   ```
   p_homogeneous = K × P_cam
   p = [p_x / p_z, p_y / p_z]
   ```

Where:
- **K** - 3×3 camera intrinsic matrix (focal length, principal point)
- **R** - 3×3 rotation matrix (LiDAR → camera)
- **t** - 3×1 translation vector

### Multi-Camera Color Fusion

For points visible in multiple frames/cameras:

```
RGB_final = (Σ RGB_i) / N
```

Where N is the number of valid observations.

## Key Features

### Calibration Tool
- **High-saturation point cloud visualization** - Intensity-based coloring for easier point identification
- **Persistent point markers** - Color-coded spheres show picked points
- **Multiple PnP solvers** - Tests ITERATIVE, EPNP, SQPNP, P3P with/without initial guess
- **RANSAC support** - Robust to outliers when ≥6 point pairs
- **Reprojection error metrics** - Per-point and mean error reporting

### Colorization Pipeline
- **Frame-aware processing** - Tracks which frame each point originated from
- **Multi-camera fusion** - Combines colors from primary and secondary cameras
- **Color enhancement** - Saturation boost and gamma correction
- **Field preservation** - Maintains original intensity and scalar_label values
- **Statistical outlier removal** - Cleans noisy points

## Dependencies

```
numpy
opencv-python (cv2)
open3d
scipy
tqdm
imageio (for GIF creation)
kiss-icp
```

## Visualization

**View colored point cloud:**
```bash
open3d results/latest/walkley_colored_map.pcd
```

**Python visualization:**
```python
import open3d as o3d
pcd = o3d.t.io.read_point_cloud('results/latest/walkley_colored_map.pcd')
o3d.visualization.draw_geometries([pcd.to_legacy()])
```

## File Outputs Summary

| File | Description |
|------|-------------|
| `walkley_primary_calibration.txt` | Primary camera extrinsics |
| `walkley_secondary_calibration.txt` | Secondary camera extrinsics |
| `calibration_check_*.gif` | Verification videos |
| `results/latest/PointCloudsIntensity_poses.npy` | KISS-ICP odometry poses |
| `results/latest/Walkley_complete_full.pcd` | Aligned map with intensity |
| `results/latest/walkley_colored_map.pcd` | Final RGB-colored point cloud |

## Typical Workflow Execution

```bash
# 1. Calibrate primary camera
python calibrate_point_picking.py --camera primary --frame 0 --use_checkerboard

# 2. Calibrate secondary camera
python calibrate_point_picking.py --camera secondary --frame 1

# 3. Verify calibrations
python visualize_walkley_calibration.py --camera primary
python visualize_walkley_calibration.py --camera secondary
python create_video_from_frames.py --input_dir walkley_calibration_check_primary --output calibration_check_primary.gif

# 4. Align point clouds
kiss_icp_pipeline --visualize "C:\path\to\PointCloudsIntensity"

# 5. Generate complete map
python saving_map.py

# 6. Colorize with both cameras
python colorize_pointcloud.py --no-include_all_points --saturation_boost 2.5 --gamma 0.75 --use_both_cameras

# 7. Visualize
open3d results/latest/walkley_colored_map.pcd
```

## Notes

- **Frame selection:** Use checkerboard frames for calibration when available (better geometric constraints)
- **Point pair guidelines:** 
  - Minimum 4 pairs required, 6+ recommended for RANSAC
  - Spread points across entire image
  - Pick distinct, unambiguous features
  - Avoid coplanar point configurations
- **Color enhancement:** Adjust `--saturation_boost` (1.0-3.0) and `--gamma` (0.6-1.2) for desired appearance
- **Memory:** Processing large datasets may require 16GB+ RAM
- **Coordinate frames:** KISS-ICP outputs right-handed coordinate system

## Troubleshooting

**Poor calibration (high reprojection error):**
- Add more point pairs with better spatial distribution
- Use checkerboard data for more accurate correspondences
- Verify points are not coplanar

**Black/missing colors in output:**
- Check calibration verification videos
- Ensure camera images match point cloud timestamps
- Try `--include_all_points` to include non-visible points

**Memory errors:**
- Enable voxel downsampling in `colorize_pointcloud.py`
- Process fewer frames by modifying file lists

---

**Generated:** November 19, 2025  
**Pipeline Version:** v1.0

