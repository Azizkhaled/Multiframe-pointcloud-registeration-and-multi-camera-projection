"""
Colorize aligned point cloud using camera images and calibration
Projects the aligned map back to each camera frame to extract RGB values
"""

import numpy as np
import cv2
import open3d as o3d
from pathlib import Path
import glob
from tqdm import tqdm
import argparse

def load_calibration(calib_file):
    """Load calibration from text file"""
    K = np.eye(3)
    R = np.eye(3)
    t = np.zeros(3)
    
    with open(calib_file, 'r') as f:
        lines = f.readlines()
        
        # Read K
        K[0] = [float(x) for x in lines[4].strip().split()]
        K[1] = [float(x) for x in lines[5].strip().split()]
        K[2] = [float(x) for x in lines[6].strip().split()]
        
        # Read R
        R[0] = [float(x) for x in lines[9].strip().split()]
        R[1] = [float(x) for x in lines[10].strip().split()]
        R[2] = [float(x) for x in lines[11].strip().split()]
        
        # Read t
        t = np.array([float(x) for x in lines[14].strip().split()])
    
    return K, R, t

def read_pcd_file(filename):
    """Read PCD file and extract XYZ points"""
    points = []
    with open(filename, 'r') as f:
        in_data = False
        for line in f:
            if line.startswith('DATA'):
                in_data = True
                continue
            if in_data:
                parts = line.strip().split()
                if len(parts) >= 3:
                    try:
                        x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                        points.append([x, y, z])
                    except:
                        continue
    return np.array(points)

def read_pcd_file_with_intensity(filename):
    """Read PCD file with Open3D to preserve all fields including intensity"""
    pcd = o3d.t.io.read_point_cloud(filename)
    positions = pcd.point["positions"].numpy()
    
    # Get intensity values
    if "intensity" in pcd.point:
        intensity = pcd.point["intensity"].numpy()
    elif "scalar_intensity" in pcd.point:
        intensity = pcd.point["scalar_intensity"].numpy()
    else:
        intensity = np.ones((positions.shape[0], 1), dtype=np.float32)
    
    # Get scalar_label values
    if "scalar_label" in pcd.point:
        scalar_label = pcd.point["scalar_label"].numpy()
    else:
        scalar_label = np.zeros((positions.shape[0], 1), dtype=np.float32)
    
    # Ensure proper shapes (Nx1)
    if len(intensity.shape) == 1:
        intensity = intensity.reshape(-1, 1)
    if len(scalar_label.shape) == 1:
        scalar_label = scalar_label.reshape(-1, 1)
    
    return positions, intensity, scalar_label

def project_to_camera(points_lidar, K, R, t):
    """
    Project 3D points from lidar frame to camera image
    
    Args:
        points_lidar: Nx3 array of points in lidar frame
        K: 3x3 camera intrinsic matrix
        R: 3x3 rotation matrix (lidar to camera)
        t: 3 translation vector (lidar to camera)
    
    Returns:
        points_2d: Nx2 array of pixel coordinates
        valid_mask: N boolean array indicating which points project inside camera view
    """
    # Transform from lidar to camera frame
    points_cam = points_lidar @ R.T + t
    
    # Filter points behind camera
    valid_depth = points_cam[:, 2] > 0.1  # At least 10cm in front
    
    # Project to image plane
    points_2d_hom = points_cam @ K.T
    points_2d = points_2d_hom[:, :2] / points_2d_hom[:, 2:3]
    
    return points_2d, valid_depth, points_cam[:, 2]

def enhance_colors(colors, saturation_boost=1.5, gamma=1.0):
    """
    Enhance color vibrancy using saturation boost and gamma correction
    
    Args:
        colors: Nx3 array of RGB colors (0-255)
        saturation_boost: Factor to multiply saturation by (> 1.0 increases vibrancy)
        gamma: Gamma correction factor (< 1.0 brightens, > 1.0 darkens)
    
    Returns:
        enhanced_colors: Nx3 array of enhanced RGB colors (0-255)
    """
    if len(colors) == 0:
        return colors
    
    # Convert to float [0, 1]
    colors_float = colors.astype(np.float32) / 255.0
    
    # Apply gamma correction
    if gamma != 1.0:
        colors_float = np.power(colors_float, gamma)
    
    # Convert RGB to HSV for saturation boost
    if saturation_boost != 1.0:
        # Reshape for cv2 (needs at least 2D)
        original_shape = colors_float.shape
        colors_rgb = (colors_float * 255).astype(np.uint8).reshape(-1, 1, 3)
        
        # Convert to HSV
        colors_hsv = cv2.cvtColor(colors_rgb, cv2.COLOR_RGB2HSV).astype(np.float32)
        
        # Boost saturation (channel 1)
        colors_hsv[:, :, 1] = np.clip(colors_hsv[:, :, 1] * saturation_boost, 0, 255)
        
        # Convert back to RGB
        colors_hsv_uint8 = colors_hsv.astype(np.uint8)
        colors_rgb = cv2.cvtColor(colors_hsv_uint8, cv2.COLOR_HSV2RGB)
        colors_float = colors_rgb.reshape(original_shape).astype(np.float32) / 255.0
    
    # Convert back to uint8
    enhanced = np.clip(colors_float * 255, 0, 255).astype(np.uint8)
    return enhanced

def extract_colors_from_image(points_2d, image, valid_mask):
    """
    Extract RGB colors from image at projected point locations
    
    Args:
        points_2d: Nx2 array of pixel coordinates
        image: HxWx3 image array (BGR format from cv2)
        valid_mask: N boolean array
    
    Returns:
        colors: Nx3 array of RGB colors (0-255)
        color_valid_mask: N boolean array indicating which points have valid colors
    """
    h, w = image.shape[:2]
    colors = np.zeros((len(points_2d), 3), dtype=np.uint8)
    color_valid_mask = np.zeros(len(points_2d), dtype=bool)
    
    # Check which points are within image bounds
    x_coords = points_2d[:, 0].astype(int)
    y_coords = points_2d[:, 1].astype(int)
    
    in_bounds = (x_coords >= 0) & (x_coords < w) & (y_coords >= 0) & (y_coords < h)
    final_valid = valid_mask & in_bounds
    
    # Extract colors for valid points
    valid_x = x_coords[final_valid]
    valid_y = y_coords[final_valid]
    
    # OpenCV uses BGR, convert to RGB
    colors[final_valid] = image[valid_y, valid_x, ::-1]
    color_valid_mask[final_valid] = True
    
    return colors, color_valid_mask

def colorize_pointcloud(data_dir, calib_file, results_dir='results/latest',
                       output_file='results/latest/walkley_colored_map.pcd',
                       camera='primary', saturation_boost=2.0, gamma=0.8,
                       include_all_points=True, use_both_cameras=False,
                       secondary_calib_file='walkley_secondary_calibration.txt'):
    """
    Colorize the aligned point cloud using camera images
    
    Args:
        data_dir: Path to walkley data directory (contains PointCloudsIntensity and primaryImages)
        calib_file: Path to calibration file for primary camera
        results_dir: Directory containing KISS-ICP results (poses.npy)
        output_file: Where to save the colored point cloud
        camera: Which camera to use when use_both_cameras=False ('primary' or 'secondary')
        saturation_boost: Factor to boost color saturation (default 2.0, use 1.0 for no boost)
        gamma: Gamma correction factor (default 0.8 brightens, 1.0 for no correction)
        include_all_points: If True, include all points (with black for non-visible). 
                           If False, only include points visible in camera frames.
        use_both_cameras: If True, use both primary and secondary cameras for coloring
        secondary_calib_file: Path to calibration file for secondary camera
    """
    
    print("=" * 60)
    print("Point Cloud Colorization Pipeline")
    print("=" * 60)
    
    # Setup cameras to process
    if use_both_cameras:
        cameras_config = [
            {'name': 'primary', 'calib_file': calib_file},
            {'name': 'secondary', 'calib_file': secondary_calib_file}
        ]
        print(f"\n✓ Using BOTH cameras for colorization")
    else:
        cameras_config = [
            {'name': camera, 'calib_file': calib_file if camera == 'primary' else secondary_calib_file}
        ]
        print(f"\n✓ Using single camera: {camera}")
    
    # Load calibrations
    camera_params = []
    for cam_cfg in cameras_config:
        K, R, t = load_calibration(cam_cfg['calib_file'])
        camera_params.append({
            'name': cam_cfg['name'],
            'K': K, 'R': R, 't': t,
            'calib_file': cam_cfg['calib_file']
        })
        print(f"  {cam_cfg['name'].capitalize()} camera:")
        print(f"    Calibration: {cam_cfg['calib_file']}")
        print(f"    Focal length: {K[0,0]:.1f} pixels")
        print(f"    Principal point: ({K[0,2]:.1f}, {K[1,2]:.1f})")
    
    # Color enhancement settings
    print(f"\n✓ Processing settings:")
    print(f"  Saturation boost: {saturation_boost}x")
    print(f"  Gamma correction: {gamma}")
    print(f"  Include all points: {include_all_points} {'(with black for non-visible)' if include_all_points else '(only colored points)'}")
    
    # Load KISS-ICP poses
    poses_file = Path(results_dir) / 'PointCloudsIntensity_poses.npy'
    poses = np.load(poses_file)
    print(f"\n✓ Loaded {len(poses)} poses from: {poses_file}")
    
    # Get file lists
    data_path = Path(data_dir)
    pc_dir = data_path / "PointCloudsIntensity"
    
    pc_files = sorted(list(pc_dir.glob("*.pcd")))
    
    # Get image files for each camera
    img_files_per_camera = {}
    for cam_cfg in cameras_config:
        img_dir = data_path / f"{cam_cfg['name']}Images"
        img_files_per_camera[cam_cfg['name']] = sorted(list(img_dir.glob("*.png")))
    
    # Determine number of frames (minimum across all sources)
    min_img_count = min(len(img_files_per_camera[cam['name']]) for cam in cameras_config)
    num_frames = min(len(pc_files), min_img_count, len(poses))
    
    print(f"\n✓ Found {num_frames} frames to process")
    print(f"  Point clouds: {pc_dir}")
    for cam_cfg in cameras_config:
        img_dir = data_path / f"{cam_cfg['name']}Images"
        print(f"  {cam_cfg['name'].capitalize()} images: {img_dir}")
    
    # PHASE 1: Build complete aligned map with all points (like saving_map.py)
    print("\nPhase 1: Loading and transforming all point clouds...")
    global_pcd = o3d.t.geometry.PointCloud()
    first = True
    point_frame_indices = []  # Track which frame each point came from
    
    for i, (pose, file_path) in enumerate(tqdm(zip(poses[:num_frames], pc_files[:num_frames]), 
                                                desc="Loading", total=num_frames)):
        # Load point cloud with intensity
        positions, intensity, scalar_label = read_pcd_file_with_intensity(file_path)
        
        # Apply the 4x4 transform to points (local to global)
        hom_points = np.hstack([positions, np.ones((positions.shape[0], 1))])
        transformed = (pose @ hom_points.T).T[:, :3]
        
        # Build tensor point cloud
        new_pcd = o3d.t.geometry.PointCloud()
        new_pcd.point["positions"] = o3d.core.Tensor(transformed, dtype=o3d.core.Dtype.Float32)
        new_pcd.point["intensity"] = o3d.core.Tensor(intensity, dtype=o3d.core.Dtype.Float32)
        new_pcd.point["scalar_label"] = o3d.core.Tensor(scalar_label, dtype=o3d.core.Dtype.Float32)
        
        # Track frame indices
        point_frame_indices.extend([i] * len(positions))
        
        # Accumulate
        if first:
            global_pcd = new_pcd
            first = False
        else:
            global_pcd.point["positions"] = o3d.core.concatenate(
                [global_pcd.point["positions"], new_pcd.point["positions"]], axis=0)
            global_pcd.point["intensity"] = o3d.core.concatenate(
                [global_pcd.point["intensity"], new_pcd.point["intensity"]], axis=0)
            global_pcd.point["scalar_label"] = o3d.core.concatenate(
                [global_pcd.point["scalar_label"], new_pcd.point["scalar_label"]], axis=0)
    
    # Get all points and properties as numpy arrays
    all_global_points = global_pcd.point["positions"].numpy()
    all_intensities = global_pcd.point["intensity"].numpy()
    all_scalar_labels = global_pcd.point["scalar_label"].numpy()
    point_frame_indices = np.array(point_frame_indices)
    
    num_total_points = len(all_global_points)
    print(f"  Total points in aligned map: {num_total_points:,}")
    
    # PHASE 2: Colorize points by projecting to camera frames
    print("\nPhase 2: Extracting colors from camera images...")
    
    # Initialize colors to black (0,0,0) for all points
    all_colors = np.zeros((num_total_points, 3), dtype=np.float32)
    color_counts = np.zeros(num_total_points, dtype=np.int32)  # Track how many times each point was colored
    
    # Track per-camera statistics
    colored_per_camera = {cam['name']: 0 for cam in camera_params}
    
    # Process each frame to extract colors
    for i in tqdm(range(num_frames), desc="Colorizing"):
        # Get points from this frame (in local coordinates)
        frame_mask = point_frame_indices == i
        frame_indices = np.where(frame_mask)[0]
        
        if len(frame_indices) == 0:
            continue
        
        # Load original local point cloud (for projection)
        positions_local, _, _ = read_pcd_file_with_intensity(pc_files[i])
        
        # Process each camera
        for cam_params in camera_params:
            cam_name = cam_params['name']
            K, R, t = cam_params['K'], cam_params['R'], cam_params['t']
            
            # Load image for this camera
            img_files = img_files_per_camera[cam_name]
            if i >= len(img_files):
                continue
            
            img = cv2.imread(str(img_files[i]))
            if img is None:
                continue
            
            # Project local points to camera
            points_2d, valid_depth, depths = project_to_camera(positions_local, K, R, t)
            
            # Extract colors for valid points
            colors_frame, color_valid = extract_colors_from_image(points_2d, img, valid_depth)
            
            # Enhance colors
            if len(colors_frame) > 0 and (saturation_boost != 1.0 or gamma != 1.0):
                colors_frame = enhance_colors(colors_frame, saturation_boost, gamma)
            
            # Update colors for points that were successfully colored
            valid_in_frame = np.where(color_valid)[0]
            global_indices = frame_indices[valid_in_frame]
            
            # Accumulate colors (we'll average later if a point is seen multiple times)
            all_colors[global_indices] += colors_frame[valid_in_frame].astype(np.float32)
            color_counts[global_indices] += 1
            
            # Track statistics
            colored_per_camera[cam_name] += len(valid_in_frame)
    
    # Average colors for points seen in multiple frames/cameras
    colored_mask = color_counts > 0
    all_colors[colored_mask] = all_colors[colored_mask] / color_counts[colored_mask, np.newaxis]
    
    num_colored = np.sum(colored_mask)
    num_uncolored = num_total_points - num_colored
    
    # Print statistics
    print(f"\n  Color extraction statistics:")
    if use_both_cameras:
        print(f"    Primary camera contributed: {colored_per_camera['primary']:,} point colors")
        print(f"    Secondary camera contributed: {colored_per_camera['secondary']:,} point colors")
        print(f"    (Note: Some points may be colored by both cameras and averaged)")
    else:
        print(f"    {camera.capitalize()} camera contributed: {colored_per_camera[camera]:,} point colors")
    
    print(f"\n  Unique colored points: {num_colored:,} ({100*num_colored/num_total_points:.1f}%)")
    if include_all_points:
        print(f"  Uncolored points (black): {num_uncolored:,} ({100*num_uncolored/num_total_points:.1f}%)")
    else:
        print(f"  Uncolored points will be excluded")
    
    # Filter to only colored points if requested
    if not include_all_points:
        print(f"\nFiltering to only colored points...")
        all_global_points = all_global_points[colored_mask]
        all_colors = all_colors[colored_mask]
        all_intensities = all_intensities[colored_mask]
        all_scalar_labels = all_scalar_labels[colored_mask]
        print(f"  Keeping {num_colored:,} colored points (removed {num_uncolored:,} uncolored)")
    
    # Convert colors to [0,1] range for Open3D
    all_colors = all_colors / 255.0
    
    # Create final point cloud with colors and intensity
    print("\nCreating final colored point cloud with intensity...")
    final_pcd = o3d.t.geometry.PointCloud()
    final_pcd.point["positions"] = o3d.core.Tensor(all_global_points, dtype=o3d.core.Dtype.Float32)
    final_pcd.point["colors"] = o3d.core.Tensor(all_colors, dtype=o3d.core.Dtype.Float32)
    final_pcd.point["intensity"] = o3d.core.Tensor(all_intensities, dtype=o3d.core.Dtype.Float32)
    final_pcd.point["scalar_label"] = o3d.core.Tensor(all_scalar_labels, dtype=o3d.core.Dtype.Float32)
    
    # Voxel downsample to reduce redundancy
    print("\nDownsampling is not done for now (preserving all points)")
    pcd_downsampled = final_pcd
    # voxel_size = 0.05
    # pcd_downsampled = final_pcd.voxel_down_sample(voxel_size)
    # print(f"  Points after downsampling: {len(pcd_downsampled.point['positions']):,}")
    
    # Optional: remove outliers
    print("\nRemoving statistical outliers...")
    pcd_legacy = pcd_downsampled.to_legacy()
    pcd_cleaned, ind = pcd_legacy.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  Points after cleaning: {len(pcd_cleaned.points):,}")
    
    # Convert back to tensor and restore intensity field
    pcd_cleaned_tensor = o3d.t.geometry.PointCloud.from_legacy(pcd_cleaned)
    pcd_cleaned_tensor.point["intensity"] = o3d.core.Tensor(
        all_intensities[ind], dtype=o3d.core.Dtype.Float32)
    pcd_cleaned_tensor.point["scalar_label"] = o3d.core.Tensor(
        all_scalar_labels[ind], dtype=o3d.core.Dtype.Float32)
    
    # Save using tensor API (preserves all fields)
    output_path = Path(output_file)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    o3d.t.io.write_point_cloud(str(output_path), pcd_cleaned_tensor)
    
    print(f"\n{'=' * 60}")
    print(f"✓ SUCCESS! Colored point cloud saved to:")
    print(f"  {output_path}")
    print(f"\nPoint cloud contains:")
    print(f"  - XYZ positions")
    if use_both_cameras:
        print(f"  - RGB colors (fused from BOTH cameras, enhanced with saturation {saturation_boost}x, gamma {gamma})")
    else:
        print(f"  - RGB colors (from {camera} camera, enhanced with saturation {saturation_boost}x, gamma {gamma})")
    print(f"  - Intensity values (scalar field)")
    print(f"  - Scalar labels (scalar field)")
    if include_all_points:
        print(f"  - Total points: {len(pcd_cleaned_tensor.point['positions']):,}")
        print(f"    • Colored: ~{int(num_colored * len(pcd_cleaned_tensor.point['positions']) / num_total_points):,} points")
        print(f"    • Black (not in camera view): remaining points")
    else:
        print(f"  - Total points: {len(pcd_cleaned_tensor.point['positions']):,} (colored only)")
    print(f"{'=' * 60}")
    print(f"\nTo visualize:")
    print(f"  open3d {output_path}")
    print(f"  or")
    print(f"  python -c \"import open3d as o3d; pcd = o3d.t.io.read_point_cloud('{output_path}'); o3d.visualization.draw_geometries([pcd.to_legacy()])\"")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Colorize aligned point cloud using camera images')
    parser.add_argument('--data_dir', type=str,
                       default=r'C:\Users\Abdul-AzizAlNajjar\CU\aligning_points\data\walkley\walkley data',
                       help='Path to Walkley data directory')
    parser.add_argument('--calib_file', type=str,
                       default='walkley_primary_calibration.txt',
                       help='Calibration file for primary camera')
    parser.add_argument('--secondary_calib_file', type=str,
                       default='walkley_secondary_calibration.txt',
                       help='Calibration file for secondary camera')
    parser.add_argument('--results_dir', type=str, default='results/latest',
                       help='Directory with KISS-ICP results (contains poses.npy)')
    parser.add_argument('--output', type=str, default='results/latest/walkley_colored_map.pcd',
                       help='Output file for colored point cloud')
    parser.add_argument('--camera', type=str, default='primary',
                       choices=['primary', 'secondary'],
                       help='Which camera to use (only used if --use_both_cameras is False)')
    parser.add_argument('--use_both_cameras', action='store_true',
                       help='Use BOTH primary and secondary cameras for coloring (recommended for better coverage)')
    parser.add_argument('--saturation_boost', type=float, default=2.0,
                       help='Saturation boost factor (default: 2.0, use 1.0 for no boost, try 2.0-3.0 for very vibrant)')
    parser.add_argument('--gamma', type=float, default=0.8,
                       help='Gamma correction factor (default: 0.8 brightens, 1.0 for no correction)')
    parser.add_argument('--include_all_points', default=True, action=argparse.BooleanOptionalAction,
                       help='Include all points from aligned map (default: True, with black for non-visible points). Use --no-include_all_points to only save colored points.')
    
    args = parser.parse_args()
    
    colorize_pointcloud(args.data_dir, args.calib_file, args.results_dir,
                       args.output, args.camera, args.saturation_boost, args.gamma,
                       args.include_all_points, args.use_both_cameras, args.secondary_calib_file)

