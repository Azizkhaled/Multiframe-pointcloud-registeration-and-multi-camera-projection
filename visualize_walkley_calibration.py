"""
Visualize calibration on Walkley dataset frames
Creates overlay visualization using saved calibration
"""

import numpy as np
import cv2
from pathlib import Path
import argparse

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

def project_points(points_3d, K, R, t):
    """Project 3D points to 2D using camera parameters"""
    # Transform points to camera frame
    points_cam = points_3d @ R.T + t
    
    # Filter points behind camera
    valid = points_cam[:, 2] > 0
    points_cam = points_cam[valid]
    
    if len(points_cam) == 0:
        return np.array([]), np.array([])
    
    # Project to image
    points_2d_hom = points_cam @ K.T
    points_2d = points_2d_hom[:, :2] / points_2d_hom[:, 2:3]
    
    return points_2d, points_cam[:, 2]  # Return 2D points and depths

def depth_to_color(depths, min_d=5, max_d=50):
    """Convert depth to color (blue=close, red=far)"""
    normalized = np.clip((depths - min_d) / (max_d - min_d), 0, 1)
    colors = np.zeros((len(depths), 3), dtype=np.uint8)
    colors[:, 2] = (255 * (1 - normalized)).astype(np.uint8)  # Blue
    colors[:, 0] = (255 * normalized).astype(np.uint8)        # Red
    colors[:, 1] = (128 * (1 - np.abs(normalized - 0.5) * 2)).astype(np.uint8)  # Green
    return colors

def create_overlay(image, points_2d, colors, point_size=2, alpha=0.7):
    """Overlay colored points on image"""
    overlay = image.copy()
    
    # Draw points
    for (x, y), color in zip(points_2d.astype(int), colors):
        if 0 <= x < overlay.shape[1] and 0 <= y < overlay.shape[0]:
            cv2.circle(overlay, (x, y), point_size, color.tolist(), -1)
    
    return overlay

def visualize_calibration(data_dir, calib_file, camera='primary', 
                         output_dir='walkley_calibration_check',
                         max_frames=10, point_size=2):
    """Visualize calibration on dataset frames"""
    
    data_path = Path(data_dir)
    output_path = Path(output_dir)
    output_path.mkdir(exist_ok=True)
    
    # Load calibration
    K, R, t = load_calibration(calib_file)
    print(f"\nLoaded calibration from: {calib_file}")
    print(f"Focal length: {K[0,0]:.1f} pixels")
    print(f"Principal point: ({K[0,2]:.1f}, {K[1,2]:.1f})")
    
    # Get file lists
    pc_dir = data_path / "PointCloudsIntensity"
    img_dir = data_path / f"{camera}Images"
    
    pc_files = sorted(list(pc_dir.glob("*.pcd")))
    img_files = sorted(list(img_dir.glob("*.png")))
    
    num_frames = min(len(pc_files), len(img_files), max_frames if max_frames > 0 else len(pc_files))
    
    print(f"\nProcessing {num_frames} frames...")
    print(f"Output directory: {output_path}")
    
    for i in range(num_frames):
        # Load data
        pc = read_pcd_file(pc_files[i])
        img = cv2.imread(str(img_files[i]))
        
        if img is None:
            print(f"Warning: Could not load image {img_files[i]}")
            continue
        
        h, w = img.shape[:2]
        
        # Project points
        points_2d, depths = project_points(pc, K, R, t)
        
        if len(points_2d) == 0:
            print(f"Warning: No points visible in frame {i+1}")
            continue
        
        # Filter to image bounds
        valid = ((points_2d[:, 0] >= 0) & (points_2d[:, 0] < w) &
                (points_2d[:, 1] >= 0) & (points_2d[:, 1] < h))
        points_2d = points_2d[valid]
        depths = depths[valid]
        
        # Get colors
        colors = depth_to_color(depths)
        
        # Create overlay
        overlay = create_overlay(img, points_2d, colors, point_size)
        
        # Add info text
        info_text = f"Frame {i+1}/{num_frames} | Points: {len(points_2d)} | Camera: {camera}"
        cv2.putText(overlay, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Save
        output_file = output_path / f"calibration_check_{i+1:05d}.png"
        cv2.imwrite(str(output_file), overlay)
        
        if (i + 1) % 10 == 0:
            print(f"  Processed {i+1}/{num_frames} frames...")
    
    print(f"\n✓ Done! Saved {num_frames} visualizations to: {output_path}")
    print(f"\nTo create a video:")
    print(f"  python create_video_from_frames.py --input_dir {output_dir} --output calibration_check.mp4 --fps 5")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Visualize Walkley calibration')
    parser.add_argument('--data_dir', type=str,
                       default=r'C:\Users\Abdul-AzizAlNajjar\CU\aligning_points\data\walkley\walkley data',
                       help='Path to Walkley data directory')
    parser.add_argument('--calib_file', type=str,
                       default='walkley_primary_calibration.txt',
                       help='Calibration file to use')
    parser.add_argument('--camera', type=str, default='primary',
                       choices=['primary', 'secondary'],
                       help='Which camera to visualize')
    parser.add_argument('--output_dir', type=str, default='walkley_calibration_check_3',
                       help='Output directory for visualizations')
    parser.add_argument('--max_frames', type=int, default=100,
                       help='Maximum number of frames to process (0 for all)')
    parser.add_argument('--point_size', type=int, default=3,
                       help='Size of points in overlay')
    
    args = parser.parse_args()
    
    visualize_calibration(args.data_dir, args.calib_file, args.camera,
                         args.output_dir, args.max_frames, args.point_size)

