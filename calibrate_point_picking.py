"""
Point-Picking Camera-LiDAR Calibration Tool

Pick corresponding points between image and point cloud to compute calibration.
Each point pair is shown in a unique color and persists after picking.
"""

import numpy as np
import cv2
import open3d as o3d
from pathlib import Path
import argparse
from scipy.spatial.transform import Rotation


def read_pcd_file(filename):
    """Read PCD file and extract XYZ points and intensity"""
    points = []
    intensities = []
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
                        # Try to read intensity if available (4th column)
                        if len(parts) >= 4:
                            intensities.append(float(parts[3]))
                        else:
                            intensities.append(0.5)
                    except:
                        continue
    return np.array(points), np.array(intensities)


def intensity_to_color(intensities, saturation=1.0):
    """Convert intensity values to high saturation RGB colors using a turbo-like colormap"""
    # Normalize intensities to 0-1 range
    intensities = np.array(intensities)
    if intensities.max() > intensities.min():
        normalized = (intensities - intensities.min()) / (intensities.max() - intensities.min())
    else:
        normalized = np.ones_like(intensities) * 0.5
    
    # Create high saturation colormap (similar to turbo/jet but more saturated)
    # Use HSV color space for better saturation control
    hue = (1.0 - normalized) * 0.75  # Map to blue->cyan->green->yellow->red
    sat = np.ones_like(normalized) * saturation
    val = np.ones_like(normalized)
    
    # Convert HSV to RGB
    colors = np.zeros((len(intensities), 3))
    for i in range(len(intensities)):
        h, s, v = hue[i], sat[i], val[i]
        # HSV to RGB conversion
        c = v * s
        x = c * (1 - abs((h * 6) % 2 - 1))
        m = v - c
        
        if h < 1/6:
            r, g, b = c, x, 0
        elif h < 2/6:
            r, g, b = x, c, 0
        elif h < 3/6:
            r, g, b = 0, c, x
        elif h < 4/6:
            r, g, b = 0, x, c
        elif h < 5/6:
            r, g, b = x, 0, c
        else:
            r, g, b = c, 0, x
        
        colors[i] = [r + m, g + m, b + m]
    
    return colors


def load_calibration(calib_file):
    """Load existing calibration from file"""
    with open(calib_file, 'r') as f:
        lines = f.readlines()
    
    # Parse K
    K = np.array([
        [float(x) for x in lines[4].strip().split()],
        [float(x) for x in lines[5].strip().split()],
        [float(x) for x in lines[6].strip().split()]
    ])
    
    # Parse R
    R = np.array([
        [float(x) for x in lines[9].strip().split()],
        [float(x) for x in lines[10].strip().split()],
        [float(x) for x in lines[11].strip().split()]
    ])
    
    # Parse t
    t = np.array([float(x) for x in lines[14].strip().split()])
    
    return K, R, t


def save_calibration(K, R, t, filename, camera, frame_idx):
    """Save calibration parameters"""
    rot = Rotation.from_matrix(R)
    euler = rot.as_euler('xyz')
    
    with open(filename, 'w') as f:
        f.write(f"# Walkley Dataset Calibration - {camera} camera\n")
        f.write(f"# Frame used: {frame_idx}\n\n")
        
        f.write("# Camera Intrinsics (K)\n")
        f.write(f"{K[0,0]} {K[0,1]} {K[0,2]}\n")
        f.write(f"{K[1,0]} {K[1,1]} {K[1,2]}\n")
        f.write(f"{K[2,0]} {K[2,1]} {K[2,2]}\n\n")
        
        f.write("# Rotation Matrix (R)\n")
        for row in R:
            f.write(f"{row[0]} {row[1]} {row[2]}\n")
        f.write("\n")
        
        f.write("# Translation Vector (t)\n")
        f.write(f"{t[0]} {t[1]} {t[2]}\n\n")
        
        f.write("# Euler Angles (degrees)\n")
        f.write(f"Roll: {np.degrees(euler[0])}\n")
        f.write(f"Pitch: {np.degrees(euler[1])}\n")
        f.write(f"Yaw: {np.degrees(euler[2])}\n")
    
    print(f"\n✅ Calibration saved to: {filename}")


class PointPickingCalibrator:
    def __init__(self, data_dir, frame_idx, camera, initial_calib_file, use_checkerboard=False):
        self.data_path = Path(data_dir)
        self.frame_idx = frame_idx
        self.camera = camera
        self.use_checkerboard = use_checkerboard
        
        # Load data based on mode
        if use_checkerboard:
            # Use checkerboard calibration data
            pc_dir = self.data_path.parent / "calibration" / "PointClouds"
            img_dir = self.data_path.parent / "calibration" / f"{camera}Images"
            
            self.pc_files = sorted(list(pc_dir.glob("*.pcd")))
            self.img_files = sorted(list(img_dir.glob("*.png")))
            
            if frame_idx >= len(self.pc_files):
                frame_idx = 0
                self.frame_idx = 0
            
            pc_file = self.pc_files[frame_idx]
            img_file = self.img_files[frame_idx]
            print(f"📐 Using CHECKERBOARD data: {img_file.name}")
        else:
            # Use regular dataset
            pc_file = self.data_path / "PointCloudsIntensity" / f"{frame_idx:05d}.pcd"
            img_file = self.data_path / f"{camera}Images" / f"{frame_idx:05d}.png"
            self.pc_files = None
            self.img_files = None
        
        self.points_3d, self.intensities = read_pcd_file(pc_file)
        self.image = cv2.imread(str(img_file))
        
        if self.image is None:
            raise ValueError(f"Could not load image: {img_file}")
        
        self.h, self.w = self.image.shape[:2]
        
        # Load initial calibration
        if Path(initial_calib_file).exists():
            self.K, self.R, self.t = load_calibration(initial_calib_file)
            print(f"✅ Loaded initial calibration from {initial_calib_file}")
        else:
            print(f"⚠️  Initial calibration not found, using defaults")
            self.K = np.array([
                [2000, 0, self.w/2],
                [0, 2000, self.h/2],
                [0, 0, 1]
            ])
            self.R = np.eye(3)
            self.t = np.array([0, 0, 0])
        
        # Point correspondences
        self.image_points = []
        self.pointcloud_points = []
        
        # UI state
        self.waiting_for = 'pointcloud'  # 'pointcloud' or 'image'
        
        # Colors for point pairs
        self.colors = [
            [1, 0, 0],  # Red
            [0, 1, 0],  # Green
            [0, 0, 1],  # Blue
            [1, 1, 0],  # Yellow
            [1, 0, 1],  # Magenta
            [0, 1, 1],  # Cyan
            [1, 0.5, 0],  # Orange
            [0.5, 0, 1],  # Purple
            [0, 1, 0.5],  # Spring green
            [1, 0, 0.5],  # Rose
        ]
        
        print(f"\n📷 Image: {self.w}x{self.h}")
        print(f"☁️  Point cloud: {len(self.points_3d)} points")
        print(f"🌈 Intensity range: {self.intensities.min():.2f} to {self.intensities.max():.2f}")
        print(f"   (Colored with high saturation: blue=low, red=high)")
        
        if use_checkerboard:
            print(f"📐 Mode: CHECKERBOARD (frame {self.frame_idx+1} of {len(self.pc_files)})")
            print(f"   Tip: Pick checkerboard corners for best accuracy!")
        
        print("\n" + "="*70)
        print("INSTRUCTIONS:")
        print("="*70)
        print("1. Press 'P' to pick a point in the POINT CLOUD")
        print("   - A new window will open for point cloud")
        print("   - Hold SHIFT and LEFT-CLICK on a point")
        print("   - Close the window when done (ESC)")
        print("2. Then CLICK on the corresponding point in the IMAGE window")
        print("3. Repeat to add more point pairs (different colors)")
        print("4. Press 'C' to COMPUTE calibration (need at least 4 pairs)")
        print("5. Press 'U' to UNDO last pair")
        print("6. Press 'R' to RESET all points")
        print("7. Press 'S' to SAVE calibration")
        print("8. Press 'Q' to QUIT")
        print("="*70)
    
    def get_color_for_pair(self, idx):
        """Get color for point pair index"""
        return self.colors[idx % len(self.colors)]
    
    def update_image_display(self):
        """Update image window with picked points"""
        img_display = self.image.copy()
        
        # Draw picked points
        for i, pt in enumerate(self.image_points):
            color_bgr = (np.array(self.get_color_for_pair(i)[::-1]) * 255).astype(int).tolist()
            cv2.circle(img_display, tuple(pt.astype(int)), 8, color_bgr, -1)
            cv2.circle(img_display, tuple(pt.astype(int)), 10, (255, 255, 255), 2)
            cv2.putText(img_display, str(i+1), (int(pt[0]+15), int(pt[1]-15)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_bgr, 2)
        
        # Show instructions
        if self.waiting_for == 'pointcloud':
            status = "Press 'P' to pick point in POINT CLOUD"
            color = (0, 255, 255)
        else:
            status = "CLICK on corresponding point in this image"
            color = (0, 255, 0)
        
        cv2.putText(img_display, status, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(img_display, f"Pairs: {len(self.image_points)}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow("Image - Pick corresponding points", img_display)
    
    def create_pointcloud_with_markers(self):
        """Create Open3D point cloud with picked point markers"""
        # Create main point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.points_3d)
        
        # Color points by intensity with high saturation
        colors = intensity_to_color(self.intensities, saturation=1.0)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # Create coordinate frame
        coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0, origin=[0, 0, 0])
        
        # Create spheres for picked points
        geometries = [pcd, coord_frame]
        
        for i, pt in enumerate(self.pointcloud_points):
            sphere = o3d.geometry.TriangleMesh.create_sphere(radius=0.15)
            sphere.translate(pt)
            color = self.get_color_for_pair(i)
            sphere.paint_uniform_color(color)
            geometries.append(sphere)
        
        return geometries
    
    def pick_pointcloud_point(self):
        """Open point cloud viewer to pick a point"""
        print("\n🖱️  Point cloud window opened!")
        print("   1. Hold SHIFT and LEFT-CLICK on a point")
        print("   2. Press ESC when done")
        
        # Create geometries
        geometries = self.create_pointcloud_with_markers()
        
        # Use VisualizerWithEditing for point picking
        vis = o3d.visualization.VisualizerWithEditing()
        vis.create_window(window_name="Pick a Point (Shift+Click, then ESC)", width=1280, height=720)
        
        for geom in geometries:
            vis.add_geometry(geom)
        
        # Set viewing options
        opt = vis.get_render_option()
        opt.point_size = 3.0
        opt.background_color = np.array([0.1, 0.1, 0.1])
        
        # Run the picker (blocks until user closes window)
        vis.run()
        
        # Get picked points
        picked_indices = vis.get_picked_points()
        vis.destroy_window()
        
        if len(picked_indices) > 0:
            # Get the first picked point
            picked_idx = picked_indices[0]
            picked_point = self.points_3d[picked_idx]
            
            color_name = ['Red', 'Green', 'Blue', 'Yellow', 'Magenta', 'Cyan', 'Orange', 'Purple', 'Spring green', 'Rose'][len(self.pointcloud_points) % 10]
            print(f"✅ Point {len(self.pointcloud_points)+1} picked: {picked_point} - Color: {color_name}")
            print(f"   Now CLICK on the corresponding point in the IMAGE window...")
            
            self.pointcloud_points.append(picked_point)
            self.waiting_for = 'image'
            return True
        else:
            print("❌ No point picked, try again")
            return False
    
    def image_click_callback(self, event, x, y, flags, param):
        """Callback when user clicks on the image"""
        if event == cv2.EVENT_LBUTTONDOWN and self.waiting_for == 'image':
            # Add the image point
            image_point = np.array([x, y], dtype=np.float32)
            self.image_points.append(image_point)
            
            print(f"✅ Point {len(self.image_points)} picked in image: ({x}, {y})")
            
            self.waiting_for = 'pointcloud'
            
            # Update display
            self.update_image_display()
            
            print(f"\n📍 Total pairs: {len(self.image_points)}")
            if len(self.image_points) >= 4:
                print("   ✅ You can now press 'C' to compute calibration!")
            print("   Press 'P' to pick another point in the point cloud...")
    
    def compute_calibration(self):
        """Compute calibration from point correspondences using PnP"""
        if len(self.image_points) < 4:
            print(f"\n❌ Need at least 4 point pairs (currently have {len(self.image_points)})")
            return False
        
        print(f"\n🔄 Computing calibration from {len(self.image_points)} point pairs...")
        
        # Convert to numpy arrays
        image_pts = np.array(self.image_points, dtype=np.float32)
        pc_pts = np.array(self.pointcloud_points, dtype=np.float32)
        
        # Show point statistics
        print(f"\n📊 Point cloud range:")
        print(f"   X: {pc_pts[:, 0].min():.2f} to {pc_pts[:, 0].max():.2f}")
        print(f"   Y: {pc_pts[:, 1].min():.2f} to {pc_pts[:, 1].max():.2f}")
        print(f"   Z: {pc_pts[:, 2].min():.2f} to {pc_pts[:, 2].max():.2f}")
        
        # Try different PnP methods
        methods = [
            (cv2.SOLVEPNP_ITERATIVE, "ITERATIVE"),
            (cv2.SOLVEPNP_EPNP, "EPNP"),
            (cv2.SOLVEPNP_SQPNP, "SQPNP"),
            (cv2.SOLVEPNP_P3P, "P3P"),
        ]
        
        best_result = None
        best_error = float('inf')
        
        for method, name in methods:
            # Try both with and without initial guess
            for use_guess in [False, True]:
                try:
                    guess_str = "with guess" if use_guess else "no guess"
                    print(f"\n🔧 Trying method: {name} ({guess_str})")
                    
                    if use_guess:
                        # Use initial guess from current calibration
                        rvec_init, _ = cv2.Rodrigues(self.R)
                        tvec_init = self.t.reshape(3, 1)
                    else:
                        rvec_init = None
                        tvec_init = None
                    
                    if len(self.image_points) >= 6 and method != cv2.SOLVEPNP_P3P:
                        # Use RANSAC for robustness if we have enough points
                        if use_guess and rvec_init is not None:
                            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                                pc_pts, image_pts, self.K, None,
                                rvec=rvec_init,
                                tvec=tvec_init,
                                useExtrinsicGuess=True,
                                flags=method,
                                reprojectionError=20.0,
                                confidence=0.99
                            )
                        else:
                            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                                pc_pts, image_pts, self.K, None,
                                flags=method,
                                reprojectionError=20.0,
                                confidence=0.99
                            )
                    else:
                        # Use regular PnP
                        if use_guess and rvec_init is not None:
                            success, rvec, tvec = cv2.solvePnP(
                                pc_pts, image_pts, self.K, None,
                                rvec=rvec_init,
                                tvec=tvec_init,
                                useExtrinsicGuess=True,
                                flags=method
                            )
                        else:
                            success, rvec, tvec = cv2.solvePnP(
                                pc_pts, image_pts, self.K, None,
                                flags=method
                            )
                        inliers = None
                    
                    if not success:
                        print(f"   ❌ Failed")
                        continue
                
                    # Convert rotation vector to matrix
                    R, _ = cv2.Rodrigues(rvec)
                    t = tvec.flatten()
                    
                    # Compute reprojection error
                    errors = []
                    for i, (pc_pt, img_pt) in enumerate(zip(pc_pts, image_pts)):
                        # Transform to camera frame
                        pt_cam = R @ pc_pt + t
                        
                        # Project to image
                        if pt_cam[2] > 0:
                            pt_2d_hom = self.K @ pt_cam
                            pt_2d = pt_2d_hom[:2] / pt_2d_hom[2]
                            error = np.linalg.norm(pt_2d - img_pt)
                            errors.append(error)
                    
                    if len(errors) == 0:
                        print(f"   ❌ No valid projections")
                        continue
                    
                    mean_error = np.mean(errors)
                    print(f"   ✅ Mean error: {mean_error:.2f} px")
                    
                    if mean_error < best_error:
                        best_error = mean_error
                        best_result = (R, t, errors, inliers, f"{name} ({guess_str})")
                
                except Exception as e:
                    print(f"   ❌ Exception: {e}")
                    continue
        
        if best_result is None:
            print("\n❌ All PnP methods failed! Check your point correspondences.")
            print("   Tips:")
            print("   - Make sure points are spread across the image")
            print("   - Verify the 3D-2D correspondences are correct")
            print("   - Try picking points on distinct features")
            return False
        
        # Use best result
        R, t, errors, inliers, method_name = best_result
        
        # Update calibration
        self.R = R
        self.t = t
        
        print(f"\n✅ Best method: {method_name}")
        print(f"\n📊 Reprojection errors:")
        
        image_pts = np.array(self.image_points, dtype=np.float32)
        pc_pts = np.array(self.pointcloud_points, dtype=np.float32)
        
        for i, (pc_pt, img_pt) in enumerate(zip(pc_pts, image_pts)):
            pt_cam = R @ pc_pt + t
            if pt_cam[2] > 0:
                pt_2d_hom = self.K @ pt_cam
                pt_2d = pt_2d_hom[:2] / pt_2d_hom[2]
                error = np.linalg.norm(pt_2d - img_pt)
                is_inlier = inliers is not None and i in inliers
                status = '✅' if is_inlier or inliers is None else '❌'
                print(f"   Point {i+1}: error = {error:.2f} px {status}")
        
        print(f"\n📊 Mean reprojection error: {best_error:.2f} pixels")
        
        if inliers is not None:
            print(f"   Inliers: {len(inliers)}/{len(self.image_points)}")
        
        # Print calibration
        rot = Rotation.from_matrix(R)
        euler = rot.as_euler('xyz', degrees=True)
        
        print(f"\n📐 Computed Calibration:")
        print(f"   Translation: tx={t[0]:.3f}, ty={t[1]:.3f}, tz={t[2]:.3f}")
        print(f"   Rotation: roll={euler[0]:.1f}°, pitch={euler[1]:.1f}°, yaw={euler[2]:.1f}°")
        
        return True
    
    def undo_last_pair(self):
        """Remove the last point pair"""
        if len(self.image_points) > 0:
            self.image_points.pop()
            self.pointcloud_points.pop()
            self.waiting_for = 'pointcloud'
            print(f"\n↩️  Undone. Pairs remaining: {len(self.image_points)}")
            self.update_image_display()
    
    def reset_points(self):
        """Clear all picked points"""
        self.image_points = []
        self.pointcloud_points = []
        self.waiting_for = 'pointcloud'
        print("\n🔄 All points cleared")
        self.update_image_display()
    
    def run(self):
        """Run the interactive calibration tool"""
        # Setup image window
        cv2.namedWindow("Image - Pick corresponding points")
        cv2.setMouseCallback("Image - Pick corresponding points", self.image_click_callback)
        self.update_image_display()
        
        print("\n🖱️  Ready! Press 'P' to pick a point in the point cloud...")
        
        # Main loop
        while True:
            key = cv2.waitKey(10) & 0xFF
            
            if key == ord('q'):
                print("\n👋 Exiting...")
                break
            elif key == ord('p') and self.waiting_for == 'pointcloud':
                # Pick point in point cloud
                self.pick_pointcloud_point()
                self.update_image_display()
            elif key == ord('c'):
                self.compute_calibration()
            elif key == ord('u'):
                self.undo_last_pair()
            elif key == ord('r'):
                self.reset_points()
            elif key == ord('s'):
                output_file = f"walkley_{self.camera}_calibration.txt"
                save_calibration(self.K, self.R, self.t, output_file, self.camera, self.frame_idx)
        
        cv2.destroyAllWindows()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Point-Picking Camera-LiDAR Calibration')
    parser.add_argument('--data_dir', type=str,
                       default=r'C:\Users\Abdul-AzizAlNajjar\CU\aligning_points\data\walkley\walkley data',
                       help='Path to Walkley data directory')
    parser.add_argument('--frame', type=int, default=0, help='Frame index to use')
    parser.add_argument('--camera', type=str, default='primary',
                       choices=['primary', 'secondary'],
                       help='Which camera to calibrate')
    parser.add_argument('--initial_calib', type=str, default='walkley_primary_calibration.txt',
                       help='Initial calibration file to load')
    parser.add_argument('--use_checkerboard', action='store_true',
                       help='Use checkerboard calibration data instead of regular dataset')
    
    args = parser.parse_args()
    
    calibrator = PointPickingCalibrator(args.data_dir, args.frame, args.camera, args.initial_calib, args.use_checkerboard)
    calibrator.run()
