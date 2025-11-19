import open3d as o3d
import numpy as np
import glob
from tqdm import tqdm

# Load estimated poses (N x 4 x 4)
poses = np.load(r'results\latest\PointCloudsIntensity_poses.npy')

# List all PCD files - fix path with forward slashes or raw strings
# pcd_files = sorted(glob.glob(r'data\ultmar\ultrmar-hydroOttawa_all\data\ultmar\ultrmar-hydroOttawa_all\PointClouds_and_dublicated\*.pcd'))
pcd_files = sorted(glob.glob(r'C:\Users\Abdul-AzizAlNajjar\CU\aligning_points\data\walkley\walkley data\PointCloudsIntensity\*.pcd'))
# Initialize a global pointcloud (using tensor API to preserve custom fields)
global_pcd = o3d.t.geometry.PointCloud()
first = True

for pose, file_path in tqdm(zip(poses, pcd_files), desc="Loading point clouds", total=len(pcd_files)):
    # Read point cloud with tensor API (reads all fields, including intensity)
    pcd = o3d.t.io.read_point_cloud(file_path)  
    positions = pcd.point["positions"].numpy()          # Nx3 array
    
    # Get intensity values
    if "intensity" in pcd.point:
        intensity = pcd.point["intensity"].numpy()
    elif "scalar_intensity" in pcd.point:
        intensity = pcd.point["scalar_intensity"].numpy()
    else:
        print(f"Warning: No intensity field found in {file_path}")
        intensity = np.ones((positions.shape[0], 1), dtype=np.float32)
    
    # Get scalar_label values
    if "scalar_label" in pcd.point:
        scalar_label = pcd.point["scalar_label"].numpy()
    else:
        # print(f"Warning: No scalar_label field found in {file_path}")
        scalar_label = np.zeros((positions.shape[0], 1), dtype=np.float32)
    
    # Ensure proper shapes (Nx1)
    if len(intensity.shape) == 1:
        intensity = intensity.reshape(-1, 1)
    if len(scalar_label.shape) == 1:
        scalar_label = scalar_label.reshape(-1, 1)
    
    # Apply the 4x4 transform to points
    hom_points = np.hstack([positions, np.ones((positions.shape[0], 1))])  # Nx4
    transformed = (pose @ hom_points.T).T[:, :3]                          # Nx3
    
    # Build a new tensor pointcloud with transformed points & properties
    new_pcd = o3d.t.geometry.PointCloud()
    new_pcd.point["positions"] = o3d.core.Tensor(transformed, dtype=o3d.core.Dtype.Float32)
    new_pcd.point["intensity"] = o3d.core.Tensor(intensity, dtype=o3d.core.Dtype.Float32)
    new_pcd.point["scalar_label"] = o3d.core.Tensor(scalar_label, dtype=o3d.core.Dtype.Float32)
    
    # For the first cloud, assign directly, for others, accumulate
    if first:
        global_pcd = new_pcd
        first = False
    else:
        # Try to safely add point clouds
        global_pcd.point["positions"] = o3d.core.concatenate(
            [global_pcd.point["positions"], new_pcd.point["positions"]], axis=0)
        global_pcd.point["intensity"] = o3d.core.concatenate(
            [global_pcd.point["intensity"], new_pcd.point["intensity"]], axis=0)
        global_pcd.point["scalar_label"] = o3d.core.concatenate(
            [global_pcd.point["scalar_label"], new_pcd.point["scalar_label"]], axis=0)

# Voxel downsample to collapse duplicates and average property values
voxel_size = 0.05  # adjust to taste
global_pcd = global_pcd.voxel_down_sample(voxel_size)
global_pcd, _ = global_pcd.remove_statistical_outliers(nb_neighbors=20, std_ratio=2.0)
# Save the final merged map (PCD preserves all fields)
o3d.t.io.write_point_cloud('results/latest/Walkley_complete_full.pcd', global_pcd)

print("Merged map with intensity and scalar_label saved to 'results/latest/Walkley_complete_full.pcd'")