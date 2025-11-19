# Visualization Guide for EEPNet Testing

This guide explains how to visualize the testing results on a frame-by-frame basis.

## Overview

The visualization system shows:
- **Red dots**: Ground truth 2D keypoints from the camera image
- **Blue dots**: Predicted 2D points from 3D LiDAR projection
- **Green lines**: Matches between ground truth and predicted points
- **Text overlay**: Frame number, registration status (SUCCESS/FAILED), RRE, RTE, and match statistics

## Quick Start

### 1. Run Test with Visualization

```bash
conda activate regis2D_3D
python test_with_visualization.py
```

This will:
- Process all test sequences (09 and 10)
- Save frame visualizations to `./test_visualizations/`
- Generate an Excel file with detailed results

### 2. Run with Custom Options

```bash
# Visualize only first 50 frames, save every 2nd frame
python test_with_visualization.py --max_frames 50 --save_every 2

# Use different output directory
python test_with_visualization.py --output_dir ./my_results

# Run on CPU
python test_with_visualization.py --device cpu
```

### 3. Create Video from Frames

After generating frame visualizations, create a video:

```bash
# Create MP4 video
python create_video_from_frames.py --input_dir ./test_visualizations --output test_results.mp4 --fps 10

# Create GIF (slower but widely compatible)
python create_video_from_frames.py --input_dir ./test_visualizations --output test_results.gif --fps 5

# Create AVI video
python create_video_from_frames.py --input_dir ./test_visualizations --output test_results.avi --fps 15
```

## Command Line Options

### test_with_visualization.py

| Option | Default | Description |
|--------|---------|-------------|
| `--dataset_path` | `./dataset` | Path to test dataset |
| `--device` | `cuda:0` | Device to use (cuda:0, cuda:1, or cpu) |
| `--batch_size` | `1` | Batch size (keep at 1 for visualization) |
| `--output_dir` | `./test_visualizations` | Directory to save visualizations |
| `--save_every` | `1` | Save every Nth frame (use 2-5 to save space) |
| `--max_frames` | `100` | Maximum frames to visualize (0 for all) |

### create_video_from_frames.py

| Option | Default | Description |
|--------|---------|-------------|
| `--input_dir` | `./test_visualizations` | Directory with frame images |
| `--output` | `./test_results.mp4` | Output file path (.mp4, .avi, .gif) |
| `--fps` | `10` | Frames per second |

## Examples

### Example 1: Quick Preview (First 20 Frames)

```bash
python test_with_visualization.py --max_frames 20
python create_video_from_frames.py --output preview.gif --fps 5
```

### Example 2: Full Test with Video

```bash
# Generate all visualizations (will take longer)
python test_with_visualization.py --max_frames 0 --save_every 1

# Create video at 15 FPS
python create_video_from_frames.py --fps 15 --output full_test.mp4
```

### Example 3: Space-Saving Mode

```bash
# Save every 5th frame, process 200 frames max
python test_with_visualization.py --max_frames 200 --save_every 5 --output_dir ./sparse_vis
python create_video_from_frames.py --input_dir ./sparse_vis --output sparse_test.mp4
```

## Output Files

After running the visualization:

1. **Frame Images**: `test_visualizations/frame_XXXXXX.png`
   - Individual frame visualizations
   - Named sequentially (e.g., frame_000000.png, frame_000001.png, ...)

2. **Results Excel**: `test_visualizations/test_results.xlsx`
   - Contains RRE, RTE, success status, and match count for each frame
   - Can be opened in Excel or analyzed with pandas

3. **Video/GIF**: `test_results.mp4` or `test_results.gif`
   - Compiled video from all frames
   - Easy to review all results sequentially

## Understanding the Visualization

### Color Coding
- 🔴 **Red dots**: Ground truth camera keypoints
- 🔵 **Blue dots**: LiDAR points projected to camera view
- 🟢 **Green lines**: Successful feature matches

### Status Colors
- 🟢 **Green text**: Registration successful (RRE and RTE within reasonable bounds)
- 🔴 **Red text**: Registration failed (could not solve PnP problem)

### Metrics
- **RRE**: Relative Rotation Error (degrees) - Lower is better
- **RTE**: Relative Translation Error (meters) - Lower is better
- **Matches**: Number of valid feature matches found
- **Out of bounds**: Number of projected points outside the image

## Tips

1. **Storage**: Full visualization of ~2800 frames takes ~1-2 GB
   - Use `--save_every 5` to reduce to ~400 MB
   - Use `--max_frames 100` for quick tests

2. **Speed**: 
   - GPU mode: ~10-15 frames/second
   - CPU mode: ~2-5 frames/second
   - Video creation: ~50-100 frames/second

3. **Analysis**:
   - Failed frames (red text) often have very few matches
   - Good frames typically have >20 matches
   - Check the Excel file to filter by success rate or error thresholds

4. **Video Quality**:
   - MP4: Best quality, smaller file size
   - AVI: Good quality, larger file size
   - GIF: Lower quality but universal compatibility

## Troubleshooting

**Issue**: "No PNG files found"
- Make sure you ran `test_with_visualization.py` first
- Check the `--output_dir` path matches

**Issue**: Video playback issues
- Try different formats (MP4, AVI, GIF)
- Adjust `--fps` (try 5-30 range)

**Issue**: Out of memory
- Reduce `--batch_size` to 1
- Use `--save_every` to skip frames
- Limit with `--max_frames`

**Issue**: Slow processing
- Use GPU: `--device cuda:0`
- Reduce resolution by editing image sizes in code
- Use `--save_every 5` to skip frames

