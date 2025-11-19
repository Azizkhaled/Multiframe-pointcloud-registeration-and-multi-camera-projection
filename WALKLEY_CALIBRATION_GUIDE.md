

# Walkley Dataset Calibration Guide

Complete guide to calibrate your LiDAR and camera sensors for the Walkley dataset.

## 📊 Your Dataset Summary

- **226 synchronized frames**
- **Point clouds**: ~169,000 points per frame (PCD format)
- **Primary camera**: 1920×1200 RGB images
- **Secondary camera**: 1920×1200 RGB images  
- **LiDAR range**: ~30m average, up to 200m
- **Status**: ❌ No calibration available

## 🎯 Calibration Goal

Get two transformations:
1. **LiDAR → Primary Camera**: Rotation (R) + Translation (t)
2. **LiDAR → Secondary Camera**: Rotation (R) + Translation (t)

Plus camera intrinsics (K) for both cameras.

---

## 🛠️ OPTION 1: Quick Manual Calibration (Fastest - Start Here!)

### What You Need:
- 5-10 minutes
- One good frame with clear overlap between LiDAR and camera view

### Steps:

```bash
conda activate regis2D_3D
cd C:\Users\Abdul-AzizAlNajjar\CU\EEPNet

# Calibrate primary camera
python calibrate_walkley_manual.py --camera primary --frame 0

# Calibrate secondary camera  
python calibrate_walkley_manual.py --camera secondary --frame 0
```

### Controls:
- **W/S**: Move forward/backward
- **A/D**: Move left/right
- **Q/E**: Move up/down
- **I/K**: Pitch rotation
- **J/L**: Yaw rotation
- **U/O**: Roll rotation
- **+/-**: Adjust focal length
- **1/2**: Small/large steps
- **SPACE**: Save calibration
- **ESC**: Quit

### Tips:
1. Start by adjusting focal length (+/-) until points are roughly the right scale
2. Use translation (W/A/S/D/Q/E) to move cloud to image
3. Fine-tune rotation (I/J/K/L/U/O) to align features
4. Use small steps (press 1) for final adjustment
5. Press SPACE to save when satisfied

### Output:
- `walkley_primary_calibration.txt`
- `walkley_secondary_calibration.txt`

---

## 🛠️ OPTION 2: Use EEPNet-Style Automatic Calibration

### Prerequisites:
- Manual calibration from Option 1 (as initial guess)
- ~20-50 frames for optimization

### Concept:
Use the pre-trained EEPNet network to extract and match features, then optimize the calibration parameters to maximize matching quality.

### Steps:

**Coming soon** - I can create this if you want!

The approach would be:
1. Load your manual calibration as initial guess
2. For each frame, project LiDAR with current calibration
3. Extract features using EEPNet
4. Compute matching quality
5. Optimize calibration to maximize matches
6. Iterate until convergence

---

## 🛠️ OPTION 3: Traditional Checkerboard Calibration

### What You Need:
- Large checkerboard (printable, ~1m x 1m)
- Mount it where both LiDAR and cameras can see it
- Take 10-20 images with checkerboard in different positions

### Why It's Best:
- Most accurate
- Standard approach
- Well-tested

### Why It's Hard:
- Requires physical setup
- Need to print large checkerboard
- Must manually place in scene

### Tools:
- MATLAB Camera Calibrator Toolbox
- OpenCV calibration
- ROS camera_calibration package

---

## 🛠️ OPTION 4: Use Existing Calibration Software

### Option 4A: OpenCalib
- Website: https://github.com/OpenCalib/LiDAR2camera
- Automatic calibration using edges/lines
- Requires clear geometric features

### Option 4B: Autoware Calibration Toolkit
- Part of Autoware autonomous driving stack
- Interactive calibration tool
- Good for automotive applications

### Option 4C: ACSC Toolbox
- Semi-automatic calibration
- Uses checkerboard or natural features

---

## 📝 After Calibration

### Verify Your Calibration:

```bash
# Visualize overlay on all frames
python create_pointcloud_overlay_video.py \
    --data_dir "C:\Users\Abdul-AzizAlNajjar\CU\aligning_points\data\walkley\walkley data" \
    --calibration_file walkley_primary_calibration.txt \
    --output_dir walkley_overlay_primary

# Create video
python create_video_from_frames.py \
    --input_dir walkley_overlay_primary \
    --output walkley_overlay.mp4
```

### Save Calibration in Standard Format:

Create `walkley_calib.yaml`:
```yaml
primary_camera:
  intrinsics:
    fx: 1000.0
    fy: 1000.0
    cx: 960.0
    cy: 600.0
  extrinsics:
    rotation: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
    translation: [tx, ty, tz]

secondary_camera:
  intrinsics:
    fx: 1000.0
    fy: 1000.0
    cx: 960.0
    cy: 600.0
  extrinsics:
    rotation: [r11, r12, r13, r21, r22, r23, r31, r32, r33]
    translation: [tx, ty, tz]
```

---

## 🎓 Understanding Calibration Parameters

### Camera Intrinsics (K)
```
K = [fx  0  cx]
    [0  fy  cy]
    [0  0   1 ]
```
- `fx, fy`: Focal lengths (in pixels)
- `cx, cy`: Principal point (image center)
- Usually: `fx ≈ fy`, `cx = width/2`, `cy = height/2`

### Extrinsics (R, t)
- **R**: 3×3 rotation matrix
- **t**: 3×1 translation vector
- Transforms point from LiDAR frame to camera frame:
  ```
  point_camera = R × point_lidar + t
  ```

### Typical Values for Automotive Setup:
- **Translation**: Camera is usually 0.5-2m forward, 0-0.5m up from LiDAR
- **Rotation**: Camera looks forward, maybe slightly down (~5-10°)

---

## 🔍 Quality Metrics

### Good Calibration Should Have:
1. **Road surface aligned** - LiDAR ground points project to road in image
2. **Vertical structures aligned** - Poles, buildings match
3. **Consistent across frames** - Works on all frames, not just one
4. **No obvious offset** - Points overlay correctly, not shifted

### Poor Calibration Looks Like:
- Points floating above/below surfaces
- Systematic shift left/right
- Points projecting behind camera (none visible)
- Different alignment in different parts of image

---

## 🚨 Common Issues & Solutions

### Issue: No points visible
- **Cause**: Camera is behind LiDAR, or translation too large
- **Fix**: Reset translation to zero, adjust rotation first

### Issue: Points all in wrong place
- **Cause**: Rotation is way off
- **Fix**: Start with identity rotation, adjust one axis at a time

### Issue: Some frames good, others bad
- **Cause**: Sensors not truly synchronized, or moving platform
- **Fix**: Use multiple frames for calibration, average results

### Issue: Scale is wrong (points too big/small)
- **Cause**: Focal length incorrect
- **Fix**: Adjust focal length with +/- keys

---

## 📈 Next Steps After Calibration

### 1. Create Colored Point Clouds
```bash
python save_colored_pointclouds_walkley.py \
    --calibration walkley_primary_calibration.txt
```

### 2. Create Visualization Videos
```bash
python create_overlay_video_walkley.py
```

### 3. Use for Downstream Tasks
- 3D object detection
- Semantic segmentation
- SLAM/Mapping
- Scene understanding

---

## 💡 Pro Tips

1. **Start Simple**: Use manual calibration first, it's fast and intuitive
2. **Use Good Frames**: Pick frames with clear geometric features
3. **Multiple Frames**: Verify on 5-10 different frames
4. **Save Intermediate**: Save calibration at each step
5. **Document Setup**: Note sensor positions, mounting, etc.

---

## 📚 Additional Resources

### Papers:
- EEPNet (this repo)
- "Automatic Targetless Extrinsic Calibration"
- "RegNet: Multimodal Sensor Registration"

### Tools:
- [Autoware](https://www.autoware.org/)
- [OpenCalib](https://github.com/OpenCalib)
- [ACSC Toolbox](https://github.com/UMich-CURLY/ACSC)

### Tutorials:
- [ROS Camera Calibration](http://wiki.ros.org/camera_calibration)
- [OpenCV Calibration Tutorial](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)

---

## 🆘 Need Help?

If you're stuck:
1. Check if sensors are actually time-synchronized
2. Verify data quality (are images clear? Is LiDAR working?)
3. Try different frames (some might be better than others)
4. Start with easier camera first (primary vs secondary)
5. Consider traditional checkerboard method if automatic fails

Good luck! 🚀

