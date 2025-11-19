import cv2
import os
import argparse
from pathlib import Path
import imageio

def create_video_from_frames(input_dir, output_path, fps=10):
    """
    Create a video from saved frame images
    """
    # Get all frame files
    frame_files = sorted([f for f in os.listdir(input_dir) if f.endswith('.png')])
    
    if not frame_files:
        print(f"No PNG files found in {input_dir}")
        return
    
    print(f"Found {len(frame_files)} frames")
    
    # Read first image to get dimensions
    first_frame_path = os.path.join(input_dir, frame_files[0])
    first_frame = cv2.imread(first_frame_path)
    height, width, _ = first_frame.shape
    
    # Determine output format
    output_ext = Path(output_path).suffix.lower()
    
    if output_ext == '.gif':
        # Create GIF
        print(f"Creating GIF: {output_path}")
        images = []
        for i, frame_file in enumerate(frame_files):
            if i % 10 == 0:  # Progress update
                print(f"Processing frame {i+1}/{len(frame_files)}")
            frame_path = os.path.join(input_dir, frame_file)
            images.append(imageio.imread(frame_path))
        
        imageio.mimsave(output_path, images, fps=fps)
        print(f"GIF saved successfully!")
        
    else:
        # Create video (MP4, AVI, etc.)
        print(f"Creating video: {output_path}")
        
        # Define codec
        if output_ext == '.mp4':
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        elif output_ext == '.avi':
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
        else:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            output_path = output_path.rsplit('.', 1)[0] + '.mp4'
            print(f"Using MP4 format: {output_path}")
        
        # Create video writer
        out = cv2.VideoWriter(output_path, fourcc, fps, (width, height))
        
        for i, frame_file in enumerate(frame_files):
            if i % 10 == 0:  # Progress update
                print(f"Processing frame {i+1}/{len(frame_files)}")
            frame_path = os.path.join(input_dir, frame_file)
            frame = cv2.imread(frame_path)
            out.write(frame)
        
        out.release()
        print(f"Video saved successfully!")
    
    print(f"Output file: {output_path}")
    print(f"Duration: {len(frame_files)/fps:.1f} seconds at {fps} FPS")

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Create video or GIF from saved test frames')
    parser.add_argument('--input_dir', type=str, default='./test_visualizations',
                        help='Directory containing the frame images')
    parser.add_argument('--output', type=str, default='./test_results.mp4',
                        help='Output video file path (.mp4, .avi, or .gif)')
    parser.add_argument('--fps', type=int, default=10,
                        help='Frames per second for the output video/gif')
    
    args = parser.parse_args()
    
    create_video_from_frames(args.input_dir, args.output, args.fps)

