import cv2
import os
import glob

def create_video_with_repeated_frames(image_folder, output_filename, target_duration=30.0, fps=30.0):
    # Retrieve all .jpg images from the specified directory
    search_path = os.path.join(image_folder, '*.png')
    image_files = glob.glob(search_path)
    
    if not image_files:
        print(f"No images found in {image_folder}")
        return

    # Sort files chronologically
    try:
        image_files.sort(key=lambda x: int(os.path.splitext(os.path.basename(x))[0]))
    except ValueError:
        print("Error: Ensure all image filenames are purely numerical timestamps.")
        return

    num_images = len(image_files)
    total_frames_needed = int(target_duration * fps)
    
    print(f"Found {num_images} images.")
    print(f"Target duration: {target_duration}s at {fps} FPS.")
    print(f"Total frames to generate: {total_frames_needed}")

    # Read the first frame to determine the video resolution
    first_frame = cv2.imread(image_files[0])
    if first_frame is None:
        print(f"Error reading the first image: {image_files[0]}")
        return
        
    height, width, _ = first_frame.shape
    resolution = (width, height)

    # Initialize the VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
    video_writer = cv2.VideoWriter(output_filename, fourcc, fps, resolution)

    print("Stitching and duplicating images into video...")
    
    frames_written = 0
    # Process each image
    for i, img_path in enumerate(image_files):
        frame = cv2.imread(img_path)
        if frame is None:
            print(f"Warning: Skipping unreadable file {img_path}")
            continue
            
        # Calculate exactly how many frames we should have written by the end of this specific image.
        # This approach prevents rounding errors from slightly shortening or lengthening the video.
        target_frames_by_now = int(((i + 1) / num_images) * total_frames_needed)
        frames_for_this_image = target_frames_by_now - frames_written
        
        # Write the same frame multiple times
        for _ in range(frames_for_this_image):
            video_writer.write(frame)
            frames_written += 1

    # Release resources
    video_writer.release()
    print(f"Success! Saved as: {output_filename}. Total frames written: {frames_written}")

# --- Execution ---
if __name__ == "__main__":
    DIRECTORY = 'images/' 
    OUTPUT_FILE = 'inputs/output_30s_30fps.mp4'
    
    # You can now control BOTH duration and FPS independently
    DURATION = 30.0
    FPS = 30.0

    create_video_with_repeated_frames(DIRECTORY, OUTPUT_FILE, DURATION, FPS)