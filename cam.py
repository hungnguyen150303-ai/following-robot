#!/usr/bin/env python3
"""
RealSense D435 Camera Test Script
Test and visualize RGB and Depth streams from Intel RealSense D435
"""

import os
os.environ['QT_QPA_PLATFORM'] = 'xcb'  # Force X11 instead of Wayland

import pyrealsense2 as rs
import numpy as np
import cv2
import sys

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    
    print("=" * 60)
    print("Intel RealSense Camera Detection")
    print("=" * 60)
    print(f"Device Name: {device.get_info(rs.camera_info.name)}")
    print(f"Serial Number: {device.get_info(rs.camera_info.serial_number)}")
    print(f"Firmware Version: {device.get_info(rs.camera_info.firmware_version)}")
    print("=" * 60)
    
    # Enable streams with lower resolution and frame rate for stability
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    
    # Start streaming
    print("\nStarting camera streams...")
    profile = pipeline.start(config)
    
    # Get depth sensor and set some options
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"Depth Scale: {depth_scale}")
    
    # Create alignment object to align depth frames to color frames
    align_to = rs.stream.color
    align = rs.align(align_to)
    
    # Create colorizer for depth visualization
    colorizer = rs.colorizer()
    
    print("\nWarming up camera... (waiting for first frame)")
    print("This may take a few seconds...")
    
    # Wait for first frames with longer timeout
    import time
    for i in range(30):
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            if frames:
                print("✓ Camera warmed up successfully!")
                break
        except RuntimeError:
            print(f"Waiting... {i+1}/30")
            time.sleep(0.1)
    else:
        raise RuntimeError("Camera failed to warm up after 30 attempts")
    
    print("\nCamera is running!")
    print("Press 'q' to quit")
    print("Press 's' to save snapshot")
    print("=" * 60)
    
    try:
        frame_count = 0
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames(timeout_ms=10000)
            frame_count += 1
            
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)
            
            # Get aligned frames
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # Get center pixel distance
            height, width = depth_image.shape
            center_x, center_y = width // 2, height // 2
            center_distance = depth_frame.get_distance(center_x, center_y)
            
            # Add text overlay to color image
            cv2.putText(color_image, f"Frame: {frame_count}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(color_image, f"Center Dist: {center_distance:.3f}m", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Draw crosshair at center
            cv2.line(color_image, (center_x - 20, center_y), (center_x + 20, center_y), (0, 255, 0), 2)
            cv2.line(color_image, (center_x, center_y - 20), (center_x, center_y + 20), (0, 255, 0), 2)
            
            # Add text overlay to depth colormap
            cv2.putText(depth_colormap, f"Depth Map (JET)", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # Stack images horizontally
            images = np.hstack((color_image, depth_colormap))
            
            # Show images
            cv2.namedWindow('RealSense D435 - RGB + Depth', cv2.WINDOW_AUTOSIZE)
            cv2.imshow('RealSense D435 - RGB + Depth', images)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('s'):
                # Save snapshot
                filename_rgb = f"snapshot_rgb_{frame_count}.png"
                filename_depth = f"snapshot_depth_{frame_count}.png"
                cv2.imwrite(filename_rgb, color_image)
                cv2.imwrite(filename_depth, depth_colormap)
                print(f"Saved: {filename_rgb} and {filename_depth}")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Stop streaming
        print("\nStopping camera...")
        pipeline.stop()
        cv2.destroyAllWindows()
        print("Camera stopped successfully")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\nFailed to initialize camera: {e}")
        print("\nPossible issues:")
        print("1. Camera not connected (check with 'lsusb')")
        print("2. USB permissions (check udev rules)")
        print("3. RealSense SDK not installed properly")
        print("4. Camera in use by another process")
        sys.exit(1)
