from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2
import time

# Load YOLOv8 TensorRT model
model = YOLO("yolov8n.engine")

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 60)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# ✅ Open RealSense device before starting pipeline
device = rs.context().devices[0]  # Get the first connected RealSense device
depth_sensor = device.first_depth_sensor()

# ✅ Set sync mode BEFORE starting streaming
if depth_sensor.supports(rs.option.inter_cam_sync_mode):
    depth_sensor.set_option(rs.option.inter_cam_sync_mode, 1)  # Enable hardware sync

# Start the RealSense pipeline
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert color frame to numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Run YOLOv8 inference
        results = model.track(color_image, tracker="bytetrack.yaml", persist=True, device=0, half=True)

        # Process YOLOv8 detections
        for r in results:
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                confidence = box.conf[0].item()  # Confidence score
                class_id = int(box.cls[0].item())  # Class ID

                # ✅ Fix: Ensure box.id is not None before accessing it
                track_id = box.id
                if track_id is not None:
                    track_id = int(track_id.item())  # Convert to integer if valid
                else:
                    track_id = -1  # Assign -1 if no tracking ID

                # Get the center of the detected object
                center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                distance = depth_frame.get_distance(center_x, center_y)
                
                # Get camera intrinsics (once is enough, put this outside the loop)
                if 'intrinsics' not in globals():
                    intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

                # Convert (u, v, depth) to 3D point [X, Y, Z]
                current_point = rs.rs2_deproject_pixel_to_point(intrinsics, [center_x, center_y], distance)

                now = time.time()

                if 'prev_point' in globals() and 'prev_time' in globals():
                    dt = now - prev_time
                    dx = current_point[0] - prev_point[0]
                    dz = current_point[2] - prev_point[2]

                    linear_velocity = np.sqrt(dx**2 + dz**2) / dt
                    angle_now = np.arctan2(current_point[0], current_point[2])
                    angle_prev = np.arctan2(prev_point[0], prev_point[2])
                    angular_velocity = (angle_now - angle_prev) / dt

                    print(f"v = {linear_velocity:.2f} m/s, ω = {angular_velocity:.2f} rad/s")

                # Update previous values
                prev_point = current_point
                prev_time = now


                # Draw bounding box with tracking ID
                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # ✅ Fix: Only display tracking ID if available
                if track_id != -1:
                    label = f"ID {track_id}: {model.names[class_id]} {confidence:.2f}, {distance:.2f}m"
                else:
                    label = f"{model.names[class_id]} {confidence:.2f}, {distance:.2f}m"

                cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        # Show real-time tracking
        cv2.imshow("YOLOv8 Tracking + RealSense", color_image)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
