from ultralytics import YOLO
import pyrealsense2 as rs
import numpy as np
import cv2
import time
import serial

### PID MOD ###
class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.last_time = time.time()

    def compute(self, measurement):
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.01

        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.last_time = current_time
        return output

# Load your custom YOLOv8 model
model = YOLO("best.engine")

# Configure RealSense
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Setup UART communication
ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1, write_timeout=1)
time.sleep(2)  # Let STM32 boot
ser.reset_input_buffer()
ser.reset_output_buffer()

# Sync RealSense
device = rs.context().devices[0]
depth_sensor = device.first_depth_sensor()
if depth_sensor.supports(rs.option.inter_cam_sync_mode):
    depth_sensor.set_option(rs.option.inter_cam_sync_mode, 1)

pipeline.start(config)

intrinsics = None
last_send_time = 0
send_interval = 0.05  # 50 ms
locked_id = None
last_seen_time = time.time()
relock_timeout = 2.0

# === PID MOD === 初始化 PID 控制器
pid = PIDController(kp=0.01, ki=0.0, kd=0.005, setpoint=320)

def send_velocity(v, w):
    message = f"{v:.2f},{w:.2f}\n"
    print(f"Sending: {message.strip()}")
    ser.write(message.encode())

# 初始化上一次的速度
v_last = 0.0
w_last = 0.0

# 速度限制与平滑参数
MAX_V = 0.8
MAX_W = 0.6
MAX_ACC_V = 0.05
MAX_ACC_W = 0.05

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        color_image = np.asanyarray(color_frame.get_data())
        
        # results = model.track(color_image, tracker="bytetrack.yaml", persist=True, device="cpu", half=False, conf=0.25)
        results = model.track(color_image, tracker="bytetrack.yaml", persist=True, device=0, half=True)

        if intrinsics is None:
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        current_time = time.time()
        v, w = 0.0, 0.0
        detected_this_frame = False

        for r in results:
            for box in r.boxes:
                if box.id is None:
                    continue
                track_id = int(box.id.item())

                if locked_id is None:
                    locked_id = track_id

                if track_id != locked_id:
                    continue  # Skip other detections

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                distance = depth_frame.get_distance(cx, cy)

                if distance < 0.1 or distance > 10.0:
                    continue

                current_point = rs.rs2_deproject_pixel_to_point(intrinsics, [cx, cy], distance)

                # === PID MOD === 保持线速度逻辑不变
                # ### 距离保持设置在1.5米 ###
                # if distance > 1.5:
                #     v = distance - 1.5
                # else:
                #     v = 0.0
                
                ### 保持在 1 米前后缓冲
                v = distance - 1

                
                # PID 控制角速度
                w = pid.compute(cx)

                # === 加入速度限制与平滑处理 ===
                ### 最大倒车速度是0.4 m/s
                v = max(min(v, MAX_V), -0.4)
                # v = max(min(v, MAX_V), -MAX_V)
                w = max(min(w, MAX_W), -MAX_W)

                dv = v - v_last
                dw = w - w_last
                dv = np.clip(dv, -MAX_ACC_V, MAX_ACC_V)
                dw = np.clip(dw, -MAX_ACC_W, MAX_ACC_W)
                v = v_last + dv
                w = w_last + dw

                v_last = v
                w_last = w
                
                label = f"v={v:.2f}, w={w:.2f} (cx={cx})"  # === PID MOD === 输出中心坐标
                cv2.putText(color_image, label, (x1, y1 - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                last_seen_time = current_time
                detected_this_frame = True

                cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                break  # Only use first locked target
        
        #### 目标丢失超过 relock_timeout 就会清空locked_id
        #### 每一帧如果没检测到目标，v, w 会保留为 0
        # # Reset lock if timeout
        # if not detected_this_frame and current_time - last_seen_time > relock_timeout:
        #     locked_id = None
        
        # === 添加丢失后缓冲期（保持上一速度 0.3 秒）===
        lost_grace_period = 0.3

        if not detected_this_frame:
            time_since_seen = current_time - last_seen_time
            if time_since_seen <= lost_grace_period:
                # 保持上一速度，不更新 v, w
                v = v_last
                w = w_last
            elif time_since_seen > relock_timeout:
                locked_id = None
                v = 0.0
                w = 0.0
                v_last = 0.0
                w_last = 0.0


        # Send to STM32 at fixed interval
        if time.time() - last_send_time >= send_interval:
            send_velocity(v, w)
            last_send_time = time.time()

        cv2.imshow("YOLO + RealSense", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    print("Stopped by user")
finally:
    pipeline.stop()
    ser.close()
    cv2.destroyAllWindows()

