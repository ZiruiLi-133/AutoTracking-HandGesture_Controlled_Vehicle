import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import time
import numpy as np

# Set COM8 directly as requested
COM_PORT = 'COM8'
print(f"Attempting to connect to {COM_PORT}...")

# Try to open the serial port with exception handling and retries
max_retries = 3
for attempt in range(max_retries):
    try:
        ser = serial.Serial(COM_PORT, 230400, timeout=1)
        print(f"Successfully connected to {COM_PORT}")
        break
    except serial.SerialException as e:
        print(f"Attempt {attempt+1}/{max_retries}: Error opening {COM_PORT}: {e}")
        if attempt < max_retries - 1:
            print("Retrying in 2 seconds...")
            time.sleep(2)
        else:
            print(f"Could not open {COM_PORT} after {max_retries} attempts.")
            print("Try closing any applications that might be using the port.")
            print("You can continue without serial connection for testing (plots will be static).")
            ser = None

# Data storage structures
class MotorData:
    def __init__(self):
        self.times = np.array([])
        self.desired_speeds = np.array([])
        self.real_speeds = np.array([])
        self.duty_cycles = np.array([])
        
    def add_point(self, time, desired, real, duty):
        self.times = np.append(self.times, time)
        self.desired_speeds = np.append(self.desired_speeds, desired)
        self.real_speeds = np.append(self.real_speeds, real)
        self.duty_cycles = np.append(self.duty_cycles, duty)
        
    def trim_data(self, cutoff_time):
        """Remove data points older than cutoff_time"""
        if len(self.times) == 0:
            return
            
        mask = self.times >= cutoff_time
        self.times = self.times[mask]
        self.desired_speeds = self.desired_speeds[mask]
        self.real_speeds = self.real_speeds[mask]
        self.duty_cycles = self.duty_cycles[mask]

# Initialize data structures for each motor
motors = {
    'lb': MotorData(),
    'rb': MotorData(),
    'lf': MotorData(),
    'rf': MotorData()
}

# Global time tracking
start_time = time.time()

# Set up the figure and subplots
fig, axs = plt.subplots(5, 1, figsize=(14, 12), sharex=True)
plt.subplots_adjust(hspace=0.3)

# Simplified regex patterns that focus only on the essential data
lb_pattern = re.compile(r"LB Ideal:\s*([+-]?\d+\.\d+)\s*\|\s*LB Real:\s*([+-]?\d+\.\d+).*?LB Duty Cycle:\s*(\d+)")
rb_pattern = re.compile(r"RB Ideal:\s*([+-]?\d+\.\d+)\s*\|\s*RB Real:\s*([+-]?\d+\.\d+).*?RB Duty Cycle:\s*(\d+)")
lf_pattern = re.compile(r"LF Ideal:\s*([+-]?\d+\.\d+)\s*\|\s*LF Real:\s*([+-]?\d+\.\d+).*?LF Duty Cycle:\s*(\d+)")
rf_pattern = re.compile(r"RF Ideal:\s*([+-]?\d+\.\d+)\s*\|\s*RF Real:\s*([+-]?\d+\.\d+).*?RF Duty Cycle:\s*(\d+)")

# Create lines for each subplot
# LB Speed Plot
lb_desired_line, = axs[0].plot([], [], 'b-', label='LB Desired')
lb_real_line, = axs[0].plot([], [], 'r-', label='LB Real')
axs[0].set_ylabel('LB Speed')
axs[0].set_title('Left Back Motor')
axs[0].legend(loc='upper left')
axs[0].grid(True)

# RB Speed Plot
rb_desired_line, = axs[2].plot([], [], 'b-', label='RB Desired')
rb_real_line, = axs[2].plot([], [], 'r-', label='RB Real')
axs[2].set_ylabel('RB Speed')
axs[2].set_title('Right Back Motor')
axs[2].legend(loc='upper left')
axs[2].grid(True)

# LF Speed Plot
lf_desired_line, = axs[1].plot([], [], 'b-', label='LF Desired')
lf_real_line, = axs[1].plot([], [], 'r-', label='LF Real')
axs[1].set_ylabel('LF Speed')
axs[1].set_title('Left Front Motor')
axs[1].legend(loc='upper left')
axs[1].grid(True)

# RF Speed Plot
rf_desired_line, = axs[3].plot([], [], 'b-', label='RF Desired')
rf_real_line, = axs[3].plot([], [], 'r-', label='RF Real')
axs[3].set_ylabel('RF Speed')
axs[3].set_title('Right Front Motor')
axs[3].legend(loc='upper left')
axs[3].grid(True)

# Duty Cycle Plot (All Motors)
lb_duty_line, = axs[4].plot([], [], 'b-', label='LB Duty')
rb_duty_line, = axs[4].plot([], [], 'r-', label='RB Duty')
lf_duty_line, = axs[4].plot([], [], 'g-', label='LF Duty')
rf_duty_line, = axs[4].plot([], [], 'orange', label='RF Duty')
axs[4].set_ylabel('Duty Cycle')
axs[4].set_xlabel('Time (s)')
axs[4].set_title('All Motors Duty Cycle')
axs[4].legend(loc='upper left')
axs[4].grid(True)

# Create a text object for message display
message_text = fig.text(0.05, 0.95, '', fontsize=8)

# Fixed window size for the plot
window_size = 10  # seconds

# Set initial axis limits
for ax in axs[:4]:  # Speed plots
    ax.set_xlim(0, window_size)
    ax.set_ylim(-2, 2)  # Adjust as needed
    
# Duty cycle plot
axs[4].set_xlim(0, window_size)
axs[4].set_ylim(-10, 1010)  # Duty cycle 0-1000

def init():
    # Initialize with empty data
    lb_desired_line.set_data([], [])
    lb_real_line.set_data([], [])
    rb_desired_line.set_data([], [])
    rb_real_line.set_data([], [])
    lf_desired_line.set_data([], [])
    lf_real_line.set_data([], [])
    rf_desired_line.set_data([], [])
    rf_real_line.set_data([], [])
    
    lb_duty_line.set_data([], [])
    rb_duty_line.set_data([], [])
    lf_duty_line.set_data([], [])
    rf_duty_line.set_data([], [])
    
    message_text.set_text('Initializing...')
    
    return (lb_desired_line, lb_real_line, rb_desired_line, rb_real_line,
            lf_desired_line, lf_real_line, rf_desired_line, rf_real_line,
            lb_duty_line, rb_duty_line, lf_duty_line, rf_duty_line)

def update(frame):
    global start_time
    current_time = time.time() - start_time
    
    # Read data from serial port if available
    if ser is not None:
        # Read up to 5 lines (4 motors + empty line)
        for _ in range(5):
            try:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                print(line)
                if not line:
                    continue
                    
                # Parse the line for each motor
                lb_match = lb_pattern.search(line)
                rb_match = rb_pattern.search(line)
                lf_match = lf_pattern.search(line)
                rf_match = rf_pattern.search(line)
                
                # Process the matches
                if lb_match:
                    lb_ideal = float(lb_match.group(1))
                    lb_real = float(lb_match.group(2))
                    lb_duty = int(lb_match.group(3))
                    motors['lb'].add_point(current_time, lb_ideal, lb_real, lb_duty)
                    message_text.set_text(f"LB - Ideal: {lb_ideal:.2f}, Real: {lb_real:.2f}, Duty: {lb_duty}")
                
                elif rb_match:
                    rb_ideal = float(rb_match.group(1))
                    rb_real = float(rb_match.group(2))
                    rb_duty = int(rb_match.group(3))
                    motors['rb'].add_point(current_time, rb_ideal, rb_real, rb_duty)
                    message_text.set_text(f"RB - Ideal: {rb_ideal:.2f}, Real: {rb_real:.2f}, Duty: {rb_duty}")
                
                elif lf_match:
                    lf_ideal = float(lf_match.group(1))
                    lf_real = float(lf_match.group(2))
                    lf_duty = int(lf_match.group(3))
                    motors['lf'].add_point(current_time, lf_ideal, lf_real, lf_duty)
                    message_text.set_text(f"LF - Ideal: {lf_ideal:.2f}, Real: {lf_real:.2f}, Duty: {lf_duty}")
                
                elif rf_match:
                    rf_ideal = float(rf_match.group(1))
                    rf_real = float(rf_match.group(2))
                    rf_duty = int(rf_match.group(3))
                    motors['rf'].add_point(current_time, rf_ideal, rf_real, rf_duty)
                    message_text.set_text(f"RF - Ideal: {rf_ideal:.2f}, Real: {rf_real:.2f}, Duty: {rf_duty}")
                
            except Exception as e:
                print(f"Error processing line: {e}")
    
    # For testing without serial data, add dummy data points
    if ser is None:
        # Add dummy data for testing the plot
        import math
        for motor in motors.values():
            motor.add_point(
                current_time,
                math.sin(current_time),
                math.sin(current_time + 0.5),
                500 + 400 * math.sin(current_time * 0.5)
            )
    
    # Trim old data points
    cutoff_time = current_time - window_size
    for motor in motors.values():
        motor.trim_data(cutoff_time)
    
    # Update the plot boundaries
    for ax in axs:
        ax.set_xlim(max(0, current_time - window_size), current_time + 0.1)
    
    # Update the line data - safely handling empty arrays
    if len(motors['lb'].times) > 0:
        lb_desired_line.set_data(motors['lb'].times, motors['lb'].desired_speeds)
        lb_real_line.set_data(motors['lb'].times, motors['lb'].real_speeds)
        lb_duty_line.set_data(motors['lb'].times, motors['lb'].duty_cycles)
    
    if len(motors['rb'].times) > 0:
        rb_desired_line.set_data(motors['rb'].times, motors['rb'].desired_speeds)
        rb_real_line.set_data(motors['rb'].times, motors['rb'].real_speeds)
        rb_duty_line.set_data(motors['rb'].times, motors['rb'].duty_cycles)
    
    if len(motors['lf'].times) > 0:
        lf_desired_line.set_data(motors['lf'].times, motors['lf'].desired_speeds)
        lf_real_line.set_data(motors['lf'].times, motors['lf'].real_speeds)
        lf_duty_line.set_data(motors['lf'].times, motors['lf'].duty_cycles)
    
    if len(motors['rf'].times) > 0:
        rf_desired_line.set_data(motors['rf'].times, motors['rf'].desired_speeds)
        rf_real_line.set_data(motors['rf'].times, motors['rf'].real_speeds)
        rf_duty_line.set_data(motors['rf'].times, motors['rf'].duty_cycles)
    
    return (lb_desired_line, lb_real_line, rb_desired_line, rb_real_line,
            lf_desired_line, lf_real_line, rf_desired_line, rf_real_line,
            lb_duty_line, rb_duty_line, lf_duty_line, rf_duty_line)

# Create animation
ani = animation.FuncAnimation(
    fig, update, init_func=init, 
    interval=100, blit=True, cache_frame_data=False
)

# Connect a callback to detect when the figure window is closed
def on_close(event):
    if ser is not None:
        ser.close()
        print("Serial port closed")

fig.canvas.mpl_connect('close_event', on_close)

plt.tight_layout()
plt.show()