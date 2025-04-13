import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import time

# Configure the serial port (adjust the port and baud rate as necessary)
ser = serial.Serial('COM8', 38400, timeout=1)

# Data storage lists for time and values
times = []
desired_speeds = []
real_speeds = []
control_values = []       # For LB Control
duty_cycle_values = []    # For LB Duty Cycle

# Use the actual time for x-axis data
start_time = time.time()

# Create a figure with two subplots sharing the x-axis.
# The top subplot (ax1) is for speeds, the bottom (ax2) for LB Control and LB Duty Cycle.
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(12, 8))

# Top subplot for speeds
line_desired, = ax1.plot([], [], label='Desired Speed', color='blue')
line_real, = ax1.plot([], [], label='Real Speed', color='red')
ax1.set_ylabel('Speed')
ax1.legend(loc='upper left')
ax1.grid(True)

# Bottom subplot for control and duty cycle
line_control, = ax2.plot([], [], label='LB Control', color='green')
line_duty, = ax2.plot([], [], label='LB Duty Cycle', color='orange')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Control / Duty Cycle')
ax2.legend(loc='upper left')
ax2.grid(True)

# Create a text object anchored to the figure for stable message display.
message_text = fig.text(0.05, 0.95, '', fontsize=10)

# Revised regex pattern to capture the fields.
# For LB Ideal and LB Real, an explicit sign is expected (with optional preceding whitespace).
# For LB Control and LB Duty Cycle, an optional sign is allowed.
pattern = re.compile(
    r"LB Ideal:\s*((?:\s*[+-])\d+\.\d+)\s*\|\s*LB Real:\s*((?:\s*[+-])\d+\.\d+)\s*\|\s*LB Control:\s*([+-]?\d+)\s*\|\s*LB Duty Cycle:\s*([+-]?\d+)"
)

window_size = 10  # Display window in seconds

def init():
    # Set fixed x-axis and y-axis limits for the speed plot.
    ax1.set_xlim(0, window_size)
    ax1.set_ylim(-2, 2)  # Adjust these limits as needed for your speed values.
    
    # For the bottom subplot, set y-axis limits to accommodate control and duty cycle values.
    ax2.set_xlim(0, window_size)
    ax2.set_ylim(-100, 1100)  # Adjust these limits as needed.
    
    # Initialize line data
    line_desired.set_data([], [])
    line_real.set_data([], [])
    line_control.set_data([], [])
    line_duty.set_data([], [])
    message_text.set_text('')
    return line_desired, line_real, line_control, line_duty, message_text

def update(frame):
    global start_time
    try:
        # Read a line from the serial port.
        line = ser.readline().decode('utf-8').strip()
        if line:
            # Update the text object with the raw message.
            message_text.set_text(line)
            
            # Parse the message using the revised regex pattern.
            match = pattern.search(line)
            if match:
                # Capture LB Ideal and LB Real strings and remove extra whitespace.
                desired_str = match.group(1).strip()
                real_str = match.group(2).strip()
                try:
                    desired = float(desired_str)
                    real = float(real_str)
                except ValueError:
                    return line_desired, line_real, line_control, line_duty, message_text
                
                # Capture LB Control and LB Duty Cycle.
                lb_control = int(match.group(3))
                lb_duty_cycle = int(match.group(4))
                
                current_time = time.time() - start_time

                # Append new data points for plotting.
                times.append(current_time)
                desired_speeds.append(desired)
                real_speeds.append(real)
                control_values.append(lb_control)
                duty_cycle_values.append(lb_duty_cycle)
                
                # Trim data to keep only the most recent 'window_size' seconds.
                while times and times[0] < current_time - window_size:
                    times.pop(0)
                    desired_speeds.pop(0)
                    real_speeds.pop(0)
                    control_values.pop(0)
                    duty_cycle_values.pop(0)
                
                # Update the plot lines.
                line_desired.set_data(times, desired_speeds)
                line_real.set_data(times, real_speeds)
                line_control.set_data(times, control_values)
                line_duty.set_data(times, duty_cycle_values)
                
                # Update x-axis limits for both subplots.
                new_xlim = (max(0, current_time - window_size), current_time)
                ax1.set_xlim(*new_xlim)
                ax2.set_xlim(*new_xlim)
    except Exception as e:
        print("Error reading/parsing line:", e)
    
    return line_desired, line_real, line_control, line_duty, message_text

# Create the animation with a high save_count to capture many frames.
ani = animation.FuncAnimation(fig, update, init_func=init, interval=20, blit=False, save_count=1000)

# Connect a callback to detect when the figure window is closed.
def on_close(event):
    # print("Animation window closed, saving video...")
    # ani.save('my_animation.mp4', writer='ffmpeg', fps=10)
    # print("Animation saved as my_animation.mp4")
    pass

fig.canvas.mpl_connect('close_event', on_close)

plt.show()
