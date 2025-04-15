import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import time

# Configure the serial port (adjust the port and baud rate as necessary)
ser = serial.Serial('COM8', 38400, timeout=1)

# Data storage dictionaries for each motor
motors = {
    'LB': {'times': [], 'desired': [], 'real': [], 'control': [], 'duty': [], 'error': [], 'integral': [], 'derivative': []},
    'RB': {'times': [], 'desired': [], 'real': [], 'control': [], 'duty': [], 'error': [], 'integral': [], 'derivative': []},
    'LF': {'times': [], 'desired': [], 'real': [], 'control': [], 'duty': [], 'error': [], 'integral': [], 'derivative': []},
    'RF': {'times': [], 'desired': [], 'real': [], 'control': [], 'duty': [], 'error': [], 'integral': [], 'derivative': []}
}

# Colors for each motor
motor_colors = {
    'LB': {'desired': 'blue', 'real': 'lightblue', 'control': 'darkblue', 'duty': 'royalblue'},
    'RB': {'desired': 'red', 'real': 'lightcoral', 'control': 'darkred', 'duty': 'firebrick'},
    'LF': {'desired': 'green', 'real': 'lightgreen', 'control': 'darkgreen', 'duty': 'forestgreen'},
    'RF': {'desired': 'orange', 'real': 'moccasin', 'control': 'darkorange', 'duty': 'goldenrod'}
}

# Use the actual time for x-axis data
start_time = time.time()

# Create figure and subplots
fig = plt.figure(figsize=(16, 12))
plt.subplots_adjust(hspace=0.4)

# Create 4 subplots for each data type
ax_speed = plt.subplot(411)  # Speed (desired and real)
ax_control = plt.subplot(412)  # Control values
ax_duty = plt.subplot(413)  # Duty cycle values
ax_error = plt.subplot(414)  # Error values

# Set up the plots
speed_lines = {}
control_lines = {}
duty_lines = {}
error_lines = {}

for motor in motors.keys():
    # Speed plot
    speed_lines[f"{motor}_desired"], = ax_speed.plot([], [], label=f'{motor} Desired', color=motor_colors[motor]['desired'], linestyle='-')
    speed_lines[f"{motor}_real"], = ax_speed.plot([], [], label=f'{motor} Real', color=motor_colors[motor]['real'], linestyle='--')
    
    # Control plot
    control_lines[motor], = ax_control.plot([], [], label=f'{motor} Control', color=motor_colors[motor]['control'])
    
    # Duty cycle plot
    duty_lines[motor], = ax_duty.plot([], [], label=f'{motor} Duty Cycle', color=motor_colors[motor]['duty'])
    
    # Error plot
    error_lines[motor], = ax_error.plot([], [], label=f'{motor} Error', color=motor_colors[motor]['desired'])

# Set plot labels and properties
ax_speed.set_ylabel('Speed')
ax_speed.set_title('Motor Speeds (Desired vs Real)')
ax_speed.legend(loc='upper left')
ax_speed.grid(True)

ax_control.set_ylabel('Control Value')
ax_control.set_title('Control Values')
ax_control.legend(loc='upper left')
ax_control.grid(True)

ax_duty.set_ylabel('Duty Cycle')
ax_duty.set_title('Duty Cycle Values')
ax_duty.legend(loc='upper left')
ax_duty.grid(True)

ax_error.set_xlabel('Time (s)')
ax_error.set_ylabel('Error')
ax_error.set_title('PID Error Values')
ax_error.legend(loc='upper left')
ax_error.grid(True)

# Create a text object for raw message display
message_text = fig.text(0.05, 0.97, '', fontsize=8)

# Regex pattern to match motor data from UART messages
pattern = re.compile(
    r"\[Δt: +(\d+\.\d+) s\] v: ([+-]\d+\.\d+) \| w: ([+-]\d+\.\d+) \| err: ([+-]\d+\.\d+) \| int: ([+-]\d+\.\d+) \| der: ([+-]\d+\.\d+) \| (LB|RB|LF|RF) Ideal: ([+-]\d+\.\d+) \| (LB|RB|LF|RF) Real: ([+-]\d+\.\d+) \| (LB|RB|LF|RF) Control: (\d+) \| (LB|RB|LF|RF) Duty Cycle: (\d+)"
)

window_size = 15  # Display window in seconds

def init():
    # Set initial x-axis limits for all subplots
    ax_speed.set_xlim(0, window_size)
    ax_speed.set_ylim(-2, 2)  # Adjust as needed
    
    ax_control.set_xlim(0, window_size)
    ax_control.set_ylim(-100, 1100)  # Adjust as needed
    
    ax_duty.set_xlim(0, window_size)
    ax_duty.set_ylim(-100, 1100)  # Adjust as needed
    
    ax_error.set_xlim(0, window_size)
    ax_error.set_ylim(-2, 2)  # Adjust as needed
    
    # Initialize all line data
    for motor in motors.keys():
        speed_lines[f"{motor}_desired"].set_data([], [])
        speed_lines[f"{motor}_real"].set_data([], [])
        control_lines[motor].set_data([], [])
        duty_lines[motor].set_data([], [])
        error_lines[motor].set_data([], [])
    
    message_text.set_text('')
    
    # Combine all lines into a single list for return
    return_lines = list(speed_lines.values()) + list(control_lines.values()) + list(duty_lines.values()) + list(error_lines.values()) + [message_text]
    return return_lines

def update(frame):
    global start_time
    
    try:
        # Try to read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        if line:
            # Update the text object with the most recent raw message
            message_text.set_text(line)
            
            # Parse the message using regex
            match = pattern.search(line)
            if match:
                # Extract the motor identifier (LB, RB, LF, RF)
                motor_id = match.group(7)  # The motor ID from the "LB/RB/LF/RF Ideal" part
                
                # Verify it matches the other mentions (should be the same motor throughout)
                if motor_id != match.group(9) or motor_id != match.group(11) or motor_id != match.group(13):
                    print(f"Warning: Inconsistent motor IDs in message: {line}")
                    return list(speed_lines.values()) + list(control_lines.values()) + list(duty_lines.values()) + list(error_lines.values()) + [message_text]
                
                # Extract all relevant data
                time_gap = float(match.group(1))
                v_desired = float(match.group(2))
                w_desired = float(match.group(3))
                error = float(match.group(4))
                integral = float(match.group(5))
                derivative = float(match.group(6))
                speed_ideal = float(match.group(8))
                speed_real = float(match.group(10))
                control = int(match.group(12))
                duty_cycle = int(match.group(14))
                
                current_time = time.time() - start_time
                
                # Store data for this motor
                motors[motor_id]['times'].append(current_time)
                motors[motor_id]['desired'].append(speed_ideal)
                motors[motor_id]['real'].append(speed_real)
                motors[motor_id]['control'].append(control)
                motors[motor_id]['duty'].append(duty_cycle)
                motors[motor_id]['error'].append(error)
                motors[motor_id]['integral'].append(integral)
                motors[motor_id]['derivative'].append(derivative)
                
                # Trim old data for this motor
                while motors[motor_id]['times'] and motors[motor_id]['times'][0] < current_time - window_size:
                    motors[motor_id]['times'].pop(0)
                    motors[motor_id]['desired'].pop(0)
                    motors[motor_id]['real'].pop(0)
                    motors[motor_id]['control'].pop(0)
                    motors[motor_id]['duty'].pop(0)
                    motors[motor_id]['error'].pop(0)
                    motors[motor_id]['integral'].pop(0)
                    motors[motor_id]['derivative'].pop(0)
                
                # Update plot lines for this motor
                speed_lines[f"{motor_id}_desired"].set_data(motors[motor_id]['times'], motors[motor_id]['desired'])
                speed_lines[f"{motor_id}_real"].set_data(motors[motor_id]['times'], motors[motor_id]['real'])
                control_lines[motor_id].set_data(motors[motor_id]['times'], motors[motor_id]['control'])
                duty_lines[motor_id].set_data(motors[motor_id]['times'], motors[motor_id]['duty'])
                error_lines[motor_id].set_data(motors[motor_id]['times'], motors[motor_id]['error'])
                
                # Update x-axis limits for all plots
                new_xlim = (max(0, current_time - window_size), current_time)
                ax_speed.set_xlim(*new_xlim)
                ax_control.set_xlim(*new_xlim)
                ax_duty.set_xlim(*new_xlim)
                ax_error.set_xlim(*new_xlim)
                
                # Auto-adjust y-axis limits if needed
                # Here we could add code to dynamically adjust y-limits based on data range
                
    except Exception as e:
        print(f"Error reading/parsing line: {e}")
    
    # Return all plot lines
    return_lines = list(speed_lines.values()) + list(control_lines.values()) + list(duty_lines.values()) + list(error_lines.values()) + [message_text]
    return return_lines

# Create the animation
ani = animation.FuncAnimation(fig, update, init_func=init, interval=20, blit=False, save_count=1000)

# Connect a callback to detect when the figure window is closed
def on_close(event):
    print("Animation window closed")
    # Uncomment the following lines to save the animation when closing
    # print("Saving video...")
    # ani.save('motor_animation.mp4', writer='ffmpeg', fps=10)
    # print("Animation saved as motor_animation.mp4")
    ser.close()  # Close the serial connection

fig.canvas.mpl_connect('close_event', on_close)

# Set a more descriptive window title
plt.get_current_fig_manager().set_window_title('Four Motor PID Controller Visualization')

plt.show()