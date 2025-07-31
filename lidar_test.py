import serial
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import medfilt
import threading

# === CONFIGURATION ===
SERIAL_PORT = 'COM6'       # Change this to match your serial port
BAUD_RATE = 115200         # Change this to match your device
PLOT_POLAR = False     # True for polar, False for XY plot
MAX_DISTANCE = 100        # Max distance to allow (adjust as needed)
MAX_POINTS = 10000           # Limit points to avoid clutter

# === DATA STORE ===
angles = []
distances = []
lock = threading.Lock()

# === SERIAL READER THREAD ===
def read_serial():
    with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1) as ser:
        while True:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("DATA:"):
                    _, payload = line.split("DATA:")
                    angle_str, distance_str = payload.split(',')

                    angle_deg = float(angle_str)  # Already in degrees
                    distance = float(distance_str) / 10

                    # Filter out unrealistic distances
                    if distance <= 0 or distance > MAX_DISTANCE:
                        continue

                    angle_rad = np.radians(angle_deg)

                    with lock:
                        angles.append(angle_rad)
                        distances.append(distance)

                        # Keep only latest MAX_POINTS
                        if len(angles) > MAX_POINTS:
                            angles.pop(0)
                            distances.pop(0)

                    print(f"Angle: {angle_deg:.1f}°, Distance: {distance:.1f} mm")

            except Exception as e:
                print(f"Parsing error: {e} | Raw line: {line}")

# === PLOTTING FUNCTION ===

def median_filter(data, kernel_size=5):
    if len(data) < kernel_size:
        return data
    return medfilt(data, kernel_size=kernel_size)

def update_plot():
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='polar' if PLOT_POLAR else None)
    wd_size = 9

    while True:
        with lock:
            ax.clear()

            if PLOT_POLAR:
                smoothed_distances = median_filter(distances, kernel_size=wd_size)
                ax.plot(angles, distances, 'b-')
                ax.set_title("Live Sensor Data (Polar View)")
            else:
                smoothed_distances = median_filter(distances, kernel_size=wd_size)

                x = [d * np.cos(a) for a, d in zip(angles, distances)]
                y = [d * np.sin(a) for a, d in zip(angles, distances)]
              
                ax.scatter(x, y, c='blue', s=2) 

                ax.set_title("Live Sensor Data (XY View)")
                ax.set_aspect('equal')

                # Auto-scale axes
                max_range = max(distances, default=1)
                ax.set_xlim(-max_range, max_range)
                ax.set_ylim(-max_range, max_range)

        plt.pause(0.1)  # Small delay for smooth updating

# === START THREADS ===
serial_thread = threading.Thread(target=read_serial, daemon=True)
serial_thread.start()

update_plot()
