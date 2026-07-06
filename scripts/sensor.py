import serial
import serial.tools.list_ports
import re
import time
import platform
import threading
import math
from collections import deque
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- User's Matrix Kalman Filter Class ---
class KalmanFilter(object):
    def __init__(self, F=None, B=None, H=None, Q=None, R=None, P=None, x0=None):
        if(F is None or H is None):
            raise ValueError("Set proper system dynamics.")

        self.n = F.shape[1]
        self.m = H.shape[1] # Note: typically m = H.shape[0], but keeping as provided

        self.F = F
        self.H = H
        self.B = 0 if B is None else B
        self.Q = np.eye(self.n) if Q is None else Q
        self.R = np.eye(self.n) if R is None else R
        self.P = np.eye(self.n) if P is None else P
        self.x = np.zeros((self.n, 1)) if x0 is None else x0

    def predict(self, u=0):
        self.x = np.dot(self.F, self.x) + np.dot(self.B, u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q
        return self.x

    def update(self, z):
        y = z - np.dot(self.H, self.x)
        S = self.R + np.dot(self.H, np.dot(self.P, self.H.T))
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.n)
        self.P = np.dot(np.dot(I - np.dot(K, self.H), self.P), 
                        (I - np.dot(K, self.H)).T) + np.dot(np.dot(K, self.R), K.T)

# --- Configuration ---
MAX_DATAPOINTS = 200
BAUDRATE = 115200

LINE_RE = re.compile(
    r"Accel\s+X:\s*([-\d.]+)\s+Y:\s*([-\d.]+)\s+Z:\s*([-\d.]+)\s+"
    r"Gyro\s+Roll:\s*([-\d.]+)\s+Pitch:\s*([-\d.]+)\s+Yaw:\s*([-\d.]+)\s+"
    r"Temp:\s*([-\d.]+)"
)

def find_serial_port():
    ports = serial.tools.list_ports.comports()
    if platform.system() == "Windows":
        candidates = [p.device for p in ports if p.device.upper().startswith("COM")]
        return candidates[0] if candidates else "COM3"
    else:
        candidates = [
            p.device for p in ports
            if "ttyUSB" in p.device or "ttyACM" in p.device
            or "usbserial" in p.device or "usbmodem" in p.device
        ]
        return candidates[0] if candidates else "/dev/ttyACM0"

def parse_line(line: str):
    match = LINE_RE.search(line)
    if not match:
        return None
    ax, ay, az, roll_rate, pitch_rate, yaw_rate, temp = (float(x) for x in match.groups())
    # We still parse everything, but we will only return what we need to process
    return {
        "roll_rate": roll_rate, "pitch_rate": pitch_rate, "yaw_rate": yaw_rate,
        "temp": temp
    }

# --- Global Shared Data ---
PLOT_CHANNELS = ["roll", "pitch", "yaw", "temp"]
data_history = {ch: deque(maxlen=MAX_DATAPOINTS) for ch in PLOT_CHANNELS}
timestamps = deque(maxlen=MAX_DATAPOINTS)
stats = {"unparsed_count": 0, "running": True}

# Helper to initialize 1D states using the generic matrix class
def create_1d_kf(q_variance, r_variance):
    return KalmanFilter(
        F = np.array([[1.0]]),
        H = np.array([[1.0]]),
        Q = np.array([[q_variance]]),
        R = np.array([[r_variance]])
    )

def serial_worker(port, start_time):
    # Initialize matrix filters for the channels we care about
    filters = {
        "roll_rate": create_1d_kf(0.5, 0.01),
        "pitch_rate": create_1d_kf(0.5, 0.01),
        "yaw_rate": create_1d_kf(0.5, 0.01),
        "temp": create_1d_kf(1e-5, 1e-2)
    }
    
    buffer = ""
    accumulated_angles = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}
    last_time = time.perf_counter()
    
    while stats["running"]:
        try:
            with serial.Serial(port, BAUDRATE, timeout=1) as ser:
                ser.reset_input_buffer()
                print("\n[Thread] Connected. Processing orientation and temp with Numpy KF...", flush=True)
                last_time = time.perf_counter()
                
                while stats["running"]:
                    if ser.in_waiting > 0:
                        chunk = ser.read_all().decode("utf-8", errors="replace")
                        buffer += chunk.replace("\r\n", "\n").replace("\r", "\n")
                        
                        while "\n" in buffer:
                            line, buffer = buffer.split("\n", 1)
                            line = line.strip()
                            if not line:
                                continue

                            parsed = parse_line(line)
                            if parsed is None:
                                stats["unparsed_count"] += 1
                                continue

                            now = time.perf_counter()
                            dt = now - last_time
                            last_time = now

                            filtered = {}
                            
                            # 1. Filter Temperature
                            filters["temp"].predict()
                            filters["temp"].update(np.array([[parsed["temp"]]]))
                            filtered["temp"] = float(filters["temp"].x[0, 0])

                            # 2. Convert to Degrees, Filter, then Integrate Gyros
                            rate_to_angle = {"roll_rate": "roll", "pitch_rate": "pitch", "yaw_rate": "yaw"}
                            for rate_ch, angle_ch in rate_to_angle.items():
                                deg_rate = math.degrees(parsed[rate_ch])
                                
                                filters[rate_ch].predict()
                                filters[rate_ch].update(np.array([[deg_rate]]))
                                clean_rate = float(filters[rate_ch].x[0, 0])
                                
                                accumulated_angles[angle_ch] += clean_rate * dt
                                filtered[angle_ch] = accumulated_angles[angle_ch]
                            
                            current_time = time.time() - start_time
                            timestamps.append(current_time)
                            
                            for ch in PLOT_CHANNELS:
                                data_history[ch].append(filtered[ch])
                    else:
                        time.sleep(0.01)
        except Exception as e:
            if stats["running"]:
                print(f"\n[Thread] Connection lost ({e}). Retrying in 2s...")
                time.sleep(2)

def main():
    port = find_serial_port()
    print(f"Targeting port: {port}")
    
    start_time = time.time()
    
    thread = threading.Thread(target=serial_worker, args=(port, start_time), daemon=True)
    thread.start()

    # Reduced to 2 subplots: Angles and Temp
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig.canvas.manager.set_window_title('Filtered Orientation & Temperature')

    lines = {
        "roll": ax1.plot([], [], label="Roll")[0],
        "pitch": ax1.plot([], [], label="Pitch")[0],
        "yaw": ax1.plot([], [], label="Yaw")[0],
        "temp": ax2.plot([], [], label="Temp", color="red")[0],
    }

    ax1.set_title("Orientation (Numpy Matrix KF Integrated Gyro)")
    ax1.set_ylabel("Angle (Degrees)")
    ax1.legend(loc="upper left")
    ax1.grid(True)

    ax2.set_title("Temperature (Numpy Matrix KF)")
    ax2.set_ylabel("Chip Temp (°C)")
    ax2.set_xlabel("Time (s)")
    ax2.legend(loc="upper left")
    ax2.grid(True)

    plt.tight_layout()

    def update_plot(frame):
        if not timestamps:
            return list(lines.values())

        t_list = list(timestamps)
        
        for ch, line in lines.items():
            line.set_data(t_list, list(data_history[ch]))

        min_t, max_t = t_list[0], t_list[-1]
        ax1.set_xlim(min_t, max_t)

        for ax, channels in [(ax1, ["roll", "pitch", "yaw"]), (ax2, ["temp"])]:
            y_mins = [min(data_history[ch]) for ch in channels if data_history[ch]]
            y_maxs = [max(data_history[ch]) for ch in channels if data_history[ch]]
            
            if y_mins and y_maxs:
                y_min, y_max = min(y_mins), max(y_maxs)
                padding = (y_max - y_min) * 0.1
                if padding == 0: padding = 1.0 
                ax.set_ylim(y_min - padding, y_max + padding)

        return list(lines.values())

    ani = animation.FuncAnimation(fig, update_plot, interval=50, blit=False, cache_frame_data=False)
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        print("\nStopping application...")
        stats["running"] = False
        print(f"Total unparsed/corrupted lines skipped: {stats['unparsed_count']}")

if __name__ == "__main__":
    main()