import csv
from collections import deque
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D

# --- Sensor Input Class ---
class SimulatedSensorInput:
    def __init__(self, filename):
        with open(filename) as f:
            reader = csv.DictReader(f)
            self.data = list(reader)
        self.index = 0

    def get_next(self):
        if self.index < len(self.data):
            row = self.data[self.index]
            self.index += 1
            return {
                "time": float(row["time_s"]),
                "latitude": float(row["latitude"]),
                "longitude": float(row["longitude"]),
                "altitude": float(row["altitude_m"]),
                "pitch": float(row["pitch_deg"]),
                "speed": float(row["speed"]),
                "torque": float(row["torque"])
            }
        else:
            return None

# --- Adaptive Speed Controller ---
class TerrainAdaptiveController:
    def __init__(self, sensor_input):
        self.sensor_input = sensor_input
        self.pitch_buffer = deque(maxlen=100)
        self.speed_buffer = deque(maxlen=100)
        self.pwm_buffer = deque(maxlen=100)
        self.altitude_buffer = deque(maxlen=100)
        self.lat_buffer = deque(maxlen=300)
        self.lon_buffer = deque(maxlen=300)
        self.alt_buffer = deque(maxlen=300)
        self.time_counter = 0

        self.log_file = open("results_log.csv", "w", newline="")
        self.logger = csv.writer(self.log_file)
        self.logger.writerow([
            "time_s", "pitch_deg", "speed", "target_speed",
            "pwm_output", "altitude_m", "latitude", "longitude"
        ])

    def compute_target_speed(self, pitch):
        base_speed = 1.2  # m/s
        if pitch > 5:
            return base_speed + 0.4  # uphill
        elif pitch < -5:
            return base_speed - 0.4  # downhill
        else:
            return base_speed  # flat terrain

    def compute_pwm_output(self, current_speed, target_speed):
        error = target_speed - current_speed
        Kp = 10.0  # tuned for low-speed robot
        pwm_output = Kp * error
        return max(0, min(100, pwm_output))  # clamp between 0–100

    def step(self):
        data = self.sensor_input.get_next()
        if data is None:
            return False

        pitch = data["pitch"]
        speed = data["speed"]
        altitude = data["altitude"]
        lat = data["latitude"]
        lon = data["longitude"]

        target_speed = self.compute_target_speed(pitch)
        pwm_output = self.compute_pwm_output(speed, target_speed)

        self.pitch_buffer.append(pitch)
        self.speed_buffer.append((speed, target_speed))
        self.pwm_buffer.append(pwm_output)
        self.altitude_buffer.append(altitude)
        self.lat_buffer.append(lat)
        self.lon_buffer.append(lon)
        self.alt_buffer.append(altitude)

        self.logger.writerow([
            round(self.time_counter, 2),
            round(pitch, 2),
            round(speed, 2),
            round(target_speed, 2),
            round(pwm_output, 2),
            round(altitude, 2),
            lat, lon
        ])

        self.time_counter += 1
        return True

    def run_with_plotting(self):
        fig = plt.figure(figsize=(12, 8))
        axs = [
            fig.add_subplot(2, 2, 1),
            fig.add_subplot(2, 2, 2),
            fig.add_subplot(2, 2, 3),
            fig.add_subplot(2, 2, 4, projection='3d')
        ]

        def update(frame):
            keep_going = self.step()
            if not keep_going:
                ani.event_source.stop()
                self.log_file.close()
                return

            axs[0].clear()
            axs[0].plot(self.pitch_buffer)
            axs[0].set_title("Pitch (deg)")

            axs[1].clear()
            speeds = list(self.speed_buffer)
            if speeds:
                actual, target = zip(*speeds)
                axs[1].plot(actual, label="Actual Speed")
                axs[1].plot(target, label="Target Speed")
                axs[1].legend()
                axs[1].set_title("Speed vs Target")

            axs[2].clear()
            axs[2].plot(self.pwm_buffer)
            axs[2].set_title("PWM Output")

            axs[3].clear()
            axs[3].plot3D(self.lon_buffer, self.lat_buffer, self.alt_buffer, color='green')
            axs[3].set_xlabel("Longitude")
            axs[3].set_ylabel("Latitude")
            axs[3].set_zlabel("Altitude")
            axs[3].set_title("3D Terrain Mapping")

        ani = FuncAnimation(fig, update, interval=200)
        plt.tight_layout()
        plt.show()

# --- Main Execution ---
if __name__ == "__main__":
    sensor = SimulatedSensorInput("D:/GPS/merged_summary.csv")
    controller = TerrainAdaptiveController(sensor)
    controller.run_with_plotting()
