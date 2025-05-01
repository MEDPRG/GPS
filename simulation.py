import csv
import math
import random

# Settings
duration = 500  # seconds
sampling_rate = 1  # Hz
num_samples = duration * sampling_rate

# Starting values
latitude = 47.4979
longitude = 19.0402
altitude = 110.0  # in meters
pitch_amplitude = 10  # degrees
base_speed = 50  # km/h or arbitrary unit
base_torque = 10  # arbitrary unit

# GPS movement per second (small increments for realism)
lat_step = 0.0001
lon_step = 0.0001

# Generate and write data
data = []
for t in range(num_samples):
    # Time
    time_s = t

    # Simulate pitch (sine wave + noise)
    pitch = pitch_amplitude * math.sin(2 * math.pi * t / 20.0) + random.uniform(-1, 1)

    # Update altitude based on pitch
    altitude += (pitch / 10.0) * 0.1  # climb or descent

    # Simulate speed with effect from pitch (uphill → slower)
    speed = base_speed - 0.5 * pitch + random.uniform(-1.5, 1.5)

    # Simulate torque increasing with pitch
    torque = base_torque + 0.3 * pitch + random.uniform(-0.5, 0.5)

    # Append the data row
    data.append([
        time_s,
        round(latitude + t * lat_step, 6),
        round(longitude + t * lon_step, 6),
        round(altitude, 2),
        round(pitch, 2),
        round(speed, 2),
        round(torque, 2)
    ])

# Write to CSV
filename = "simulated_gps_data.csv"
with open(filename, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["time_s", "latitude", "longitude", "altitude_m", "pitch_deg", "speed", "torque"])
    writer.writerows(data)

print(f"Simulation data saved to {filename}")
