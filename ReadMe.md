# 🌄 Terrain-Adaptive Speed Controller for Ground Robots

This repository provides a complete pipeline for **adaptive speed control** of a robot navigating different terrains using **GPS, IMU, and Odometry** data. The controller adjusts speed based on pitch (terrain inclination) using PD/P controllers. It includes:

- 🧪 Data simulation for offline testing
- 🛠 Merging and preprocessing real ROS-recorded sensor data
- 📊 Visual testing and plotting
- 🤖 Real-time deployment on a ROS-enabled robot

---

## 🗂️ Project Structure

```
.
├── simulate_data.py             # Simulates GPS/IMU data into CSV
├── merge_ros_csv.py            # Merges GNSS, IMU, Odometry, Motor into one file
├── adaptive_controller_test.py # Simulated controller + live plots
├── adaptive_controller_ros.py  # Final ROS-based controller
├── simulated_gps_data.csv      # Output of simulated data
├── merged_summary.csv          # Output of merged real robot data
├── results_log.csv             # Log of controller results in test mode
├── README.md                   # This file
```

---

## 📌 Goals

- Adapt robot speed automatically based on pitch (incline/decline).
- Provide testable pipeline on simulated and real robot data.
- Visualize system response in real-time using `matplotlib` and RViz.
- Maintain modular, easy-to-tune PID control logic.

---

## 🚦 1. Simulate Synthetic Sensor Data

**File:** `simulate_data.py`

This script generates realistic-looking GPS, pitch, speed, and torque values into a CSV file.

**Features:**

- Sine-based pitch simulation with noise
- Speed and torque adapt based on pitch
- Outputs to `simulated_gps_data.csv`

```bash
python simulate_data.py
```

---

## 🧩 2. Merge Real ROS CSV Logs

**File:** `merge_ros_csv.py`

Merges logs extracted from a ROS bag using `rostopic echo` or `rosbag_to_csv` tools. Supported inputs:

- `gnss.csv`
- `imu_data.csv`
- `odometry.csv`
- `scout_status.csv`

**Process:**

- Timestamp alignment with tolerance
- Computes pitch from IMU quaternions
- Calculates velocity magnitude
- Applies smoothing on pitch, speed, and altitude
- Outputs to `merged_summary.csv`

```bash
python merge_ros_csv.py
```

---

## 📈 3. Simulated Adaptive Control & Visualization

**File:** `adaptive_controller_test.py`

Simulates the controller pipeline using the merged CSV data before running it on the robot.

**Features:**

- PD controller for PWM output
- Plots pitch, speed vs. target speed, PWM output, and 3D path
- Saves logs to `results_log.csv`

```bash
python adaptive_controller_test.py
```

**Example Plot Output:**

- Subplot 1: Real-time pitch
- Subplot 2: Actual vs Target speed
- Subplot 3: PWM output over time
- Subplot 4: 3D Terrain Path

---

## 🤖 4. Real Robot Deployment with ROS

**File:** `adaptive_controller_ros.py`

ROS node to control robot based on real-time sensor data.

**ROS Topics Subscribed:**

- `/gps/fix` → Position
- `/imu/data` → Orientation (to compute pitch)
- `/odom` → Velocity

**ROS Topics Published:**

- `/controller/pitch_deg`, `/controller/speed`, `/controller/pwm_output` → For monitoring
- `/controller/path` → Path visualization in RViz
- `/cmd_vel` → Commands the robot to move

**Usage:**

```bash
rosrun your_package adaptive_controller_ros.py
```

---

## ⚙️ PID Control Strategy

The system uses **PD control** to compute PWM commands:

- `P`: Proportional term (error-based)
- `D`: Derivative term (rate of change)

Can be easily switched to pure P or extended to full PID.

```python
pwm = Kp * error + Kd * (de / dt)
```

---

## 🛠 Requirements

- Python 3.6+
- ROS (tested on ROS Noetic)
- `numpy`, `pandas`, `matplotlib`, `scipy`
- `rospy`, `message_filters`, `sensor_msgs`, `nav_msgs`, etc.
- For visualization: `RViz`, `rqt_plot` (optional)

---

## 📌 Example Demo

- Simulate → Merge → Test → Deploy
- Works on both flat and inclined terrain
- Ideal for autonomous ground robots, delivery bots, UGVs

---

## 📤 Data Output

| Column Name | Description                    |
| ----------- | ------------------------------ |
| time_s      | Time in seconds                |
| latitude    | GPS latitude                   |
| longitude   | GPS longitude                  |
| altitude_m  | Altitude in meters             |
| pitch_deg   | Pitch angle from IMU (degrees) |
| speed       | Current speed (m/s)            |
| torque      | Motor torque (if available)    |

---

## 📎 Notes

- Replace hardcoded file paths (`D:/GPS/...`) with relative paths for portability.
- Tune `Kp`, `Kd`, and pitch thresholds based on your robot and terrain.
- RViz visualization assumes a fixed frame called `"map"`.

---

## 🧠 Future Work

- Integrate Kalman filter for pitch/speed smoothing
- Fuse terrain classification with adaptive control
- Extend to 3D navigation using 9DOF IMU and depth sensors

---

## 👤 Authors

[**Mohammed EL Amine Hoceini**](https://github.com/MEDPRG)  
[**Ines Meliani**](https://github.com/MELIANIInes)

MSc in Autonomous Systems  
Eötvös Loránd University, Budapest, Hungary
