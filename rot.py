import pandas as pd
import numpy as np
from scipy.spatial.transform import Rotation as R

# -------- Load CSV Files --------
gps_df = pd.read_csv("D:/GPS/mapping_gnss.csv")
imu_df = pd.read_csv("D:/GPS/mapping_imu.csv")
odom_df = pd.read_csv("D:/GPS/mapping_Odometry.csv")
motor_df = pd.read_csv("D:/GPS/mapping_status.csv")

# -------- Standardize and Clean --------
for df in [gps_df, imu_df, odom_df, motor_df]:
    df.rename(columns={"field.header.stamp": "timestamp"}, inplace=True)
    for col in ["%time", "field.header.seq", "field.header.frame_id"]:
        if col in df.columns:
            df.drop(columns=col, inplace=True)
    df["timestamp"] = df["timestamp"].astype(np.int64)
    df.sort_values("timestamp", inplace=True)

# -------- Merge Data on Timestamp with 50ms Tolerance --------
tolerance_ns = int(5e7)  # 50 milliseconds
merged = pd.merge_asof(gps_df, imu_df, on="timestamp", tolerance=tolerance_ns, direction="nearest")
merged = pd.merge_asof(merged, odom_df, on="timestamp", tolerance=tolerance_ns, direction="nearest")
merged = pd.merge_asof(merged, motor_df, on="timestamp", tolerance=tolerance_ns, direction="nearest")
merged.dropna(inplace=True)

# -------- Compute Pitch from Quaternion (degrees) --------
def compute_pitch(qx, qy, qz, qw):
    r = R.from_quat([qx, qy, qz, qw])
    return r.as_euler('xyz', degrees=True)[1]

pitch_list = [
    compute_pitch(qx, qy, qz, qw)
    for qx, qy, qz, qw in zip(
        merged["field.orientation.x"],
        merged["field.orientation.y"],
        merged["field.orientation.z"],
        merged["field.orientation.w"]
    )
]

# -------- Compute Speed in m/s from Velocity Vector --------
vx = merged["field.twist.twist.linear.x"]
vy = merged["field.twist.twist.linear.y"]
vz = merged["field.twist.twist.linear.z"]
speed_mps = np.sqrt(vx**2 + vy**2 + vz**2)

# -------- Apply Smoothing --------
pitch_smooth = pd.Series(pitch_list).rolling(window=5, center=True).mean()
speed_smooth = pd.Series(speed_mps).rolling(window=5, center=True).mean()
altitude_smooth = merged["field.altitude"].rolling(window=5, center=True).mean()

# -------- Create Final DataFrame --------
final_df = pd.DataFrame({
    "time_s": merged["timestamp"] / 1e9,  # convert from nanoseconds to seconds
    "latitude": merged["field.latitude"],
    "longitude": merged["field.longitude"],
    "altitude_m": altitude_smooth,
    "pitch_deg": pitch_smooth,
    "speed": speed_smooth,  # in m/s
    "torque": merged["field.motor_states0.current"]
})

final_df.dropna(inplace=True)

# -------- Save to CSV --------
output_path = "D:/GPS/merged_summary.csv"
final_df.to_csv(output_path, index=False)
print(f"✅ Real robot data saved to: {output_path}")
