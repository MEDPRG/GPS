# this script it isn't tested yet since we were wroking just on recoded data.
# we need to test on the robot directly
import rospy
import math
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial.transform import Rotation as R

class TerrainAdaptiveController:
    def __init__(self):
        rospy.init_node("terrain_adaptive_controller")

        # Subscribers
        self.gps_sub = Subscriber("/gps/fix", NavSatFix)
        self.imu_sub = Subscriber("/imu/data", Imu)
        self.odom_sub = Subscriber("/odom", Odometry)

        self.sync = ApproximateTimeSynchronizer(
            [self.gps_sub, self.imu_sub, self.odom_sub],
            queue_size=10,
            slop=0.1
        )
        self.sync.registerCallback(self.callback)

        # Publishers for rqt_plot (optional)
        self.pitch_pub = rospy.Publisher("/controller/pitch_deg", Float32, queue_size=10)
        self.speed_pub = rospy.Publisher("/controller/speed", Float32, queue_size=10)
        self.pwm_pub   = rospy.Publisher("/controller/pwm_output", Float32, queue_size=10)

        # RViz Path Publisher
        self.path_pub = rospy.Publisher("/controller/path", Path, queue_size=10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        rospy.loginfo("✅ Terrain Adaptive Controller Initialized")

    def compute_pitch_from_quaternion(self, qx, qy, qz, qw):
        r = R.from_quat([qx, qy, qz, qw])
        euler = r.as_euler('xyz', degrees=True)
        return euler[1]  # pitch in degrees

    def compute_target_speed(self, pitch):
        base_speed = 1.2  # m/s
        if pitch > 5:
            return base_speed + 0.4
        elif pitch < -5:
            return base_speed - 0.4
        else:
            return base_speed

    def compute_pwm_output(self, current_speed, target_speed):
        error = target_speed - current_speed
        Kp = 10.0
        pwm = Kp * error
        return max(0, min(100, pwm))

    def callback(self, gps_msg, imu_msg, odom_msg):
        # Extract GPS data
        lat = gps_msg.latitude
        lon = gps_msg.longitude
        alt = gps_msg.altitude

        # Extract IMU orientation
        qx = imu_msg.orientation.x
        qy = imu_msg.orientation.y
        qz = imu_msg.orientation.z
        qw = imu_msg.orientation.w
        pitch = self.compute_pitch_from_quaternion(qx, qy, qz, qw)

        # Extract linear velocity and compute speed
        vx = odom_msg.twist.twist.linear.x
        vy = odom_msg.twist.twist.linear.y
        vz = odom_msg.twist.twist.linear.z
        speed = math.sqrt(vx**2 + vy**2 + vz**2)

        # Compute target speed and PWM
        target_speed = self.compute_target_speed(pitch)
        pwm = self.compute_pwm_output(speed, target_speed)

        # Log to terminal
        rospy.loginfo(f"Pitch: {pitch:.2f} deg | Speed: {speed:.2f} m/s | Target: {target_speed:.2f} | PWM: {pwm:.2f}")

        # Publish for rqt_plot
        self.pitch_pub.publish(pitch)
        self.speed_pub.publish(speed)
        self.pwm_pub.publish(pwm)

        # Publish path for RViz
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = lon
        pose.pose.position.y = lat
        pose.pose.position.z = alt
        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_pub.publish(self.path_msg)

if __name__ == "__main__":
    try:
        controller = TerrainAdaptiveController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
