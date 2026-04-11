import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
from std_msgs.msg import Bool, String
from servo import MG996R


class Task_A_Controller(Node):
    def __init__(self):
        super().__init__('task_a_node')
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.active_sub = self.create_subscription(Bool, '/task_a_active', self.active_cb, 10)
        self.create_subscription(PoseStamped, 'target_3d', self.task_a, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.drive_timer = self.create_timer(0.05, self.drive_callback)

        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.current_yaw = 0.0

        self.distance_to_travel = 0.0
        self.angle_to_turn = 0.0
        self.state = 'idle'

        # Tuneables
        self.stop_dist   = 0.07   # 7cm from aruco marker
        self.side_offset = 0.15   # 15cm sideways to actual target (to the right)
        self.x_threshold = 0.02   # lateral alignment tolerance (2cm)
        self.lin_speed   = 0.04   # 4cm/s
        self.ang_speed   = 0.2    # rad/s

        # rotation_phase:
        # 0 = aligning face-on to aruco
        # 1 = turning RIGHT 90° toward target
        # 2 = turning LEFT 90° to face target after offset drive
        self.rotation_phase = 0

        self.is_active = False

        # Shot sequencing
        self.shots_fired = 0
        self.total_shots = 3
        self.shot_interval = 5.0  # seconds between shots
        self.firing = False       # guard so fire() isn't re-entered

        # Initiate servo
        self.servo = MG996R(pin=18)

    def active_cb(self, msg):
        self.is_active = msg.data
        if not self.is_active:
            self.stop_robot()

    def drive_callback(self):
        if not self.is_active:
            self.state = 'idle'
            return

        if self.state == 'rotating':
            yaw_traveled = self._angle_diff(self.current_yaw, self.start_yaw)

            if abs(yaw_traveled) >= abs(self.angle_to_turn):
                if self.rotation_phase == 0:
                    self.get_logger().info("Aligned to marker! Driving in...")
                    self.rotation_phase = 0
                    self.state = 'driving_to_marker'
                    self.start_x = self.current_x
                    self.start_y = self.current_y

                elif self.rotation_phase == 1:
                    self.get_logger().info("Turned right! Driving offset...")
                    self.rotation_phase = 0
                    self.state = 'driving_offset'
                    self.start_x = self.current_x
                    self.start_y = self.current_y

                elif self.rotation_phase == 2:
                    self.get_logger().info("Facing target!")
                    self.rotation_phase = 0
                    self.fire_sequence()
            else:
                cmd = Twist()
                cmd.angular.z = -self.ang_speed if self.angle_to_turn > 0 else self.ang_speed
                self.cmd_pub.publish(cmd)

        elif self.state == 'driving_to_marker':
            dist_traveled = math.sqrt(
                (self.current_x - self.start_x)**2 +
                (self.current_y - self.start_y)**2
            )
            if dist_traveled >= self.distance_to_travel:
                self.get_logger().info("Reached marker! Turning right 90°")
                self.rotation_phase = 1
                self.angle_to_turn = math.pi / 2
                self.state = 'rotating'
                self.start_yaw = self.current_yaw
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)

        elif self.state == 'driving_offset':
            dist_traveled = math.sqrt(
                (self.current_x - self.start_x)**2 +
                (self.current_y - self.start_y)**2
            )
            if dist_traveled >= self.side_offset:
                self.get_logger().info("Offset reached! Turning left 90°")
                self.rotation_phase = 2
                self.angle_to_turn = -math.pi / 2
                self.state = 'rotating'
                self.start_yaw = self.current_yaw
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def task_a(self, msg):
        self.get_logger().info(f"task_a called, is_active={self.is_active}, state={self.state}")
        if not self.is_active: return
        if self.state != 'idle': return

        marker_id = int(msg.pose.orientation.w)
        if marker_id != 1: return

        marker_z = msg.pose.position.x   # forward depth
        marker_x = msg.pose.position.y   # lateral offset

        if marker_z > 0.5:
            self.get_logger().info(f"Marker too far ({marker_z:.2f}m), waiting...")
            return

        if abs(marker_x) > 0.3:
            self.get_logger().info(f"Marker too far sideways ({marker_x:.2f}m), waiting...")
            return

        if abs(marker_x) > self.x_threshold:
            self.angle_to_turn = math.atan2(marker_x, marker_z)
            self.distance_to_travel = max(0.0, marker_z - self.stop_dist)
            self.start_yaw = self.current_yaw
            self.rotation_phase = 0
            self.state = 'rotating'
            self.get_logger().info(f"Aligning: {math.degrees(self.angle_to_turn):.1f}°, then driving {self.distance_to_travel:.3f}m")
        else:
            self.distance_to_travel = max(0.0, marker_z - self.stop_dist)
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'driving_to_marker'
            self.get_logger().info(f"Already aligned. Driving {self.distance_to_travel:.3f}m")

    def fire_sequence(self):
        """Kick off the 3-shot sequence in a background thread so the ROS spin isn't blocked."""
        if self.firing:
            return
        self.firing = True
        self.shots_fired = 0
        threading.Thread(target=self._fire_thread, daemon=True).start()

    def _fire_thread(self):
        import time
        while self.shots_fired < self.total_shots:
            self.shots_fired += 1
            self.get_logger().info(f"🔥 FIRE {self.shots_fired}/{self.total_shots}")
            self.servo.fire(count = 1)
            if self.shots_fired < self.total_shots:
                time.sleep(self.shot_interval)

        # All shots done
        self.get_logger().info("All shots fired. Reporting SUCCESS.")
        self.firing = False
        status_msg = String()
        status_msg.data = "SUCCESS"
        self.status_pub.publish(status_msg)
        self.stop_robot()

    def stop_robot(self):
        self.state = 'idle'
        self.rotation_phase = 0
        self.shots_fired = 0
        self.firing = False
        self.cmd_pub.publish(Twist())

    def _angle_diff(self, current, start):
        diff = current - start
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff


def main(args=None):
    rclpy.init(args=args)
    node = Task_A_Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Manual Shutdown")
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        node.servo.close()
        rclpy.shutdown()


if __name__ == '__main__':
    main()