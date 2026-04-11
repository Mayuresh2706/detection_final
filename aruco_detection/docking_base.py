import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class DockingBase(Node):
    def __init__(self, node_name, dock_marker_id):
        super().__init__(node_name)

        self.dock_marker_id = dock_marker_id

        # Odometry state
        self.current_x = 0.0
        self.current_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.current_yaw = 0.0
        self.start_yaw = 0.0

        # Motion targets
        self.distance_to_travel = 0.0
        self.angle_to_turn = 0.0

        # State machine
        self.state = 'idle'
        self.alignment_done = False   # False = standoff pass, True = final approach

        # Tuning parameters
        self.stop_dist = 0.40    # 10cm stop distance from marker
        self.x_threshold = 0.02  # 2cm lateral alignment tolerance
        self.lin_speed = 0.04    # 4cm/s
        self.ang_speed = 0.2     # rad/s

        # Activation gate
        self.is_active = False
        self._logged_waiting = False  # throttle flag for not-active log

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.fire_pub = self.create_publisher(Bool, '/fire', 10)

        # Subscriptions
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, 'target_3d', self.marker_callback, 10)

        # 20 Hz drive timer
        self.drive_timer = self.create_timer(0.05, self.drive_callback)

        self.get_logger().info(
            f'Node ready — dock_marker_id={self.dock_marker_id}. '
            f'Waiting for activation...'
        )

    # ------------------------------------------------------------------
    # Activation gate — subclasses wire their own topic to this callback
    # ------------------------------------------------------------------
    def active_cb(self, msg):
        self.is_active = msg.data
        if self.is_active:
            self._logged_waiting = False
            self.get_logger().info('Activated — looking for marker...')
        else:
            self.get_logger().info('Deactivated.')
            self.stop_robot()

    # ------------------------------------------------------------------
    # Shared odometry handler
    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    # ------------------------------------------------------------------
    # Shared marker callback — only reacts to dock_marker_id
    # ------------------------------------------------------------------
    def marker_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)

        if marker_id != self.dock_marker_id:
            return
        if not self.is_active:
            if not self._logged_waiting:
                self.get_logger().info(
                    f'Marker ID {marker_id} detected but node not active — '
                    f'publish /task_a_active or /task_b_active to start'
                )
                self._logged_waiting = True
            return
        if self.state != 'idle':
            return

        marker_z = msg.pose.position.x   # forward depth
        marker_x = msg.pose.position.y   # lateral offset

        if marker_z > 1.5:
            self.get_logger().info(f'Marker too far ({marker_z:.2f}m), waiting...')
            return

        if abs(marker_x) > 0.5:
            self.get_logger().info(f'Marker too far sideways ({marker_x:.2f}m), waiting...')
            return

        # Step 1: standoff pass — drive to 0.5m in front of marker
        # Step 2: final approach — drive to stop_dist
        target_dist = self.stop_dist if self.alignment_done else 0.5
        step_label = 'final approach' if self.alignment_done else 'standoff pass'

        if abs(marker_x) > self.x_threshold:
            self.angle_to_turn = math.atan2(marker_x, marker_z)
            self.distance_to_travel = max(0.0, marker_z - target_dist)
            self.start_yaw = self.current_yaw
            self.state = 'rotating'
            self.get_logger().info(
                f'[{step_label}] Marker at {marker_z:.2f}m forward, {marker_x:.2f}m sideways — '
                f'aligning {math.degrees(self.angle_to_turn):.1f}°, then driving {self.distance_to_travel:.3f}m'
            )
        else:
            self.distance_to_travel = max(0.0, marker_z - target_dist)
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.state = 'driving_to_marker'
            self.get_logger().info(
                f'[{step_label}] Marker at {marker_z:.2f}m forward, {marker_x:.2f}m sideways — '
                f'already aligned, driving {self.distance_to_travel:.3f}m'
            )

    # ------------------------------------------------------------------
    # Shared drive state machine
    # ------------------------------------------------------------------
    def drive_callback(self):
        if not self.is_active:
            self.state = 'idle'
            return

        if self.state == 'rotating':
            yaw_traveled = self._angle_diff(self.current_yaw, self.start_yaw)

            if abs(yaw_traveled) >= abs(self.angle_to_turn):
                self.get_logger().info('Aligned to marker! Driving in...')
                self.state = 'driving_to_marker'
                self.start_x = self.current_x
                self.start_y = self.current_y
            else:
                cmd = Twist()
                cmd.angular.z = -self.ang_speed if self.angle_to_turn > 0 else self.ang_speed
                self.cmd_pub.publish(cmd)

        elif self.state == 'driving_to_marker':
            dist_traveled = math.sqrt(
                (self.current_x - self.start_x) ** 2 +
                (self.current_y - self.start_y) ** 2
            )
            if dist_traveled >= self.distance_to_travel:
                self.cmd_pub.publish(Twist())
                if not self.alignment_done:
                    self.get_logger().info('Standoff reached — re-acquiring marker for final approach.')
                    self.alignment_done = True
                    self.state = 'idle'
                else:
                    self.get_logger().info('Final approach complete. Docked.')
                    self.state = 'docked'
                    self.on_docked()
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)

        elif self.state == 'docked':
            self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Hook — subclasses override to implement task-specific behaviour
    # ------------------------------------------------------------------
    def on_docked(self):
        pass

    # ------------------------------------------------------------------
    # Shared stop
    # ------------------------------------------------------------------
    def stop_robot(self):
        self.state = 'idle'
        self.alignment_done = False
        self.cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Angle utility
    # ------------------------------------------------------------------
    def _angle_diff(self, current, start):
        diff = current - start
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return diff
