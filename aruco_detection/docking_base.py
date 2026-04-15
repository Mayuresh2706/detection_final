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

        # State machine:
        # idle -> align_front -> turn_face -> visual_approach -> odom_drive -> docked
        #
        # align_front     : drive straight to the point directly in front of the marker
        # turn_face       : rotate in place until angle to marker is ~0
        # visual_approach : drive straight toward marker, stop at stop_dist
        # odom_drive      : fallback if marker is lost near the end
        self.state = 'idle'

        # Tuning
        self.stop_dist = 0.05           # final stop distance from marker (m)
        self.lateral_threshold = 0.05   # x-offset threshold for align_front (m)
        self.yaw_threshold = 0.05       # angle threshold for turn_face (rad)
        self.x_threshold = 0.04         # x-offset threshold during visual_approach (m)
        self.lin_speed = 0.06           # max linear speed (m/s)
        self.ang_speed = 0.3            # max angular speed (rad/s)

        # Marker state
        self.last_marker_z = 0.0
        self.last_marker_x = 0.0
        self.last_marker_rvec_y = 0.0
        self.last_marker_rvec_z = 0.0
        self.last_marker_time = None
        self.marker_timeout = 0.3

        # Odom drive fallback
        self.drive_distance = 0.0

        # Approach counter
        self.aligned_count = 0
        self.aligned_frames_needed = 5

        # Lateral alignment sub-phases (used in align_front)
        # 'turn_to_target' -> rotate to face the lateral waypoint
        # 'drive_to_target' -> drive straight to it
        self._align_phase = 'turn_to_target'
        self._align_target_yaw = None       # heading toward waypoint (None = not yet computed)
        self._align_target_world_x = 0.0   # world-frame waypoint
        self._align_target_world_y = 0.0
        self._align_drive_dist = 0.0
        self._align_drive_start_x = 0.0
        self._align_drive_start_y = 0.0
        self._align_spin_dir = 1            # kept for turn_face recovery direction

        # Activation
        self.is_active = False
        self._logged_waiting = False

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'task_status', 10)
        self.fire_pub = self.create_publisher(Bool, '/fire', 10)

        # Subscriptions
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, 'target_3d', self.marker_callback, 10)

        self.drive_timer = self.create_timer(0.05, self.drive_callback)

        self.get_logger().info(
            f'Node ready — dock_marker_id={self.dock_marker_id}. '
            f'Waiting for activation...'
        )

    def active_cb(self, msg):
        self.is_active = msg.data
        if self.is_active:
            self._logged_waiting = False
            self.aligned_count = 0
            self.last_marker_time = None
            self._align_phase = 'turn_to_target'
            self._align_target_yaw = None
            self._align_target_world_x = 0.0
            self._align_target_world_y = 0.0
            self._align_drive_dist = 0.0
            self.state = 'align_front'
            self.get_logger().info('Activated — computing waypoint to align in front of marker...')
        else:
            self.get_logger().info('Deactivated.')
            self.stop_robot()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def marker_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)
        if marker_id != self.dock_marker_id:
            return
        if not self.is_active:
            if not self._logged_waiting:
                self.get_logger().info(f'Marker {marker_id} seen but not active.')
                self._logged_waiting = True
            return

        self.last_marker_z = msg.pose.position.x   # depth (forward distance)
        self.last_marker_x = msg.pose.position.y   # lateral offset
        self.last_marker_rvec_y = msg.pose.orientation.y
        self.last_marker_rvec_z = msg.pose.orientation.z
        self.last_marker_time = self.get_clock().now().nanoseconds / 1e9

    def drive_callback(self):
        if not self.is_active:
            self.state = 'idle'
            return

        if self.state == 'docked':
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # ── STEP 1: Compute the point directly in front of the marker and drive straight to it ──
        #   Sub-phase A ('turn_to_target'): rotate to face the lateral waypoint
        #   Sub-phase B ('drive_to_target'): drive straight to that waypoint
        if self.state == 'align_front':

            # ── Sub-phase A: compute waypoint once, then rotate to face it ──
            if self._align_phase == 'turn_to_target':
                # Only read the marker when we haven't committed to a waypoint yet
                if self._align_target_yaw is None:
                    marker_age = (now - self.last_marker_time) if self.last_marker_time else 999
                    if marker_age > self.marker_timeout:
                        self.cmd_pub.publish(Twist())
                        self.get_logger().warn('Align: marker lost — waiting for marker...')
                        return

                    x = self.last_marker_x  # lateral offset in robot frame

                    if abs(x) < self.lateral_threshold:
                        self.cmd_pub.publish(Twist())
                        self.get_logger().info(
                            f'Already in front of marker (x={x:.3f}m) — turning to face...'
                        )
                        self.state = 'turn_face'
                        return

                    # Project lateral offset into world frame.
                    # Robot forward = (cos(yaw), sin(yaw)), so robot left = (-sin(yaw), cos(yaw)).
                    # Positive marker_x means the marker is to the robot's left.
                    self._align_target_world_x = (
                        self.current_x + x * (-math.sin(self.current_yaw))
                    )
                    self._align_target_world_y = (
                        self.current_y + x * math.cos(self.current_yaw)
                    )

                    dx = self._align_target_world_x - self.current_x
                    dy = self._align_target_world_y - self.current_y
                    self._align_target_yaw = math.atan2(dy, dx)   # heading toward waypoint
                    self._align_drive_dist = math.sqrt(dx ** 2 + dy ** 2)
                    self._align_spin_dir = 1 if x > 0 else -1

                    self.get_logger().info(
                        f'Align: lateral offset={x:.3f}m — '
                        f'turning {math.degrees(self._angle_diff(self._align_target_yaw, self.current_yaw)):+.1f}° '
                        f'then driving {self._align_drive_dist:.3f}m straight to waypoint.'
                    )

                yaw_err = self._angle_diff(self._align_target_yaw, self.current_yaw)
                if abs(yaw_err) < 0.05:
                    self._align_phase = 'drive_to_target'
                    self._align_drive_start_x = self.current_x
                    self._align_drive_start_y = self.current_y
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info('Align turn done — driving straight to waypoint...')
                    return

                cmd = Twist()
                cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 2.0 * yaw_err))
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f'Align turn: yaw_err={math.degrees(yaw_err):.1f}°')

            # ── Sub-phase B: drive straight (odometry) to the pre-computed waypoint ──
            elif self._align_phase == 'drive_to_target':
                dist = math.sqrt(
                    (self.current_x - self._align_drive_start_x) ** 2
                    + (self.current_y - self._align_drive_start_y) ** 2
                )
                if dist >= self._align_drive_dist:
                    # Reset for next time, then hand off to turn_face
                    self._align_phase = 'turn_to_target'
                    self._align_target_yaw = None
                    self._align_drive_dist = 0.0
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(
                        f'Waypoint reached ({dist:.3f}m) — turning to face marker...'
                    )
                    self.state = 'turn_face'
                    return

                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    f'Align drive: {dist:.3f}/{self._align_drive_dist:.3f}m'
                )

        # ── STEP 2: Rotate in place to face the marker squarely ──
        elif self.state == 'turn_face':
            marker_age = (now - self.last_marker_time) if self.last_marker_time else 999
            if marker_age > self.marker_timeout:
                # After lateral drive, marker is off to the side — rotate back to find it
                cmd = Twist()
                cmd.angular.z = -self._align_spin_dir * 0.2
                self.cmd_pub.publish(cmd)
                self.get_logger().warn('Turn face: marker lost — searching...')
                return

            x, z = self.last_marker_x, self.last_marker_z
            angle_error = math.atan2(x, max(z, 0.01))

            if abs(angle_error) < self.yaw_threshold:
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Facing marker — starting visual approach...')
                self.aligned_count = 0
                self.state = 'visual_approach'
                return

            cmd = Twist()
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 1.5 * angle_error))
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                f'Turn face: angle={math.degrees(angle_error):.1f}° x={x:.3f}m'
            )

        # ── STEP 3: Visual drive straight to stop_dist, odom fallback on marker loss ──
        elif self.state == 'visual_approach':
            marker_age = (now - self.last_marker_time) if self.last_marker_time else 999

            if marker_age > self.marker_timeout:
                if 0.0 < self.last_marker_z < 0.35:
                    # Close enough — commit to odom straight drive for the remainder
                    self.drive_distance = max(0.0, self.last_marker_z - self.stop_dist)
                    self.start_x, self.start_y = self.current_x, self.current_y
                    self.aligned_count = 0
                    self.state = 'odom_drive'
                    self.get_logger().info(
                        f'Marker lost at {self.last_marker_z:.3f}m — finishing on odom '
                        f'({self.drive_distance:.3f}m remaining).'
                    )
                else:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().warn('Visual approach: marker lost — waiting...')
                return

            z, x = self.last_marker_z, self.last_marker_x

            if z <= self.stop_dist:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f'Docked at {z:.3f}m!')
                self.state = 'docked'
                self.on_docked()
                return

            cmd = Twist()
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 2.0 * x))

            if abs(x) > self.x_threshold:
                # Still off-centre — creep forward while correcting
                cmd.linear.x = 0.02
                self.aligned_count = 0
            else:
                self.aligned_count += 1
                cmd.linear.x = min(self.lin_speed, max(0.02, 0.3 * (z - self.stop_dist)))
                self.get_logger().info(
                    f'Visual approach ({self.aligned_count}/{self.aligned_frames_needed}) '
                    f'z={z:.3f}m'
                )

            self.cmd_pub.publish(cmd)

        # ── STEP 4: Odom straight-drive fallback ──
        elif self.state == 'odom_drive':
            dist = math.sqrt(
                (self.current_x - self.start_x) ** 2
                + (self.current_y - self.start_y) ** 2
            )
            if dist >= self.drive_distance:
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Docked via odom!')
                self.state = 'docked'
                self.on_docked()
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)

    def on_docked(self):
        # Overridden by subclasses (Task_A_Main, Task_B_Main).
        pass

    def stop_robot(self):
        self.state = 'idle'
        self.aligned_count = 0
        self.last_marker_time = None
        self._align_phase = 'turn_to_target'
        self._align_target_yaw = None
        self._align_target_world_x = 0.0
        self._align_target_world_y = 0.0
        self._align_drive_dist = 0.0
        self.cmd_pub.publish(Twist())

    def _normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _angle_diff(self, target, current):
        return self._normalize_angle(target - current)