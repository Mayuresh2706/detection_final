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
        # idle -> drive_to_waypoint -> rotate_to_face -> visual_approach -> odom_drive -> docked
        #
        # drive_to_waypoint : lock on marker once, compute a waypoint at (marker_x, marker_z - approach_offset)
        #                     in world frame, turn to face it, drive straight to it via odometry
        # rotate_to_face    : rotate in place by the trigonometrically computed angle to face the marker
        #                     (face_heading = initial robot yaw when locked, since the waypoint was
        #                      placed directly along that axis from the robot to the marker)
        # visual_approach   : drive straight toward marker with lateral correction, stop at stop_dist
        # odom_drive        : if marker lost, drive last known z - stop_dist via odometry
        self.state = 'idle'

        # Tuning
        self.stop_dist = 0.05           # final stop distance from marker (m)
        self.approach_offset = 0.30     # waypoint is this far in front of the marker (m)
        self.yaw_threshold = 0.03       # angle threshold for rotation phases (rad)
        self.x_threshold = 0.04         # lateral threshold during visual_approach (m)
        self.lin_speed = 0.06           # max linear speed (m/s)
        self.ang_speed = 0.3            # max angular speed (rad/s)

        # Marker state
        self.last_marker_z = 0.0
        self.last_marker_x = 0.0
        self.last_marker_rvec_y = 0.0
        self.last_marker_rvec_z = 0.0
        self.last_marker_time = None
        self.marker_timeout = 0.3

        # Odom drive
        self.drive_distance = 0.0

        # Approach counter
        self.aligned_count = 0
        self.aligned_frames_needed = 5

        # Waypoint state — computed once when locking onto the marker
        #
        # Geometry explanation:
        #   Robot at (rx, ry) facing yaw θ sees marker at robot-frame (mx, mz).
        #   World frame:  marker  = (rx + mz·cosθ − mx·sinθ,  ry + mz·sinθ + mx·cosθ)
        #                 waypoint = (rx + (mz−offset)·cosθ − mx·sinθ,  ry + (mz−offset)·sinθ + mx·cosθ)
        #   The vector waypoint→marker is always (offset·cosθ, offset·sinθ), i.e. exactly the
        #   original robot heading θ regardless of lateral offset mx.
        #   Therefore face_heading = initial θ (no trigonometric lookup needed at runtime;
        #   the trig is baked into the waypoint placement).
        self._wp_computed = False
        self._wp_world_x = 0.0
        self._wp_world_y = 0.0
        self._wp_heading = 0.0          # world heading to turn toward before driving to waypoint
        self._wp_drive_dist = 0.0       # straight-line distance to waypoint
        self._face_heading = 0.0        # world heading to face marker once at waypoint (= initial yaw)
        self._wp_phase = 'turn'         # 'turn' first, then 'drive'
        self._wp_drive_start_x = 0.0
        self._wp_drive_start_y = 0.0

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
            self._wp_computed = False
            self._wp_phase = 'turn'
            self.state = 'drive_to_waypoint'
            self.get_logger().info('Activated — waiting for first marker reading to lock waypoint...')
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

        # ── STEP 1: Drive straight to the waypoint offset in front of the marker ──
        #
        # The waypoint is computed once from the first good marker reading.
        # We project the marker position (mx, mz) into world frame and subtract
        # approach_offset along the robot's forward axis to get the waypoint.
        # Sub-phase 'turn': rotate to face the waypoint direction.
        # Sub-phase 'drive': drive straight to it via odometry.
        if self.state == 'drive_to_waypoint':

            if not self._wp_computed:
                marker_age = (now - self.last_marker_time) if self.last_marker_time else 999
                if marker_age > self.marker_timeout:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().warn('Drive to waypoint: waiting for marker...')
                    return

                mx = self.last_marker_x   # lateral offset in robot frame
                mz = self.last_marker_z   # depth in robot frame
                yaw = self.current_yaw

                # Waypoint: (mz - approach_offset) forward, mx lateral, projected to world.
                # Robot forward in world: (cos(yaw), sin(yaw))
                # Robot left   in world: (-sin(yaw), cos(yaw))
                fwd = mz - self.approach_offset
                self._wp_world_x = self.current_x + fwd * math.cos(yaw) - mx * math.sin(yaw)
                self._wp_world_y = self.current_y + fwd * math.sin(yaw) + mx * math.cos(yaw)

                dx = self._wp_world_x - self.current_x
                dy = self._wp_world_y - self.current_y
                self._wp_heading    = math.atan2(dy, dx)
                self._wp_drive_dist = math.sqrt(dx ** 2 + dy ** 2)

                # After arriving at the waypoint, the marker is exactly approach_offset away
                # along the original robot heading (yaw). So face_heading = initial yaw.
                # Trig proof: waypoint→marker vector = (offset·cosθ, offset·sinθ) → atan2 = θ.
                self._face_heading = yaw

                self._wp_computed = True
                self.get_logger().info(
                    f'Waypoint locked: marker x={mx:.3f}m z={mz:.3f}m | '
                    f'waypoint ({self._wp_world_x:.3f}, {self._wp_world_y:.3f}) '
                    f'dist={self._wp_drive_dist:.3f}m | '
                    f'face_heading={math.degrees(self._face_heading):.1f}°'
                )

            # Sub-phase A: turn to face the waypoint direction
            if self._wp_phase == 'turn':
                yaw_err = self._angle_diff(self._wp_heading, self.current_yaw)
                if abs(yaw_err) < self.yaw_threshold:
                    self._wp_phase = 'drive'
                    self._wp_drive_start_x = self.current_x
                    self._wp_drive_start_y = self.current_y
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(
                        f'Waypoint turn done — driving {self._wp_drive_dist:.3f}m straight...'
                    )
                    return

                cmd = Twist()
                cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 2.0 * yaw_err))
                self.cmd_pub.publish(cmd)
                self.get_logger().info(f'Waypoint turn: yaw_err={math.degrees(yaw_err):.1f}°')

            # Sub-phase B: drive straight to the waypoint via odometry
            elif self._wp_phase == 'drive':
                dist = math.sqrt(
                    (self.current_x - self._wp_drive_start_x) ** 2
                    + (self.current_y - self._wp_drive_start_y) ** 2
                )
                if dist >= self._wp_drive_dist:
                    self.cmd_pub.publish(Twist())
                    self.get_logger().info(
                        f'Waypoint reached ({dist:.3f}m) — rotating to face marker...'
                    )
                    self.state = 'rotate_to_face'
                    return

                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)
                self.get_logger().info(
                    f'Waypoint drive: {dist:.3f}/{self._wp_drive_dist:.3f}m'
                )

        # ── STEP 2: Rotate in place to face the marker ──
        #
        # face_heading = initial robot yaw (locked in step 1).
        # This works because the waypoint was placed along the original approach axis,
        # so the marker is always directly ahead at approach_offset distance.
        elif self.state == 'rotate_to_face':
            yaw_err = self._angle_diff(self._face_heading, self.current_yaw)
            if abs(yaw_err) < self.yaw_threshold:
                self.cmd_pub.publish(Twist())
                self.aligned_count = 0
                marker_age = (now - self.last_marker_time) if self.last_marker_time else 999
                if marker_age <= self.marker_timeout:
                    self.get_logger().info('Facing marker — starting visual approach...')
                    self.state = 'visual_approach'
                else:
                    # Marker not visible: we are approach_offset in front of it, drive that minus stop_dist
                    self.drive_distance = max(0.0, self.approach_offset - self.stop_dist)
                    self.start_x, self.start_y = self.current_x, self.current_y
                    self.get_logger().info(
                        f'Marker not visible after rotation — odom drive {self.drive_distance:.3f}m'
                    )
                    self.state = 'odom_drive'
                return

            cmd = Twist()
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 2.0 * yaw_err))
            self.cmd_pub.publish(cmd)
            self.get_logger().info(f'Rotate to face: yaw_err={math.degrees(yaw_err):.1f}°')

        # ── STEP 3: Visual approach with lateral correction ──
        #
        # If marker is lost mid-approach, fall back to odom using last known z.
        elif self.state == 'visual_approach':
            marker_age = (now - self.last_marker_time) if self.last_marker_time else 999

            if marker_age > self.marker_timeout:
                self.drive_distance = max(0.0, self.last_marker_z - self.stop_dist)
                self.start_x, self.start_y = self.current_x, self.current_y
                self.aligned_count = 0
                self.state = 'odom_drive'
                self.get_logger().info(
                    f'Marker lost at z={self.last_marker_z:.3f}m — '
                    f'finishing on odom ({self.drive_distance:.3f}m remaining).'
                )
                return

            z, x = self.last_marker_z, self.last_marker_x

            if z <= self.stop_dist:
                self.cmd_pub.publish(Twist())
                self.get_logger().info(f'Docked at z={z:.3f}m!')
                self.state = 'docked'
                self.on_docked()
                return

            cmd = Twist()
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 2.0 * x))

            if abs(x) > self.x_threshold:
                cmd.linear.x = 0.02   # creep while correcting lateral error
                self.aligned_count = 0
            else:
                self.aligned_count += 1
                cmd.linear.x = min(self.lin_speed, max(0.02, 0.3 * (z - self.stop_dist)))
                self.get_logger().info(
                    f'Visual approach ({self.aligned_count}/{self.aligned_frames_needed}) z={z:.3f}m'
                )

            self.cmd_pub.publish(cmd)

        # ── STEP 4: Odom fallback — drive pre-computed distance straight ahead ──
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
        self._wp_computed = False
        self._wp_phase = 'turn'
        self.cmd_pub.publish(Twist())

    def _normalize_angle(self, angle):
        while angle > math.pi:  angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _angle_diff(self, target, current):
        return self._normalize_angle(target - current)