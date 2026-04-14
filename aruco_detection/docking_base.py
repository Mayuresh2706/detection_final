# import math
# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, PoseStamped
# from nav_msgs.msg import Odometry
# from std_msgs.msg import Bool, String


# class DockingBase(Node):
#     def __init__(self, node_name, dock_marker_id):
#         super().__init__(node_name)

#         self.dock_marker_id = dock_marker_id

#         # Odometry state
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.start_x = 0.0
#         self.start_y = 0.0
#         self.current_yaw = 0.0

#         # State machine: idle -> aligning_angle -> visual_servo -> odom_drive -> docked
#         self.state = 'idle'

#         # Tuning
#         self.stop_dist = 0.10
#         self.x_threshold = 0.04
#         self.angle_threshold = 0.05  # ~3°
#         self.lin_speed = 0.04
#         self.ang_speed = 0.3
#         self.ang_kp = 2.0

#         # Marker state — never consumed, always latest value
#         self.last_marker_z = 0.0
#         self.last_marker_x = 0.0
#         self.marker_visible = False
#         self.last_marker_time = None
#         self.marker_timeout = 0.3  # seconds

#         # Odom drive state
#         self.drive_distance = 0.0

#         # Alignment stability counter
#         self.aligned_count = 0
#         self.aligned_frames_needed = 5

#         # Activation
#         self.is_active = False
#         self._logged_waiting = False

#         # Publishers
#         self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
#         self.status_pub = self.create_publisher(String, 'task_status', 10)
#         self.fire_pub = self.create_publisher(Bool, '/fire', 10)

#         # Subscriptions
#         self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
#         self.create_subscription(PoseStamped, 'target_3d', self.marker_callback, 10)

#         self.drive_timer = self.create_timer(0.05, self.drive_callback)

#         self.get_logger().info(
#             f'Node ready — dock_marker_id={self.dock_marker_id}. '
#             f'Waiting for activation...'
#         )

#     def active_cb(self, msg):
#         self.is_active = msg.data
#         if self.is_active:
#             self._logged_waiting = False
#             self.aligned_count = 0
#             self.marker_visible = False
#             self.last_marker_time = None
#             self.last_marker_z = 0.0
#             self.last_marker_x = 0.0
#             self.state = 'aligning_angle'
#             self.get_logger().info('Activated — correcting heading first...')
#         else:
#             self.get_logger().info('Deactivated.')
#             self.stop_robot()

#     def odom_callback(self, msg):
#         self.current_x = msg.pose.pose.position.x
#         self.current_y = msg.pose.pose.position.y
#         q = msg.pose.pose.orientation
#         siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
#         cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
#         self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

#     def marker_callback(self, msg):
#         marker_id = int(msg.pose.orientation.w)
#         if marker_id != self.dock_marker_id:
#             return
#         if not self.is_active:
#             if not self._logged_waiting:
#                 self.get_logger().info(f'Marker {marker_id} seen but not active.')
#                 self._logged_waiting = True
#             return

#         self.last_marker_z = msg.pose.position.x
#         self.last_marker_x = msg.pose.position.y
#         self.marker_visible = True
#         self.last_marker_time = self.get_clock().now().nanoseconds / 1e9

#     def drive_callback(self):
#         if not self.is_active:
#             self.state = 'idle'
#             return

#         if self.state == 'docked':
#             self.cmd_pub.publish(Twist())
#             return

#         now = self.get_clock().now().nanoseconds / 1e9
#         marker_age = (now - self.last_marker_time) if self.last_marker_time else 999

#         # ── PHASE 0: rotate in place to face marker ──
#         if self.state == 'aligning_angle':
#             if marker_age > self.marker_timeout:
#                 self.cmd_pub.publish(Twist())
#                 return

#             z = self.last_marker_z
#             x = self.last_marker_x
#             angle_error = math.atan2(x, z)

#             if abs(angle_error) < self.angle_threshold:
#                 self.get_logger().info(
#                     f'Heading corrected! Starting visual servo...'
#                 )
#                 self.aligned_count = 0
#                 self.state = 'visual_servo'
#             else:
#                 cmd = Twist()
#                 cmd.angular.z = max(-self.ang_speed,
#                                     min(self.ang_speed, 2.0 * angle_error))
#                 self.cmd_pub.publish(cmd)
#                 self.get_logger().info(
#                     f'Correcting heading: {math.degrees(angle_error):.1f}°'
#                 )

#         # ── PHASE 1: visual servo — drive in while keeping centered ──
#         elif self.state == 'visual_servo':
#             if marker_age > self.marker_timeout:
#                 if 0.0 < self.last_marker_z < 0.35:
#                     # Close enough — switch to odom
#                     self.drive_distance = max(0.0, self.last_marker_z - self.stop_dist)
#                     self.start_x = self.current_x
#                     self.start_y = self.current_y
#                     self.aligned_count = 0
#                     self.state = 'odom_drive'
#                     self.get_logger().info(
#                         f'Marker lost at {self.last_marker_z:.3f}m — '
#                         f'odom driving {self.drive_distance:.3f}m'
#                     )
#                 else:
#                     self.aligned_count = 0
#                     self.cmd_pub.publish(Twist())
#                     self.get_logger().warn('Marker lost too far out — waiting...')
#                 return

#             z = self.last_marker_z
#             x = self.last_marker_x

#             if z <= self.stop_dist:
#                 self.cmd_pub.publish(Twist())
#                 self.get_logger().info(f'Docked at {z:.3f}m via visual servo!')
#                 self.state = 'docked'
#                 self.on_docked()
#                 return

#             cmd = Twist()
#             cmd.angular.z = max(-self.ang_speed,
#                                 min(self.ang_speed, self.ang_kp * x))

#             if abs(x) > self.x_threshold:
#                 cmd.linear.x = 0.02
#                 self.aligned_count = 0
#             else:
#                 self.aligned_count += 1
#                 cmd.linear.x = min(self.lin_speed,
#                                    max(0.02, 0.3 * (z - self.stop_dist)))
#                 self.get_logger().info(
#                     f'Aligned ({self.aligned_count}/{self.aligned_frames_needed}) '
#                     f'z={z:.3f}m x={x:.3f}m'
#                 )

#             self.cmd_pub.publish(cmd)

#         # ── PHASE 2: odom straight drive for last bit ──
#         elif self.state == 'odom_drive':
#             dist_traveled = math.sqrt(
#                 (self.current_x - self.start_x) ** 2 +
#                 (self.current_y - self.start_y) ** 2
#             )
#             if dist_traveled >= self.drive_distance:
#                 self.cmd_pub.publish(Twist())
#                 self.get_logger().info('Docked via odom!')
#                 self.state = 'docked'
#                 self.on_docked()
#             else:
#                 cmd = Twist()
#                 cmd.linear.x = self.lin_speed
#                 self.cmd_pub.publish(cmd)

#     def on_docked(self):
#         pass

#     def stop_robot(self):
#         self.state = 'idle'
#         self.marker_visible = False
#         self.aligned_count = 0
#         self.last_marker_time = None
#         self.cmd_pub.publish(Twist())

#     def _angle_diff(self, current, start):
#         diff = current - start
#         while diff > math.pi:
#             diff -= 2 * math.pi
#         while diff < -math.pi:
#             diff += 2 * math.pi
#         return diff


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
        # idle -> fix_lateral -> fix_yaw -> visual_servo -> odom_drive -> docked
        self.state = 'idle'

        # Tuning
        self.stop_dist = 0.10
        self.lateral_threshold = 0.05  
        self.yaw_threshold = 0.05      
        self.x_threshold = 0.04        
        self.lin_speed = 0.04
        self.ang_speed = 0.3

        # Marker state
        self.last_marker_z = 0.0
        self.last_marker_x = 0.0
        self.last_marker_rvec_y = 0.0
        self.last_marker_rvec_z = 0.0
        self.last_marker_time = None
        self.marker_timeout = 0.3
        self.marker_lost_timeout = 2.0

        # --- Blind Alignment Snapshot Targets ---
        self.target_yaw_step1 = None
        self.target_yaw_step2 = None

        # Odom drive
        self.drive_distance = 0.0

        # Alignment counter
        self.aligned_count = 0
        self.aligned_frames_needed = 5

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
            self.target_yaw_step1 = None
            self.target_yaw_step2 = None
            self.state = 'fix_lateral'
            self.get_logger().info('Activated — Taking snapshots for blind alignment...')
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

        self.last_marker_z = msg.pose.position.x
        self.last_marker_x = msg.pose.position.y
        self.last_marker_rvec_y = msg.pose.orientation.y
        self.last_marker_rvec_z = msg.pose.orientation.z
        self.last_marker_time = self.get_clock().now().nanoseconds / 1e9

        # Take the snapshots once during Step 1
        if self.state == 'fix_lateral' and self.target_yaw_step1 is None:
            # Angle needed to point the nose exactly at the marker
            angle_to_marker = math.atan2(self.last_marker_x, self.last_marker_z)
            self.target_yaw_step1 = self._normalize_angle(self.current_yaw + angle_to_marker)
            
            # Angle needed to be parallel/square with the dock face
            self.target_yaw_step2 = self._normalize_angle(self.target_yaw_step1 + self.last_marker_rvec_y)
            
            self.get_logger().info(
                f'Snapshots Locked! Target1: {math.degrees(self.target_yaw_step1):.1f}°, '
                f'Target2: {math.degrees(self.target_yaw_step2):.1f}°'
            )

    def drive_callback(self):
        if not self.is_active:
            self.state = 'idle'
            return

        if self.state == 'docked':
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # ── STEP 1: Blind rotation to Nose-on ──
        if self.state == 'fix_lateral':
            if self.target_yaw_step1 is None:
                self.cmd_pub.publish(Twist())
                return

            error = self._angle_diff(self.target_yaw_step1, self.current_yaw)

            if abs(error) < self.lateral_threshold:
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Step 1 Blind Alignment Complete.')
                self.state = 'fix_yaw'
                return

            cmd = Twist()
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 1.5 * error))
            self.cmd_pub.publish(cmd)

        # ── STEP 2: Blind rotation to Square-on ──
        elif self.state == 'fix_yaw':
            if self.target_yaw_step2 is None:
                self.state = 'fix_lateral' # Fallback
                return

            error = self._angle_diff(self.target_yaw_step2, self.current_yaw)

            if abs(error) < self.yaw_threshold:
                self.cmd_pub.publish(Twist())
                self.get_logger().info('Step 2 Blind Alignment Complete.')
                self.state = 'visual_servo'
                return

            cmd = Twist()
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 1.5 * error))
            self.cmd_pub.publish(cmd)

        # ── STEP 3: Visual Servo (Live) ──
        elif self.state == 'visual_servo':
            marker_age = (now - self.last_marker_time) if self.last_marker_time else 999

            if marker_age > self.marker_timeout:
                if 0.0 < self.last_marker_z < 0.35:
                    self.drive_distance = max(0.0, self.last_marker_z - self.stop_dist)
                    self.start_x, self.start_y = self.current_x, self.current_y
                    self.aligned_count = 0
                    self.state = 'odom_drive'
                    self.get_logger().info(f'Marker lost at {self.last_marker_z:.3f}m — Finishing on Odom.')
                else:
                    self.aligned_count = 0
                    self.cmd_pub.publish(Twist())
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
                cmd.linear.x = 0.02
                self.aligned_count = 0
            else:
                self.aligned_count += 1
                cmd.linear.x = min(self.lin_speed, max(0.02, 0.3 * (z - self.stop_dist)))
                self.get_logger().info(f'Aligned ({self.aligned_count}/{self.aligned_frames_needed}) z={z:.3f}m')

            self.cmd_pub.publish(cmd)

        # ── STEP 4: Odom straight drive ──
        elif self.state == 'odom_drive':
            dist = math.sqrt((self.current_x - self.start_x)**2 + (self.current_y - self.start_y)**2)
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
        # Restore full mission signals
        fire_msg = Bool()
        fire_msg.data = True
        self.fire_pub.publish(fire_msg)
        
        status_msg = String()
        status_msg.data = "docked"
        self.status_pub.publish(status_msg)

    def stop_robot(self):
        self.state = 'idle'
        self.aligned_count = 0
        self.last_marker_time = None
        self.target_yaw_step1 = None
        self.target_yaw_step2 = None
        self.cmd_pub.publish(Twist())

    def _normalize_angle(self, angle):
        while angle > math.pi: angle -= 2 * math.pi
        while angle < -math.pi: angle += 2 * math.pi
        return angle

    def _angle_diff(self, target, current):
        diff = target - current
        return self._normalize_angle(diff)