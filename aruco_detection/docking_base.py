import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32


class DockingNode(Node):
    def __init__(self):
        super().__init__('docking_node')

        self.dock_marker_id = None
        self.target_ids = {1, 2}

        # Odometry state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # State machine:
        #   idle        -> not active
        #   locate      -> active, waiting for first marker sighting (plan once)
        #   go_to_normal-> driving to closest point on the marker's normal line
        #   search      -> turning to face the marker; if seen, switch to visual
        #   visual      -> visual servo toward marker until stop_dist
        #   blind_drive -> drive straight the pre-computed distance, no marker
        #   docked
        self.state = 'idle'

        # Tuning
        self.stop_dist = 0.10
        self.lin_speed = 0.08
        self.ang_gain = 2.0
        self.ang_speed = 0.3
        self.marker_timeout = 0.3
        self.align_tolerance = 0.03     # radians (~5 deg)
        self.reach_tolerance = 0.03     # meters — "arrived" at normal line point
        self.min_lin_speed = 0.03       # floor to overcome static friction
        self.dock_timeout = 60.0

        # Latest marker data
        self.last_marker_z = 0.0
        self.last_marker_x = 0.0
        self.last_marker_bearing = 0.0
        self.last_marker_time = None
        self.last_bearing_time = None

        # Pose snapshot at first sighting — used for the one-shot plan
        self.snap_x = 0.0
        self.snap_y = 0.0
        self.snap_yaw = 0.0

        # One-shot plan (computed on first marker sighting, never updated)
        self.plan_valid = False
        self.target_x = 0.0         # closest point on normal line (odom frame)
        self.target_y = 0.0
        self.final_yaw = 0.0        # yaw to face marker from target point
        self.final_dist = 0.0       # distance from target point to marker
        self.aimed_at_waypoint = False  # set True after pivoting to face waypoint

        self.odom_start_x = 0.0     # set when blind_drive begins
        self.odom_start_y = 0.0

        self.dock_activated_time = None
        self.is_active = False

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'task_status', 10)

        # Subscriptions
        self.create_subscription(Bool, '/dock_active', self.active_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, 'target_3d', self.marker_callback, 10)
        self.create_subscription(Float32, 'marker_normal_angle', self.bearing_callback, 10)

        self.drive_timer = self.create_timer(0.05, self.drive_callback)
        self.get_logger().info('Docking node ready')

    def bearing_callback(self, msg):
        if self.is_active:
            self.last_marker_bearing = msg.data
            self.last_bearing_time = self.get_clock().now().nanoseconds / 1e9

    def active_cb(self, msg):
        was_active = self.is_active
        self.is_active = msg.data
        if self.is_active and not was_active:
            self.dock_marker_id = None
            self.last_marker_time = None
            self.plan_valid = False
            self.state = 'locate'
            self.dock_activated_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info('Activated')
        elif not self.is_active and was_active:
            self.state = 'idle'
            self.dock_activated_time = None
            self._stop_cmd()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny, cosy)

    def marker_callback(self, msg):
        parts = msg.header.frame_id.split(':', 1)
        if len(parts) != 2:
            return
        try:
            marker_id = int(parts[1])
        except ValueError:
            return

        if self.dock_marker_id is not None and marker_id != self.dock_marker_id:
            return
        if self.dock_marker_id is None and marker_id not in self.target_ids:
            return
        if not self.is_active:
            return

        if self.dock_marker_id is None:
            self.dock_marker_id = marker_id
            self.get_logger().info(f'Locked onto marker {marker_id}')

        self.last_marker_z = msg.pose.position.x
        self.last_marker_x = msg.pose.position.y
        self.last_marker_time = self.get_clock().now().nanoseconds / 1e9

        # Plan once on first sighting — but only if bearing has arrived too,
        # otherwise we'd plan with bearing=0 and the "normal line" would line
        # up with the robot's heading (→ a straight-line approach).
        if not self.plan_valid and self.last_marker_z > 0.0 and self.last_bearing_time is not None:
            self.snap_x = self.current_x
            self.snap_y = self.current_y
            self.snap_yaw = self.current_yaw
            self._plan_normal_approach()
            self.plan_valid = True
            self.aimed_at_waypoint = False
            self.state = 'go_to_normal'
            self.get_logger().info(
                f'Plan: marker(z={self.last_marker_z:.2f}, x={self.last_marker_x:.2f}, '
                f'bearing={self.last_marker_bearing:.1f}°) '
                f'→ waypoint=({self.target_x:.2f},{self.target_y:.2f}) '
                f'from robot=({self.snap_x:.2f},{self.snap_y:.2f}, '
                f'yaw={math.degrees(self.snap_yaw):.0f}°), '
                f'final_yaw={math.degrees(self.final_yaw):.0f}°, '
                f'final_dist={self.final_dist:.2f}m'
            )

    def drive_callback(self):
        if not self.is_active:
            return

        if self.state == 'docked':
            self.cmd_pub.publish(Twist())
            return

        now = self.get_clock().now().nanoseconds / 1e9

        # Global watchdog
        if self.dock_activated_time is not None:
            if (now - self.dock_activated_time) > self.dock_timeout:
                self._stop_cmd()
                self.state = 'idle'
                self.dock_activated_time = None
                fail = String()
                fail.data = 'DOCK_FAILED'
                self.status_pub.publish(fail)
                self.get_logger().warn('Docking timeout — giving up')
                return

        marker_age = (now - self.last_marker_time) if self.last_marker_time else 999.0
        marker_fresh = marker_age <= self.marker_timeout

        if self.state == 'locate':
            # Sit still until marker_callback captures first sighting and plans.
            self._stop_cmd()
            return

        if self.state == 'go_to_normal':
            dx = self.target_x - self.current_x
            dy = self.target_y - self.current_y
            dist = math.hypot(dx, dy)

            if dist < self.reach_tolerance:
                self.state = 'search'
                self.get_logger().info(
                    f'Reached normal line (dist={dist:.3f}m) — turning toward marker '
                    f'(yaw_err={math.degrees(self._norm(self.final_yaw - self.current_yaw)):.0f}°)'
                )
                return

            target_yaw = math.atan2(dy, dx)
            yaw_err = self._norm(target_yaw - self.current_yaw)

            cmd = Twist()
            if not self.aimed_at_waypoint:
                # Phase 1: pivot in place until pointing at the waypoint.
                if abs(yaw_err) < self.align_tolerance:
                    self.aimed_at_waypoint = True
                    self.get_logger().info(
                        f'Aimed at waypoint (yaw_err={math.degrees(yaw_err):.1f}°) — driving straight'
                    )
                else:
                    cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, self.ang_gain * yaw_err))
                self.cmd_pub.publish(cmd)
                return

            # Phase 2: drive straight. Light yaw correction to counter drift,
            # but we keep moving forward — don't re-enter pivot mode.
            cmd.linear.x = min(self.lin_speed, max(self.min_lin_speed, 0.4 * dist))
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, self.ang_gain * yaw_err))
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'search':
            # Rotate toward the planned final_yaw (≈90° from approach). If the
            # marker pops into view during the turn, chase it. Otherwise commit
            # to the blind drive using the one-shot plan.
            if marker_fresh and self.last_marker_z > 0.0:
                self.state = 'visual'
                self.get_logger().info('Marker reacquired — visual servo')
                return

            yaw_err = self._norm(self.final_yaw - self.current_yaw)
            if abs(yaw_err) < self.align_tolerance:
                if self.final_dist <= self.stop_dist:
                    self._dock()
                    return
                self.state = 'blind_drive'
                self.odom_start_x = self.current_x
                self.odom_start_y = self.current_y
                self.get_logger().info(
                    f'Marker not found after turn — blind drive {self.final_dist:.2f}m'
                )
                return

            cmd = Twist()
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 3.0 * yaw_err))
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'visual':
            if not marker_fresh:
                # Lost mid-servo → fall back using the last seen distance.
                self.state = 'blind_drive'
                self.odom_start_x = self.current_x
                self.odom_start_y = self.current_y
                self.final_dist = self.last_marker_z
                self.get_logger().info(
                    f'Marker lost during servo — blind drive {self.final_dist:.2f}m'
                )
                return

            z, x = self.last_marker_z, self.last_marker_x
            self.get_logger().info(
                f'Visual: z={z:.3f} x={x:.3f} bearing={self.last_marker_bearing:.1f}° '
                f'age={marker_age:.2f}s',
                throttle_duration_sec=0.5
            )
            if z <= self.stop_dist:
                self._dock()
                return

            cmd = Twist()
            # Scale down forward speed when laterally misaligned so the
            # robot prioritises centering before closing distance.
            x_scale = max(0.0, 1.0 - 4.0 * abs(x) / max(z, 0.01))
            cmd.linear.x = x_scale * min(self.lin_speed, max(self.min_lin_speed, 0.3 * (z - self.stop_dist)))
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, self.ang_gain * x))
            self.cmd_pub.publish(cmd)
            return

        if self.state == 'blind_drive':
            driven = math.hypot(self.current_x - self.odom_start_x,
                                self.current_y - self.odom_start_y)
            if driven >= self.final_dist - self.stop_dist:
                self._dock()
                return
            cmd = Twist()
            cmd.linear.x = self.lin_speed
            self.cmd_pub.publish(cmd)

    def _plan_normal_approach(self):
        x0, y0, yaw0 = self.snap_x, self.snap_y, self.snap_yaw
        z = self.last_marker_z
        x = self.last_marker_x
        beta = math.radians(self.last_marker_bearing)

        # Marker position in odom frame
        mx = x0 + z * math.cos(yaw0) - x * math.sin(yaw0)
        my = y0 + z * math.sin(yaw0) + x * math.cos(yaw0)

        # Normal line through the marker
        normal_yaw = yaw0 + beta
        nx, ny = math.cos(normal_yaw), math.sin(normal_yaw)

        # Closest point on that line to the robot
        t = (x0 - mx) * nx + (y0 - my) * ny
        self.target_x = mx + t * nx
        self.target_y = my + t * ny
        self.final_dist = abs(t)
        self.final_yaw = self._norm(normal_yaw + (math.pi if t > 0 else 0.0))

    def _norm(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def _dock(self):
        self._stop_cmd()
        self.state = 'docked'
        self.dock_activated_time = None
        msg = String()
        msg.data = 'DOCKED'
        self.status_pub.publish(msg)
        self.get_logger().info('Docked!')

    def _stop_cmd(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
