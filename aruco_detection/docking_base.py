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
        self.state = 'idle'
        self.odom_substate = 'approach_normal' # approach_normal -> rotate_to_normal -> drive_in

        # Odometry state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Tuning
        self.stop_dist = 0.05
        self.lin_speed = 0.06
        self.ang_gain = 2.0
        self.ang_speed = 0.3
        self.marker_timeout = 0.3

        # Last known marker data
        self.last_marker_z = 0.0       # Forward (Robot X)
        self.last_marker_x = 0.0       # Lateral (Robot Y)
        self.last_marker_bearing = 0.0 # Angle of marker normal relative to Robot X
        self.last_marker_time = None

        # Odom targets (calculated at moment of marker loss)
        self.target_point_x = 0.0 
        self.target_point_y = 0.0
        self.dist_to_dock = 0.0
        self.odom_start_x = 0.0
        self.odom_start_y = 0.0
        self.odom_start_yaw = 0.0

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.status_pub = self.create_publisher(String, 'task_status', 10)

        # Subscriptions
        self.create_subscription(Bool, '/dock_active', self.active_cb, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(PoseStamped, 'target_3d', self.marker_callback, 10)
        self.create_subscription(Float32, 'marker_normal_angle', self.bearing_callback, 10)

        self.drive_timer = self.create_timer(0.05, self.drive_callback)

    def bearing_callback(self, msg):
        # We only update bearing while we can see the marker
        if self.state == 'docking':
            self.last_marker_bearing = msg.data

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def marker_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)
        if self.dock_marker_id is not None and marker_id != self.dock_marker_id:
            return
        if self.dock_marker_id is None and marker_id not in self.target_ids:
            return

        if not self.is_active: return

        if self.dock_marker_id is None:
            self.dock_marker_id = marker_id
            self.get_logger().info(f'Locked onto marker {marker_id}')

        self.last_marker_z = msg.pose.position.x
        self.last_marker_x = msg.pose.position.y
        self.last_marker_time = self.get_clock().now().nanoseconds / 1e9

    def drive_callback(self):
        if not self.is_active:
            self.state = 'idle'
            return

        now = self.get_clock().now().nanoseconds / 1e9
        marker_age = (now - self.last_marker_time) if self.last_marker_time else 999.0

        if self.state == 'docking':
            if marker_age > self.marker_timeout:
                if self.last_marker_time is not None and 0.0 < self.last_marker_z < 0.6:
                    self._prepare_odom_navigation()
                    self.state = 'odom_drive'
                else:
                    self._stop_cmd()
                return

            # Visual Servoing
            z, x = self.last_marker_z, self.last_marker_x
            if z <= self.stop_dist:
                self._finish_docking()
                return

            cmd = Twist()
            cmd.linear.x = min(self.lin_speed, max(0.02, 0.3 * (z - self.stop_dist)))
            cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, self.ang_gain * x))
            self.cmd_pub.publish(cmd)

        elif self.state == 'odom_drive':
            if marker_age <= self.marker_timeout:
                self.state = 'docking'
                self.odom_substate = 'approach_normal'
                return
            
            self._execute_odom_navigation()

    def _prepare_odom_navigation(self):
        """Calculates the right-triangle path based on the marker normal."""
        bearing_rad = math.radians(self.last_marker_bearing)
        m_x, m_y = self.last_marker_z, self.last_marker_x

        # Normal line vector (pointing away from marker face)
        nx, ny = math.cos(bearing_rad), math.sin(bearing_rad)
        # Tangent vector (the line the robot needs to intersect)
        tx, ty = -ny, nx

        # Signed distance from marker to the intersection point along the tangent
        # Projecting the vector from marker to robot onto the tangent
        t = -m_x * tx - m_y * ty

        # Target point in robot's current local frame
        self.target_point_x = m_x + t * tx
        self.target_point_y = m_y + t * ty
        self.dist_to_dock = abs(t) # Distance to travel once turned 90 deg

        # Store world-frame start for odom tracking
        self.odom_start_x = self.current_x
        self.odom_start_y = self.current_y
        self.odom_start_yaw = self.current_yaw
        self.odom_substate = 'approach_normal'
        
        self.get_logger().info(f'Switching to Normal Approach. Bearing: {self.last_marker_bearing:.1f}deg')

    def _execute_odom_navigation(self):
        # Calculate how much we've moved in the "world" since starting odom drive
        dx_total = self.current_x - self.odom_start_x
        dy_total = self.current_y - self.odom_start_y
        
        # Transform moved distance back into the frame we had when we lost the marker
        # This gives us our current position relative to the 'loss point'
        cos_s = math.cos(self.odom_start_yaw)
        sin_s = math.sin(self.odom_start_yaw)
        rel_x = dx_total * cos_s + dy_total * sin_s
        rel_y = -dx_total * sin_s + dy_total * cos_s

        if self.odom_substate == 'approach_normal':
            dx = self.target_point_x - rel_x
            dy = self.target_point_y - rel_y
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < 0.02:
                self.odom_substate = 'rotate_to_normal'
                # New odom anchor for the next leg
                self.odom_start_x, self.odom_start_y = self.current_x, self.current_y
            else:
                target_heading = math.atan2(dy, dx)
                self._drive_to_local_pose(dist, target_heading)

        elif self.odom_substate == 'rotate_to_normal':
            # Target is to face the marker directly (bearing direction)
            target_yaw = self._normalize_angle(self.odom_start_yaw + math.radians(self.last_marker_bearing) + math.pi)
            error = self._normalize_angle(target_yaw - self.current_yaw)
            
            if abs(error) < 0.05:
                self.odom_substate = 'drive_in'
                self.odom_start_x, self.odom_start_y = self.current_x, self.current_y
            else:
                cmd = Twist()
                cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 1.5 * error))
                self.cmd_pub.publish(cmd)

        elif self.odom_substate == 'drive_in':
            dist_moved = math.sqrt((self.current_x - self.odom_start_x)**2 + (self.current_y - self.odom_start_y)**2)
            remaining = (self.dist_to_dock - self.stop_dist) - dist_moved
            
            if remaining <= 0:
                self._finish_docking()
            else:
                cmd = Twist()
                cmd.linear.x = self.lin_speed
                self.cmd_pub.publish(cmd)

    def _drive_to_local_pose(self, dist, local_angle):
        cmd = Twist()
        cmd.linear.x = min(self.lin_speed, 0.5 * dist)
        cmd.angular.z = max(-self.ang_speed, min(self.ang_speed, 2.0 * local_angle))
        self.cmd_pub.publish(cmd)

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _finish_docking(self):
        self._stop_cmd()
        self.state = 'docked'
        msg = String()
        msg.data = 'DOCKED'
        self.status_pub.publish(msg)
        self.get_logger().info('Docked successfully!')

    def _stop_cmd(self):
        self.cmd_pub.publish(Twist())

    def active_cb(self, msg):
        self.is_active = msg.data
        if self.is_active:
            self.state = 'docking'
            self.dock_marker_id = None
        else:
            self.state = 'idle'
            self._stop_cmd()

# ... (main remains the same)

def main(args=None):
    rclpy.init(args=args)
    node = DockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Manual Shutdown')
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
