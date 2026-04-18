import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped


class DockingNoNav(Node):
    def __init__(self):
        super().__init__('docking_no_nav')

        # IDs 1 and 2 are valid docking targets; everything else is ignored
        self.target_ids = {1, 2}
        self.state = 'WAITING'
        self.detected_marker_id = None

        self.dock_pub   = self.create_publisher(Bool, '/dock_active', 10)
        self.task_a_pub = self.create_publisher(Bool, '/task_a_active', 10)
        self.task_b_pub = self.create_publisher(Bool, '/task_b_active', 10)

        self.create_subscription(PoseStamped, 'target_3d', self.aruco_callback, 10)
        self.create_subscription(String, 'task_status', self.task_status_cb, 10)

        # Docking watchdog: if docking_base never reports back within this window,
        # assume it's dead/stuck and reset.
        self.docking_start_time = None
        self.docking_timeout = 75.0
        self.create_timer(1.0, self._docking_watchdog)

        self.get_logger().info('DockingNoNav Initialized - WAITING for ArUco marker')

    def aruco_callback(self, msg):
        parts = msg.header.frame_id.split(':', 1)
        if len(parts) != 2:
            return
        try:
            marker_id = int(parts[1])
        except ValueError:
            return

        if marker_id not in self.target_ids:
            return

        if self.state != 'WAITING':
            return

        self.get_logger().info(f'Marker ID {marker_id} detected! Starting docking...')
        self.detected_marker_id = marker_id
        self.state = 'DOCKING'
        self.docking_start_time = self.get_clock().now().nanoseconds / 1e9
        self.dock_pub.publish(Bool(data=True))

    def task_status_cb(self, msg):
        if msg.data == 'DOCKED' and self.state == 'DOCKING':
            self.dock_pub.publish(Bool(data=False))
            self.docking_start_time = None
            task_label = 'A' if self.detected_marker_id == 1 else 'B'
            self.get_logger().info(f'Docking complete. Activating Task {task_label}...')
            self.state = 'TASKING'
            if self.detected_marker_id == 1:
                self.task_a_pub.publish(Bool(data=True))
            else:
                self.task_b_pub.publish(Bool(data=True))

        elif msg.data == 'DOCK_FAILED' and self.state == 'DOCKING':
            self.get_logger().warn('docking_base reported DOCK_FAILED — resetting')
            self.dock_pub.publish(Bool(data=False))
            self.reset()

        elif msg.data == 'SUCCESS' and self.state == 'TASKING':
            task_label = 'A' if self.detected_marker_id == 1 else 'B'
            self.get_logger().info(f'Task {task_label} complete. Ready for next marker.')
            if self.detected_marker_id == 1:
                self.task_a_pub.publish(Bool(data=False))
            else:
                self.task_b_pub.publish(Bool(data=False))
            self.reset()

    def reset(self):
        self.state = 'WAITING'
        self.detected_marker_id = None
        self.docking_start_time = None
        self.get_logger().info('Back to WAITING')

    def _docking_watchdog(self):
        if self.state != 'DOCKING' or self.docking_start_time is None:
            return
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self.docking_start_time
        if elapsed > self.docking_timeout:
            self.get_logger().warn(
                f'Docking watchdog tripped after {elapsed:.1f}s — resetting'
            )
            self.dock_pub.publish(Bool(data=False))
            self.reset()


def main():
    rclpy.init()
    node = DockingNoNav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Manual Shutdown')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
