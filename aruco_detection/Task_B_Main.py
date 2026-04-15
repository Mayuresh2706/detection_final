import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from aruco_detection.docking_base import DockingBase


class Task_B_Controller(DockingBase):
    def __init__(self):
        super().__init__('task_b_node', dock_marker_id=2)
        self.create_subscription(Bool, '/task_b_active', self.active_cb, 10)

        self.shots_fired = 0

        self.create_subscription(PoseStamped, 'target_3d', self._pendulum_callback, 10)

    def on_docked(self):
        self.state = 'tracking'
        self.shots_fired = 0
        self.get_logger().info('Docked — waiting for pendulum marker (ID 3)')

    def _pendulum_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)

        if marker_id != 3:
            return
        if self.state != 'tracking':
            return
        if self.shots_fired >= 3:
            return

        self.shots_fired += 1
        self.get_logger().info(f'FIRE {self.shots_fired}/3 — marker 3 seen')
        self.fire_pub.publish(Bool(data=True))

        if self.shots_fired == 3:
            self.get_logger().info('All shots fired. Reporting SUCCESS.')
            msg_out = String()
            msg_out.data = 'SUCCESS'
            self.status_pub.publish(msg_out)
            self.stop_robot()

    def stop_robot(self):
        super().stop_robot()
        self.shots_fired = 0


def main(args=None):
    rclpy.init(args=args)
    node = Task_B_Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Manual Shutdown')
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
