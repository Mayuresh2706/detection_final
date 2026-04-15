import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Bool, String
from aruco_detection.docking_base import DockingBase


class Task_B_Controller(DockingBase):
    def __init__(self):
        super().__init__('task_b_node', dock_marker_id=2)
        self.create_subscription(Bool, '/task_b_active', self.active_cb, 10)

        # Pendulum tracking state (use distinct names to avoid base-class conflicts)
        self.pendulum_x = None
        self.pendulum_time = None
        self.zero_crossings = []
        self.period_samples = []
        self.period = None
        self.shots_fired = 0

        # Second target_3d subscription — listens for the pendulum marker (ID 3)
        self.create_subscription(PoseStamped, 'target_3d', self._pendulum_callback, 10)

    def on_docked(self):
        self.state = 'tracking'
        self.pendulum_x = None
        self.pendulum_time = None
        self.zero_crossings = []
        self.period_samples = []
        self.period = None
        self.shots_fired = 0
        self.get_logger().info('Docked — waiting for pendulum marker (ID 3)')

    def _pendulum_callback(self, msg):
        marker_id = int(msg.pose.orientation.w)

        if marker_id != 3:
            return
        if self.state != 'tracking':
            return

        marker_x = msg.pose.position.x
        now = self.get_clock().now().nanoseconds / 1e9

        if self.pendulum_x is not None:
            # Detect a zero crossing
            if self.pendulum_x * marker_x < 0:
                if not self.zero_crossings or (now - self.zero_crossings[-1] > 0.1):
                    self.zero_crossings.append(now)

            if len(self.zero_crossings) >= 3:
                current_cycle_period = self.zero_crossings[-1] - self.zero_crossings[-3]
                self.period_samples.append(current_cycle_period)

            if len(self.period_samples) == 3:
                self.period = sum(self.period_samples) / len(self.period_samples)

            if self.pendulum_x * marker_x < 0:
                if self.period is not None and self.period > 0 and self.shots_fired < 3:
                    self.shots_fired += 1
                    self.get_logger().info(f'FIRE {self.shots_fired}/3 at zero crossing')
                    self.fire_pub.publish(Bool(data=True))

                if self.shots_fired == 3:
                    self.get_logger().info('All shots fired. Reporting SUCCESS.')
                    msg_out = String()
                    msg_out.data = 'SUCCESS'
                    self.status_pub.publish(msg_out)
                    self.stop_robot()

        self.pendulum_x = marker_x
        self.pendulum_time = now

    def stop_robot(self):
        super().stop_robot()
        self.shots_fired = 0
        self.zero_crossings = []
        self.period_samples = []
        self.period = None
        self.pendulum_x = None
        self.pendulum_time = None


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
