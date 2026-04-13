import threading
import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from aruco_detection.docking_base import DockingBase


class Task_A_Controller(DockingBase):
    def __init__(self):
        super().__init__('task_a_node', dock_marker_id=1)
        self.create_subscription(Bool, '/task_a_active', self.active_cb, 10)

        # Shot sequencing
        self.shots_fired = 0
        self.total_shots = 3
        self.shot_interval = 5.0  # seconds between shots
        self.firing = False

    def on_docked(self):
        self.fire_sequence()

    def fire_sequence(self):
        if self.firing:
            return
        self.firing = True
        self.shots_fired = 0
        threading.Thread(target=self._fire_thread, daemon=True).start()

    def _fire_thread(self):
        import time
        while self.shots_fired < self.total_shots:
            self.shots_fired += 1
            self.get_logger().info(f'FIRE {self.shots_fired}/{self.total_shots}')
            self.fire_pub.publish(Bool(data=True))
            if self.shots_fired < self.total_shots:
                time.sleep(self.shot_interval)

        self.get_logger().info('All shots fired. Reporting SUCCESS.')
        self.firing = False
        msg = String()
        msg.data = 'SUCCESS'
        self.status_pub.publish(msg)
        self.stop_robot()

    def stop_robot(self):
        super().stop_robot()
        self.shots_fired = 0
        self.firing = False


def main(args=None):
    rclpy.init(args=args)
    node = Task_A_Controller()
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
