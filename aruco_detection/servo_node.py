import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from aruco_detection.servo import MG996R


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.servo = MG996R(pin=18)
        self.create_subscription(Bool, '/fire', self.fire_cb, 10)
        self.get_logger().info('Servo node ready')

    def fire_cb(self, msg):
        if msg.data:
            self.get_logger().info('Fired n')
            self.servo.fire(count=1)


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down servo node')
    finally:
        node.servo.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
