import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
import os


class ArucoSub_Pub(Node):
    def __init__(self):
        super().__init__('ArucoSub_Pub')
        self.subscription = self.create_subscription(
            Float32MultiArray, 'target_pixels', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(PoseStamped, 'target_3d', 10)

        cal_path = os.path.join(os.path.dirname(__file__), 'camera_calibration.npz')
        data = np.load(cal_path)
        self.mtx = data['mtx']
        self.dist = data['dist']

        # Outer border size: 93mm
        self.marker_size = 0.093

        self.obj_points = np.array([
        [-self.marker_size/2,  self.marker_size/2, 0],
        [ self.marker_size/2,  self.marker_size/2, 0],
        [ self.marker_size/2, -self.marker_size/2, 0],
        [-self.marker_size/2, -self.marker_size/2, 0] ], dtype=np.float32)

    def listener_callback(self, msg):
        data = msg.data
        if len(data) < 9:
            return

        corners = np.array(data[:8]).reshape((4, 1, 2)).astype(np.float32)
        marker_id = int(data[8])


        success, rvec, tvec = cv2.solvePnP(
            self.obj_points, corners, self.mtx, self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)

        if success:
            self.publish_pose(tvec, rvec,marker_id)
            print(f"Marker id = {marker_id}")

    def publish_pose(self, tvec, rvec,id):
        pose_msg = PoseStamped()

        pose_msg.header.frame_id = "camera_link"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(tvec[2][0])
        pose_msg.pose.position.y = -float(tvec[0][0])
        pose_msg.pose.position.z = -float(tvec[1][0])


        pose_msg.pose.orientation.x = float(rvec[0][0])
        pose_msg.pose.orientation.y = float(rvec[1][0])
        pose_msg.pose.orientation.z = float(rvec[2][0])
        pose_msg.pose.orientation.w = float(id)

        self.publisher_.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoSub_Pub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
