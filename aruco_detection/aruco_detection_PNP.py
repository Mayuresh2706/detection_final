import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
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
        self.angle_publisher_ = self.create_publisher(Float32, 'marker_normal_angle', 10)

        cal_path = os.path.join(os.path.dirname(__file__), 'camera_calibration.npz')
        data = np.load(cal_path)
        self.mtx = data['mtx']
        self.dist = data['dist']

        # Outer border size: 125mm
        self.marker_size = 0.125

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
            R, _ = cv2.Rodrigues(rvec)
            normal_cam = R @ np.array([0.0, 0.0, 1.0])
            # Remap to robot frame (same mapping as tvec)
            normal_robot_x = normal_cam[2]    # camera Z → robot X (forward)
            normal_robot_y = -normal_cam[0]   # camera X → robot Y (lateral)
            angle_rad = np.arctan2(normal_robot_y, normal_robot_x)
            angle_deg = float(np.degrees(angle_rad))

            self.publish_pose(tvec, rvec, marker_id, angle_deg)
            print(f"Marker id = {marker_id}, normal angle = {angle_deg:.2f} deg")

    def publish_pose(self, tvec, rvec, id, angle_deg):
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

        angle_msg = Float32()
        angle_msg.data = angle_deg
        self.angle_publisher_.publish(angle_msg)


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
