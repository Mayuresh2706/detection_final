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
        self.marker_size = 0.050

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

            # Use -1.0 to get the vector pointing FROM the marker TO the camera
            normal_cam = R @ np.array([0.0, 0.0, -1.0])

            normal_robot_x = normal_cam[2]
            normal_robot_y = -normal_cam[0]

            # 0 = marker faces away from robot, ±pi = marker faces the robot
            angle_rad = np.arctan2(normal_robot_y, normal_robot_x)
            angle_deg = float(np.degrees(angle_rad))

            self.publish_pose(tvec, marker_id, angle_deg)

    def publish_pose(self, tvec, marker_id, angle_deg):
        pose_msg = PoseStamped()
        # Encode marker id in the frame_id so consumers can associate id with
        # pose atomically (no cross-topic ordering race). Consumers split on ':'.
        pose_msg.header.frame_id = f"camera_link:{marker_id}"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(tvec[2][0])
        pose_msg.pose.position.y = -float(tvec[0][0])
        pose_msg.pose.position.z = -float(tvec[1][0])
        pose_msg.pose.orientation.w = 1.0
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
