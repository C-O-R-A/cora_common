from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

import os

from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from lifecycle_msgs.msg import Transition
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from sensor_msgs.msg import Image
from cora_vision_msgs.msg import Marker, MarkerArray
from pathlib import Path

from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R

import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy


class VisionNode(LifecycleNode):
    def __init__(self):
        super().__init__("vision_node")

        # Declare parameters
        self.declare_parameter("aruco_dict", "DICT_4X4_50")
        self.declare_parameter("marker_size", 0.05)

        self.img_subscriber = None
        self.aruco_publisher = None
        self.timer = None

        self.bridge = CvBridge()
        self.img = None

    def get_img(self, img: Image):
        try:
            self.img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Image Subscription Error: {e}')

    def aruco_publisher_callback(self):
        if self.img is not None:
            image = self.img
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.aruco_dict)

            if ids is not None:
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    self.marker_size,
                    self.camera_matrix,
                    self.dist_coeffs
                )
            for i, marker_id in enumerate(ids):
                t = TransformStamped()

                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "camera_frame"
                t.child_frame_id = f"marker_{marker_id[0]}"

                # translation
                t.transform.translation.x = float(tvecs[i][0][0])
                t.transform.translation.y = float(tvecs[i][0][1])
                t.transform.translation.z = float(tvecs[i][0][2])

                # rotation (Rodrigues → quaternion)
                rot_mat, _ = cv2.Rodrigues(rvecs[i])
                quat = R.from_matrix(rot_mat).as_quat()

                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.tf_broadcaster.sendTransform(t)
        else:
            self.get_logger().warn('No image received yet')

    def on_configure(self, state: State):
        self.get_logger().info("Vision configuring...")

        # Get parameters
        self.aruco_dict_name = self.get_parameter("aruco_dict").get_parameter_value().string_value
        self.marker_size = self.get_parameter("marker_size").get_parameter_value().double_value

        self.get_logger().info(f"Using dict: {self.aruco_dict_name}")
        self.get_logger().info(f"Marker size: {self.marker_size}")

        # Setup OpenCV ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, self.aruco_dict_name)
        )
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.img_subscriber = self.create_subscription(
            msg_type=Image,  # Replace with actual message type
            topic="/gripper_cam/image_raw",
            callback=self.get_img,
            qos_profile=10,
        )

        self.aruco_publisher = self.create_lifecycle_publisher(
            msg_type=MarkerArray,  # Replace with actual message type
            topic="/gripper_cam/aruco",
            qos_profile=10,
        )

        self.aruco_publisher_timer = self.create_timer(0.02, self.aruco_publisher_callback)
        self.tf_broadcaster = TransformBroadcaster(self)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("Vision activated")
        self.aruco_publisher.activate()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info("Vision deactivated")
        self.aruco_publisher.deactivate()        
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info("Cleaning up Vision Node...")

        self.destroy_subscription(self.img_subscriber)
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.aruco_publisher)

        self.img_subscriber = None
        self.timer = None
        self.aruco_publisher = None

        return TransitionCallbackReturn.SUCCESS
    
    
def main(args=None) -> None:
  rclpy.init(args=args)
  lifecycle_node = VisionNode()
  rclpy.spin(lifecycle_node)
  rclpy.shutdown()

if __name__ == "__main__":
  main()