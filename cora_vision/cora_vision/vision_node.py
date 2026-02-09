from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn


class VisionNode(LifecycleNode):
    def __init__(self):
        super().__init__("vision_node")
        self.feed_publisher = self.create_lifecycle_publisher(
            msg_type=None,  # Replace with actual message type
            topic="camera_feed",
            qos_profile=10,
        )
        self.aruco_publisher = self.create_lifecycle_publisher(
            msg_type=None,  # Replace with actual message type
            topic="aruco_data",
            qos_profile=10,
        )

    def on_configure(self, state: State):
        self.get_logger().info("Vision configuring...")
        # allocate resources, get camerera up
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State):
        self.get_logger().info("Vision activated")
        # start recording and publishing
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State):
        self.get_logger().info("Vision deactivated")
        # stop work
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State):
        self.get_logger().info("Vision cleaned up")
        return TransitionCallbackReturn.SUCCESS

