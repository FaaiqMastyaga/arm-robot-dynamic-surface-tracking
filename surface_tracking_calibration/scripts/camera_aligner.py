#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class CameraAligner(Node):
    def __init__(self):
        super().__init__('camera_aligner')

        # Listen to the Aimooe Tree
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Broadcast  to the Global Tree
        self.tf_broadcaster = TransformBroadcaster(self)

        # Run timer
        self.timer = self.create_timer(0.01, self.align_camera)

    def align_camera(self):
        try:
            # 1. The Magic Trick: Ask TF2 for the INVERSE transform automatically
            # We ask for the path FROM the marker TO the camera
            t = self.tf_buffer.lookup_transform(
                'robot_base',           # Target Frame
                'aimooe_camera_link',   # Source Frame
                rclpy.time.Time()
            )

            # 2. The Transplant: Change the names to bridge the two trees
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'elfin_base_link'       # Anchor it to the Gazebo robot
            t.child_frame_id = 'aimooe_camera_link'     # Move the camera

            # 3. Publish the bridge!
            self.tf_broadcaster.sendTransform(t)

        except (LookupException, ConnectivityException, ExtrapolationException):
            # Ignore errors if the camera drops a frame or hasn't started yet
            pass

def main(args=None):
    rclpy.init(args=args)
    node = CameraAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()