import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class StaticTransformPublisher(Node):
    def __init__(self):
        super().__init__('static_transform_publisher')

        self.static_broadcaster = StaticTransformBroadcaster(self)

        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'base_link_marker'

        static_transform.transform.translation.x = 0.575
        static_transform.transform.translation.y = -0.34
        static_transform.transform.translation.z = -0.02
        static_transform.transform.rotation.x = 0.707
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.707
        static_transform.transform.rotation.w = 0.0

        self.static_broadcaster.sendTransform(static_transform)

def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
