import rclpy

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class RGBNode(Node):
    def __init__(self):
        super().__init__("guidedog_rgb")

        self.frames = 0

        self.subscription = self.create_subscription(
            Image,
            "/rgb_image",
            self.on_image,
            qos_profile_sensor_data,
        )

        self.get_logger().info("GuideDog RGB subscriber started")

    def on_image(self, msg):
        self.frames += 1

        if self.frames % 5 == 0:
            self.get_logger().info(
                f"Frame #{self.frames} | "
                f"{msg.width}x{msg.height} | "
                f"encoding={msg.encoding} | "
                f"frame={msg.header.frame_id}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = RGBNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
