import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState

class TurtleBotListener(Node):
    def __init__(self):
        super().__init__("turtlebot_listener")

        # Namespace z. B. "/tb4_01"
        namespace = "/tb4_01"

        topic = f"{namespace}/battery_state"
        self.subscription = self.create_subscription(
            BatteryState,
            topic,
            self.battery_callback,
            10
        )

        self.get_logger().info(f"Listening on {topic}")

    def battery_callback(self, msg):
        self.get_logger().info(f"Batterie: {msg.percentage * 100:.1f}%")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
