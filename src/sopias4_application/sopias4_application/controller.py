"""THIS FILE IS NOT USED"""

# adeed
import math

# import tf_transformations
from sopias4_framework.nodes.controller_pyplugin import *
from sopias4_framework.nodes.controller_pyplugin import ControllerPyPlugin


class RotationStraightController(ControllerPyPlugin):

    def __init__(self, namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(
                node_name="rotation_straight_controller_node",
                plugin_name="rotation_straight_controller",
            )
        else:
            super().__init__(
                node_name="rotation_straight_controller_node",
                plugin_name="rotation_straight_controller",
                namespace=namespace,
            )
        print("INIT CONTROLLER")
        self.get_logger().debug("INIT KONTROLLER")

    def get_node(self):
        return self.get_node()

    def compute_velocity_command(
        self,
        goal_pose: Pose,
        current_pos: PoseStamped,
        current_vel: Twist,
        costmap: PyCostmap2D,
    ) -> TwistStamped:
        """
        Here the velocity command should be computed

        Args:
            goal_pose (Pose): The position to which the robot should drive next
            current_pos (PoseStamped): Current position of the robot in the map domain
            current_vel (Twist): The current velocity of the robot

        Returns:
            TwistStamped: The computed velocity command which the robot should execute next
        """

        # Constants

        # Proportional Gain for angular speed
        proportional_gain_angular_speed = 0.5

        # Proportional Gain for linear speed
        proportional_gain_linear_speed = 0.5

        # Algorithm here
        print(
            "COMPUTE VELOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO"
        )
        #  goal_pose
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y
        goal_z = goal_pose.position.z
        goal_orientation = goal_pose.orientation

        # current_pos
        current_x = current_pos.pose.position.x
        current_y = current_pos.pose.position.y
        current_z = current_pos.pose.position.z
        current_orientation = current_pos.pose.orientation
        current_timestamp = current_pos.header.stamp
        current_frame_id = current_pos.header.frame_id

        # current_vel
        current_linear_x = current_vel.linear.x
        current_linear_y = current_vel.linear.y
        current_linear_z = current_vel.linear.z
        current_angular_x = current_vel.angular.x
        current_angular_y = current_vel.angular.y
        current_angular_z = current_vel.angular.z

        # costmap
        # costmap_width = costmap.get_size_x()
        # costmap_height = costmap.get_size_y()

        """
        # Beispielhafte Berechnung eines TwistStamped-Befehls
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = 1.0  # Beispielwert für lineare Geschwindigkeit
        twist_stamped.twist.angular.z = 0.0  # Beispielwert für Winkelgeschwindigkeit
        """

        # Calculate the difference between the goal and current position
        dx = goal_x - current_x
        dy = goal_y - current_y

        # Calculate the distance to the goal
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate the angle to the goal
        angle_to_goal = math.atan2(dy, dx)

        # Convert the current orientation to Euler angles
        _, _, current_yaw = self.quaternion_to_euler(
            current_orientation
        )  # TODO:  control if this is correct

        # Calculate linear and angular speed
        linear_speed = proportional_gain_linear_speed * distance
        angular_speed = proportional_gain_angular_speed * (angle_to_goal - current_yaw)

        # Create the TwistStamped command
        twist_stamped = TwistStamped()
        twist_stamped.header.stamp = self.get_clock().now().to_msg()
        twist_stamped.header.frame_id = "base_link"
        twist_stamped.twist.linear.x = linear_speed
        twist_stamped.twist.angular.z = angular_speed

        return twist_stamped

    def quaternion_to_euler(self, quaternion):
        # euler = tf_transformations.euler_from_quaternion(
        #     [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        # )
        # return euler
        #  # (roll, pitch, yaw)
        pass


def main(args=None):
    print("MAIIIIIIIIINNNNNNNNNNN")
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = RotationStraightController()
    ros2_node.get_logger().info(
        "\n\nrtcheuxgheuht gxuepgxahe ctpxhsexhc xguController is up\n\n\n\n\n\n"
    )
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
