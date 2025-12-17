"""This file was mainly edited by Nils Gerth, Patrick Henle"""

import rclpy
from sopias4_framework.nodes.layer_pyplugin import *
from sopias4_framework.nodes.layer_pyplugin import LayerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools as CT

from sopias4_msgs.msg import Robot, RobotStates


class RobotLayer(LayerPyPlugin):
    opponent_robot: Robot

    def __init__(self, namespace: str | None = None) -> None:

        super().__init__(node_name="robot_layer_node", plugin_name="robot_layer")
        if namespace is None:
            pass
        else:
            super().__init__(
                node_name="robot_layer_node",
                plugin_name="robot_layer",
                namespace=namespace,
            )
        self.own_namespace = namespace
        self.opponent_robot = Robot()
        print("LAYER ")
        self.create_subscription(
            RobotStates, "/robot_states", self.robot_states_callback, 10
        )

    def update_costs(
        self, min_i: int, min_j: int, max_i: int, max_j: int, costmap: PyCostmap2D
    ) -> PyCostmap2D:
        """Gets the opponends pose and transforms it into the correct frame"""
        x, y = CT.pose_2_costmap(self.opponent_robot.pose.pose.pose, costmap)
        # print("XXXXXX: ", x, "YYYYYY: ", y)
        """Sets Cost for the Opp. Robot into the Costmap as a Circle"""
        self.set_circle_cost(
            costmap, x, y, 5, self.COST_LETHAL_OBSTACLE
        )  # Beispielwerte

        """Würde den Pfad des Gegners mit in die Costmap übernehmen, taktisch ehr nicht sinnvoll"""
        # for pose in self.opponent_robot.nav_path.poses:
        #     x, y = CT.pose_2_costmap(pose, costmap)
        #     self.set_circle_cost(costmap, x, y, 1, self.COST_LETHAL_OBSTACLE)
        """END of Comment"""

        return costmap

    def print_costs(self, costmap: PyCostmap2D, width: int, height: int) -> None:
        for y in range(height):
            row = []
            for x in range(width):
                cost = costmap.getCostXY(x, y)
                row.append(cost)
            print(" ".join(map(str, row)))

    def robot_states_callback(self, msg: RobotStates):
        # Überprüfen, ob es Roboter in der Liste gibt
        for i, robot in enumerate(msg.robot_states):
            if (
                robot.name_space != self.own_namespace
            ):  # HIER MUSS UNGLEICH HINGEMACHT WERDEN !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                self.opponent_robot = robot

    def set_circle_cost(
        self,
        costmap: PyCostmap2D,
        center_x: int,
        center_y: int,
        radius: int,
        cost: np.uint8,
    ) -> None:
        width = costmap.getSizeInCellsX()
        height = costmap.getSizeInCellsY()

        for y in range(center_y - radius, center_y + radius + 1):
            for x in range(center_x - radius, center_x + radius + 1):
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius**2:
                    if 0 <= x < width and 0 <= y < height:  # Check if within bounds
                        costmap.setCost(x, y, cost)


def main(args=None):
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = RobotLayer()
    ros2_node.get_logger().info("Layer is up")
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
