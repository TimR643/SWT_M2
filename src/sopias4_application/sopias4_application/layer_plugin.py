#!/usr/bin/env python3

import rclpy
import numpy as np
from nav2_simple_commander.costmap_2d import PyCostmap2D
from sopias4_framework.nodes.layer_pyplugin import LayerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools

from sopias4_msgs.msg import Robot, RobotStates


class LayerPlugin(LayerPyPlugin):
    """
    A plugin template to integrate additional layers. Remember that each new plugin needs to be added to the
    configuration in order to be integrated properly. Check the documentation of the base class for further details!
    """
    opponent_robot: Robot

    def __init__(self, namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(node_name="layer_plugin_node", plugin_name="layer_plugin")
        else:
            super().__init__(
                node_name="layer_plugin_node",
                plugin_name="layer_plugin",
                namespace=namespace,
            )

        self.own_namespace = namespace
        self.opponent_robot = Robot()
        print("LAYER ")
        self.create_subscription(
            RobotStates, "/robot_states", self.robot_states_callback, 10
        )

        self.get_logger().info("Started node")

    def update_costs(
        self, min_i: int, min_j: int, max_i: int, max_j: int, costmap: PyCostmap2D
    ) -> PyCostmap2D:
        """
        Here the costmap should be updated. Only update the region inside the window, specified by the min_* and max_* \
        arguments, to save computational time.

        Args:
            min_i (int): The minimum x-index of the update window
            min_j (int): The minimum y-index of the update window
            max_i (int): The maximum x-index of the update window
            max_j (int): The maximum y-index of the update window
            costmap(nav2_simplecommander.costmap_2d.PyCostmap2D): The costmap that should be updated

        Returns:
            nav2_simplecommander.costmap_2d.PyCostmap2D: The updated costmap 
        """

        # Set cost of all pixels in costmap to zero, because only the costs calculated by this layer should be included in the map.
        # In the plugin bridge, all layers get combined so the cleared data doesn't get lost if it is up to date
        costmap.costmap.fill(self.COST_FREE_SPACE)

        x, y = costmap_tools.pose_2_costmap(self.opponent_robot.pose.pose.pose, costmap)

        self.set_circle_cost(
            costmap, x, y, 5, self.COST_LETHAL_OBSTACLE
        )

        return costmap


    def print_costs(self, costmap: PyCostmap2D, width: int, height: int) -> None:
        for y in range(height):
            row = []
            for x in range(width):
                cost = costmap.getCostXY(x, y)
                row.append(cost)
            print(" ".join(map(str, row)))

    def robot_states_callback(self, msg: RobotStates):
        # check if robot is available in list
        for i, robot in enumerate(msg.robot_states):
            if (
                robot.name_space != self.own_namespace
            ):
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
    """
    :meta private:
    """
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = LayerPlugin()
    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
