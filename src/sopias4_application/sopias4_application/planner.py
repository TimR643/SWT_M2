

import heapq
import math
from collections import defaultdict
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.service import Service
from sopias4_framework.nodes.planner_pyplugin import *
from sopias4_framework.nodes.planner_pyplugin import PlannerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools
from sopias4_framework.tools.ros2.costmap_tools import LETHAL_COST

from sopias4_msgs.srv import CreatePlan


class Astar(PlannerPyPlugin):
    goal_tolerance: float = 0.2
    caching_tolerance: float = 0.4
    cached_path: list[Tuple[int, int]]

    def __init__(self, namespace: str | None = None) -> None:
        (
            super().__init__(node_name="planner_astar", plugin_name="astar")
            if namespace is None
            else super().__init__(
                node_name="planner_astar", plugin_name="astar", namespace=namespace
            )
        )
        self.enable_caching()
        self.get_logger().info("Started node")
        self.get_logger().set_level(30)

        self.enable_caching()
        self.get_logger().info("Started node")
        self.get_logger().set_level(30)
        print("PLANNER UP")

    def generate_path(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        costmap: PyCostmap2D,
        goal_tolerance: float = 0.2,
    ) -> list[tuple[int, int]]:
        """
        Generates a path from start to goal coordinates based on A*.
        """
        dist_to_goal = costmap_tools.euclidian_distance_map_domain(start, goal, costmap)
        if dist_to_goal <= goal_tolerance:
            return [
                start,
                goal,
            ]  # If the goal is already close enough, return a direct path

        open_set = []  # Priority queue for A*
        heapq.heappush(open_set, (0.0, start))  # Add start node with priority 0

        g_score = defaultdict(lambda: math.inf)  # Cost from start to a node
        g_score[start] = 0.0

        f_score = defaultdict(lambda: math.inf)  # Estimated total cost to goal
        f_score[start] = self.heuristic(start, goal, costmap)

        came_from = {}  # Stores the predecessor of a node

        neighbors_8 = [  # 8 possible movements (including diagonals)
            (1, 0),
            (-1, 0),
            (0, 1),
            (0, -1),
            (1, 1),
            (1, -1),
            (-1, 1),
            (-1, -1),
        ]

        while open_set:
            _, current = heapq.heappop(open_set)  # Next node with the lowest priority
            if (
                costmap_tools.euclidian_distance_map_domain(current, goal, costmap)
                <= goal_tolerance
            ):
                return self.reconstruct_path(came_from, current)  # Trace back the path

            for dx, dy in neighbors_8:
                nx, ny = current[0] + dx, current[1] + dy  # Neighbor coordinates

                cell_cost = costmap.getCostXY(
                    nx, ny
                )  # Retrieve the cost of the cell from the costmap
                if cell_cost >= LETHAL_COST:
                    continue  # Skip blocked or dangerous cells

                move_cost = 1.0 + (
                    cell_cost / 255.0
                )  # Calculate movement cost based on cell cost
                tentative_g_score = g_score[current] + round(move_cost)

                if tentative_g_score < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = current  # Store the best predecessor
                    g_score[(nx, ny)] = tentative_g_score  # Update cost to neighbor
                    f_score[(nx, ny)] = tentative_g_score + self.heuristic(
                        (nx, ny), goal, costmap
                    )
                    heapq.heappush(
                        open_set, (f_score[(nx, ny)], (nx, ny))
                    )  # Add neighbor to queue

        self.get_logger().warn("No valid path found by A*.")
        return []  # No path found

    """Only for test purposes"""
    # def print_costs(self, costmap: PyCostmap2D, width: int, height: int) -> None:
    #     for y in range(height):
    #         row = []
    #         for x in range(width):
    #             cost = costmap.getCostXY(x, y)
    #             row.append(cost)
    #         print(" ".join(map(str, row)))

    def heuristic(
        self, node: tuple[int, int], goal: tuple[int, int], costmap: PyCostmap2D
    ) -> float:
        """
        Computes the heuristic for the A* algorithm (Euclidean distance to goal).
        """
        return costmap_tools.euclidian_distance_map_domain(node, goal, costmap)

    def reconstruct_path(
        self,
        came_from: dict[tuple[int, int], tuple[int, int]],
        current: tuple[int, int],
    ) -> list[tuple[int, int]]:
        """
        Reconstructs the path from the goal position back to the start.
        """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()  # Reverse the path to go from start to goal
        return path


def main(args=None):
    # Initialize node context
    rclpy.init(args=args)
    # Create ROS2 Node
    ros2_node = Astar()
    ros2_node.get_logger().info("Planner is up")

    # Run node
    rclpy.spin(ros2_node)
    # Cleanup
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
