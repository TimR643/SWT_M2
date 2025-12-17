"""This file was mainly edited by Nils Gerth, Torben Müller"""

import heapq
import math
from collections import defaultdict
from typing import Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav_msgs.msg import Path
from sopias4_framework.nodes.planner_pyplugin import *
from sopias4_framework.nodes.planner_pyplugin import PlannerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools
from sopias4_framework.tools.ros2.costmap_tools import LETHAL_COST

from sopias4_msgs.srv import CreatePlan


class Astar(PlannerPyPlugin):
    goal_tolerance: float = 0.2
    caching_tolerance: float = 0.4
    cached_path: list[Tuple[int, int]]
    schnitzeljagd_mode: bool
    collision_penalty_seconds: float
    schnitzeljagd_average_speed: float
    replan_overhead_seconds: float

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

        self.schnitzeljagd_mode = self.declare_parameter(
            "schnitzeljagd_mode", False
        ).value
        self.collision_penalty_seconds = self.declare_parameter(
            "collision_penalty_seconds", 5.0
        ).value
        self.schnitzeljagd_average_speed = self.declare_parameter(
            "schnitzeljagd_average_speed", 0.35
        ).value
        self.replan_overhead_seconds = self.declare_parameter(
            "replan_overhead_seconds", 0.5
        ).value

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
                path = self.reconstruct_path(came_from, current)
                if self.schnitzeljagd_mode:
                    schnitzeljagd_path = self.evaluate_schnitzeljagd_strategy(
                        start=start, goal=goal, safe_path=path, costmap=costmap
                    )
                    if schnitzeljagd_path is not None:
                        return schnitzeljagd_path
                return path  # Trace back the path

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
        if self.schnitzeljagd_mode:
            self.get_logger().warn(
                "Falling back to direct penalty path because no valid path was found"
            )
            return self.build_direct_path(start, goal)
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

    def evaluate_schnitzeljagd_strategy(
        self,
        start: tuple[int, int],
        goal: tuple[int, int],
        safe_path: list[tuple[int, int]],
        costmap: PyCostmap2D,
    ) -> list[tuple[int, int]] | None:
        """
        Decide whether to follow the safe path or take a direct route and accept
        a collision penalty.

        Args:
            start: Start coordinate in costmap domain.
            goal: Goal coordinate in costmap domain.
            safe_path: Path that avoids lethal obstacles.
            costmap: Current costmap used for estimations.

        Returns:
            The chosen path if the penalty option is faster, otherwise ``None`` to
            indicate that the safe path should be used.
        """

        direct_path = self.build_direct_path(start, goal)

        safe_time = self.estimate_travel_time_seconds(safe_path, costmap)
        direct_time = self.estimate_travel_time_seconds(direct_path, costmap)

        adjusted_safe_time = safe_time + self.replan_overhead_seconds
        penalty_time = direct_time + self.collision_penalty_seconds

        if penalty_time < adjusted_safe_time:
            self.get_logger().info(
                "Schnitzeljagd mode: Choosing direct path with penalty because it is faster"
            )
            return direct_path

        self.get_logger().debug(
            "Schnitzeljagd mode: Using safe path because penalty shortcut is slower"
        )
        return None

    def estimate_travel_time_seconds(
        self, path: list[tuple[int, int]], costmap: PyCostmap2D
    ) -> float:
        """
        Estimate travel time for a path based on average speed and map resolution.
        """
        if len(path) < 2 or self.schnitzeljagd_average_speed <= 0:
            return 0.0

        distance_m = 0.0
        for previous, current in zip(path[:-1], path[1:]):
            distance_m += costmap_tools.euclidian_distance_pixel_domain(
                previous, current, costmap
            ) * costmap.getResolution()

        return distance_m / self.schnitzeljagd_average_speed

    def build_direct_path(
        self, start: tuple[int, int], goal: tuple[int, int]
    ) -> list[tuple[int, int]]:
        """Generate a straight-line path between two points in costmap space."""
        path: list[tuple[int, int]] = [start]
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        steps = max(abs(dx), abs(dy))

        if steps == 0:
            return path

        for step in range(1, steps):
            x = round(start[0] + (dx * step) / steps)
            y = round(start[1] + (dy * step) / steps)
            if (x, y) != path[-1]:
                path.append((x, y))

        path.append(goal)
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
