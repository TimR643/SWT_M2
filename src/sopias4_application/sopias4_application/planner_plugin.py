#!/usr/bin/env python3

import heapq
import math
from collections import defaultdict
from typing import List, Optional, Tuple

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from nav2_simple_commander.costmap_2d import PyCostmap2D
from sopias4_framework.nodes.planner_pyplugin import PlannerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools
from sopias4_framework.tools.ros2.costmap_tools import LETHAL_COST
from sopias4_msgs.msg import RobotStates


# Class name must be PlannerPlugin so the GUI can load it.
class PlannerPlugin(PlannerPyPlugin):

    def __init__(self, namespace: str | None = None) -> None:
        if namespace is None:
            super().__init__(node_name="planner_astar", plugin_name="astar")
            self.ns_prefix = ""
        else:
            super().__init__(
                node_name="planner_astar", plugin_name="astar", namespace=namespace
            )
            self.ns_prefix = f"/{namespace.strip('/')}" if namespace else ""

        self.enable_caching()
        self.get_logger().info("Started node - Schnitzeljagd Planner")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        print("PLANNER UP")

        service_name = f"{self.ns_prefix}/local_costmap/clear_entirely_local_costmap"
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, service_name)

        self.schnitzeljagd_mode = True
        self.collision_penalty_seconds = self.declare_parameter(
            "collision_penalty_seconds", 5.0
        ).value
        self.average_speed = self.declare_parameter(
            "schnitzeljagd_average_speed", 0.35
        ).value
        self.replan_overhead_seconds = self.declare_parameter(
            "replan_overhead_seconds", 0.5
        ).value
        self.detour_threshold_factor = 1.1

        self.opponent_robot_namespace = None

    def generate_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        costmap: PyCostmap2D,
        goal_tolerance: float = 0.2,
    ) -> List[Tuple[int, int]]:

        safe_path = self._run_astar(start, goal, costmap, goal_tolerance)

        if not self.schnitzeljagd_mode:
            return safe_path if safe_path else []

        euclidean_dist = (
            costmap_tools.euclidian_distance_map_domain(start, goal, costmap)
            * costmap.getResolution()
        )

        if safe_path:
            safe_dist = self.calculate_path_length(safe_path, costmap)
            if safe_dist <= (euclidean_dist * self.detour_threshold_factor):
                return safe_path

            self.get_logger().info(
                "Detour detected (A*: %.2fm vs straight: %.2fm).",
                safe_dist,
                euclidean_dist,
            )
        else:
            self.get_logger().warn("A* blocked; falling back to safe path.")

        self.get_logger().info("Schnitzeljagd: using safe path only.")
        return safe_path if safe_path else []

    def trigger_costmap_clear(self):
        """Clear local costmap to ignore obstacles (not used in safe mode)."""
        if self.clear_costmap_client.service_is_ready():
            req = ClearEntireCostmap.Request()
            self.clear_costmap_client.call_async(req)

    def is_path_blocked(
        self, path: List[Tuple[int, int]], costmap: PyCostmap2D
    ) -> bool:
        for x, y in path:
            if costmap.getCostXY(x, y) >= LETHAL_COST:
                return True
        return False

    def calculate_path_length(
        self, path: List[Tuple[int, int]], costmap: PyCostmap2D
    ) -> float:
        if not path or len(path) < 2:
            return 0.0
        dist = 0.0
        res = costmap.getResolution()
        for i in range(len(path) - 1):
            dx = path[i + 1][0] - path[i][0]
            dy = path[i + 1][1] - path[i][1]
            dist += math.sqrt(dx * dx + dy * dy) * res
        return dist

    def build_direct_path(self, start, goal):
        path = [start]
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        steps = int(max(abs(dx), abs(dy)))
        if steps == 0:
            return path
        for step in range(1, steps):
            t = step / steps
            path.append((int(round(start[0] + dx * t)), int(round(start[1] + dy * t))))
        path.append(goal)
        return path

    def _run_astar(self, start, goal, costmap, goal_tolerance):
        dist_to_goal = costmap_tools.euclidian_distance_map_domain(start, goal, costmap)
        if dist_to_goal <= goal_tolerance:
            return [start, goal]

        open_set = []
        heapq.heappush(open_set, (0.0, start))
        g_score = defaultdict(lambda: math.inf)
        g_score[start] = 0.0
        f_score = defaultdict(lambda: math.inf)
        f_score[start] = self.heuristic(start, goal, costmap)
        came_from = {}
        neighbors_8 = [
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
            _, current = heapq.heappop(open_set)
            if (
                costmap_tools.euclidian_distance_map_domain(current, goal, costmap)
                <= goal_tolerance
            ):
                return self.reconstruct_path(came_from, current)

            for dx, dy in neighbors_8:
                nx, ny = current[0] + dx, current[1] + dy
                cell_cost = costmap.getCostXY(nx, ny)
                if cell_cost >= LETHAL_COST:
                    continue

                move_cost = 1.0 + (cell_cost / 255.0)
                tentative_g_score = g_score[current] + round(move_cost)

                if tentative_g_score < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g_score
                    f_score[(nx, ny)] = tentative_g_score + self.heuristic(
                        (nx, ny), goal, costmap
                    )
                    heapq.heappush(open_set, (f_score[(nx, ny)], (nx, ny)))
        return None

    def heuristic(self, node, goal, costmap):
        return costmap_tools.euclidian_distance_map_domain(node, goal, costmap)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def robot_states_callback(self, msg: RobotStates) -> None:
        pass


def main(args=None):
    rclpy.init(args=args)
    ros2_node = PlannerPlugin()
    ros2_node.get_logger().info("Planner is up")
    rclpy.spin(ros2_node)
    ros2_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
