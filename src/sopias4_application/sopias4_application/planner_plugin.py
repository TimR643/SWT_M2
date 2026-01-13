#!/usr/bin/env python3

import heapq
import math
from collections import defaultdict
from typing import List, Tuple, Optional

import rclpy
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_msgs.srv import ClearEntireCostmap
from sopias4_framework.nodes.planner_pyplugin import PlannerPyPlugin
from sopias4_framework.tools.ros2 import costmap_tools
from sopias4_framework.tools.ros2.costmap_tools import LETHAL_COST
from sopias4_msgs.msg import RobotStates

# WICHTIG: Die Klasse muss PlannerPlugin heißen, damit die GUI sie findet!
class PlannerPlugin(PlannerPyPlugin):

    def __init__(self, namespace: str | None = None) -> None:
        # Konstruktor Aufruf korrigiert und vereinfacht
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

        # --- Service Client zum Löschen der Lokalen Costmap ---
        # Dies ist notwendig, damit der Controller durch dynamische Hindernisse fährt
        service_name = f"{self.ns_prefix}/local_costmap/clear_entirely_local_costmap"
        self.clear_costmap_client = self.create_client(ClearEntireCostmap, service_name)

        # --- Parameter Initialisierung ---
        # Wir definieren die Attribute direkt hier, keine Klassen-Annotationen (vermeidet AttributeErrors)
        self.schnitzeljagd_mode = self.declare_parameter("schnitzeljagd_mode", False).value
        self.collision_penalty_seconds = self.declare_parameter("collision_penalty_seconds", 5.0).value
        self.average_speed = self.declare_parameter("schnitzeljagd_average_speed", 0.35).value
        self.replan_overhead_seconds = self.declare_parameter("replan_overhead_seconds", 0.5).value
        self.detour_threshold_factor = 1.1 # Ab wann gilt ein Weg als Umweg? (1.1 = 10% länger)

        self.opponent_robot_namespace = None
        
        # Subscriber für Gegner-Erkennung
        self.create_subscription(
            RobotStates, "/robot_states", self.robot_states_callback, 10
        )

    def generate_path(
        self,
        start: Tuple[int, int],
        goal: Tuple[int, int],
        costmap: PyCostmap2D,
        goal_tolerance: float = 0.2,
    ) -> List[Tuple[int, int]]:
        
        # 1. Berechne Standard A* (Sicherer Weg)
        safe_path = self._run_astar(start, goal, costmap, goal_tolerance)
        
        # Wenn Schnitzeljagd aus ist, einfach den sicheren Weg nehmen
        if not self.schnitzeljagd_mode:
            return safe_path if safe_path else []

        # -------------------------------------------------------------
        # 2. Lazy Evaluation: Müssen wir überhaupt "durchbrechen"?
        # -------------------------------------------------------------
        
        # Luftlinien-Distanz (Meter)
        euclidean_dist = costmap_tools.euclidian_distance_map_domain(start, goal, costmap) * costmap.getResolution()
        
        # Wenn wir einen Pfad haben, prüfen wir, ob es ein großer Umweg ist
        if safe_path:
            safe_dist = self.calculate_path_length(safe_path, costmap)
            # Wenn der Weg nur < 10% länger als Luftlinie ist -> Fahr ihn, kein Hindernis.
            if safe_dist <= (euclidean_dist * self.detour_threshold_factor):
                return safe_path

            self.get_logger().info(f"Umweg erkannt! (A*: {safe_dist:.2f}m vs Luftlinie: {euclidean_dist:.2f}m)")
        else:
             self.get_logger().warn("A* blockiert! Prüfe Durchbruch...")

        # -------------------------------------------------------------
        # 3. Entscheidung: Durchfahren oder Umfahren?
        # -------------------------------------------------------------

        # Direkten Weg simulieren
        direct_path = self.build_direct_path(start, goal)
        
        # Wenn direkter Weg frei ist (z.B. A* Bug), nimm ihn einfach
        if not self.is_path_blocked(direct_path, costmap):
            return direct_path

        # Zeitvergleich
        time_safe = 2#(self.calculate_path_length(safe_path, costmap) / self.average_speed) if safe_path else float('inf')
        time_direct_penalty = 1#(self.calculate_path_length(direct_path, costmap) / self.average_speed) + self.collision_penalty_seconds

        self.get_logger().info(f"Vergleich: Umweg {time_safe:.1f}s vs. Durchbruch {time_direct_penalty:.1f}s")

        if time_direct_penalty < time_safe:
            self.get_logger().warn(">> ENTSCHEIDUNG: Durchfahren ist schneller! Lösche Costmap.")
            # Controller blind machen für Hindernis
            self.trigger_costmap_clear()
            return direct_path
        else:
            self.get_logger().info(">> ENTSCHEIDUNG: Umweg ist besser.")
            return safe_path if safe_path else []

    # --- Hilfsfunktionen ---

    def trigger_costmap_clear(self):
        """Löscht die lokale Costmap, damit der Controller durch das Hindernis fährt."""
        if self.clear_costmap_client.service_is_ready():
            req = ClearEntireCostmap.Request()
            self.clear_costmap_client.call_async(req)

    def is_path_blocked(self, path: List[Tuple[int, int]], costmap: PyCostmap2D) -> bool:
        for x, y in path:
            if costmap.getCostXY(x, y) >= LETHAL_COST:
                return True
        return False

    def calculate_path_length(self, path: List[Tuple[int, int]], costmap: PyCostmap2D) -> float:
        if not path or len(path) < 2: return 0.0
        dist = 0.0
        res = costmap.getResolution()
        for i in range(len(path) - 1):
            dx = path[i+1][0] - path[i][0]
            dy = path[i+1][1] - path[i][1]
            dist += math.sqrt(dx*dx + dy*dy) * res
        return dist

    def build_direct_path(self, start, goal):
        path = [start]
        dx = goal[0] - start[0]
        dy = goal[1] - start[1]
        steps = int(max(abs(dx), abs(dy)))
        if steps == 0: return path
        for step in range(1, steps):
            t = step / steps
            path.append((int(round(start[0] + dx*t)), int(round(start[1] + dy*t))))
        path.append(goal)
        return path

    def _run_astar(self, start, goal, costmap, goal_tolerance):
        """Standard A* Implementierung"""
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
        neighbors_8 = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, current = heapq.heappop(open_set)
            if costmap_tools.euclidian_distance_map_domain(current, goal, costmap) <= goal_tolerance:
                return self.reconstruct_path(came_from, current)

            for dx, dy in neighbors_8:
                nx, ny = current[0] + dx, current[1] + dy
                cell_cost = costmap.getCostXY(nx, ny)
                if cell_cost >= LETHAL_COST: continue

                move_cost = 1.0 + (cell_cost / 255.0)
                tentative_g_score = g_score[current] + round(move_cost)

                if tentative_g_score < g_score[(nx, ny)]:
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)] = tentative_g_score
                    f_score[(nx, ny)] = tentative_g_score + self.heuristic((nx, ny), goal, costmap)
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
        """Aktiviert Schnitzeljagd-Modus, wenn Turtlebot 3 der Gegner ist."""
        own_namespace = self.get_namespace()
        for robot in msg.robot_states:
            if robot.name_space == own_namespace:
                continue

            self.opponent_robot_namespace = robot.name_space
            opponent_suffix = robot.name_space.rstrip("/").split("/")[-1]

            if opponent_suffix in ("3", "robot3"):
                if not self.schnitzeljagd_mode:
                    self.get_logger().info("Gegner Robot 3 erkannt -> AKTIVIERE Schnitzeljagd Modus!")
                self.schnitzeljagd_mode = True
            return

def main(args=None):
    rclpy.init(args=args)
    ros2_node = PlannerPlugin()
    ros2_node.get_logger().info("Planner is up")
    rclpy.spin(ros2_node)
    ros2_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()