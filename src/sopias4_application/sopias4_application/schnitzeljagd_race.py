

import itertools
import math
import threading
import time
from typing import List, Optional

import rclpy
import tf2_geometry_msgs
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import ComputePathThroughPoses, NavigateThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class SchnitzeljagdRace(Node):
    def __init__(self) -> None:
        super().__init__("schnitzeljagd_race")

        # Parameter laden
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value
        self.waypoint_topic = self.declare_parameter(
            "waypoint_topic", "schnitzeljagd/waypoint_pose"
        ).value
        self.waypoint_count = int(self.declare_parameter("waypoint_count", 3).value)
        self.planner_id = self.declare_parameter("planner_id", "").value

        # TF Buffer setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Nav2 Action Clients
        self.compute_path_client = ActionClient(
            self, ComputePathThroughPoses, "compute_path_through_poses"
        )
        self.navigate_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )

        # Publisher für direkte Motorsteuerung (für die Pirouette)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Interne Variablen
        self.waypoints: List[PoseStamped] = []
        self._lock = threading.Lock()
        self._processing = False

        # Subscriber für Waypoints (z.B. 2D Nav Goals aus Rviz)
        self.create_subscription(PoseStamped, self.waypoint_topic, self._waypoint_cb, 10)
        
        # Timer um zu prüfen, ob genug Punkte da sind
        self.create_timer(0.2, self._maybe_process)

        self.get_logger().info(
            f"Warte auf {self.waypoint_count} Wegpunkte auf Topic '{self.waypoint_topic}'..."
        )

    def _waypoint_cb(self, msg: PoseStamped) -> None:
        """Sammelt Wegpunkte und transformiert sie in den Map-Frame."""
        pose = self._to_map_frame(msg)
        if pose is None:
            self.get_logger().warn("Konnte Wegpunkt nicht in Map-Frame transformieren.")
            return

        with self._lock:
            if len(self.waypoints) >= self.waypoint_count:
                return
            self.waypoints.append(pose)
            count = len(self.waypoints)

        self.get_logger().info(f"Wegpunkt {count}/{self.waypoint_count} empfangen.")

    def _maybe_process(self) -> None:
        """Startet den Prozess, sobald genug Punkte gesammelt wurden."""
        with self._lock:
            if self._processing or len(self.waypoints) < self.waypoint_count:
                return
            waypoints = list(self.waypoints[: self.waypoint_count])
            self.waypoints = []
            self._processing = True

        # Verarbeitung in separatem Thread starten, um Callbacks nicht zu blockieren
        thread = threading.Thread(
            target=self._process_waypoints, args=(waypoints,), daemon=True
        )
        thread.start()

    def _process_waypoints(self, waypoints: List[PoseStamped]) -> None:
        """Berechnet die beste Route (TSP) und startet die Sequenz."""
        try:
            start_pose = self._get_robot_pose()
            if start_pose is None:
                self.get_logger().warn("Roboter-Position unbekannt. Breche ab.")
                return

            best_order = None
            best_length = math.inf

            # Brute-Force TSP (probiert alle Reihenfolgen durch)
            for order in itertools.permutations(waypoints):
                path = self._compute_path(start_pose, list(order))
                length = self._path_length(path) if path else math.inf
                if length < best_length:
                    best_length = length
                    best_order = list(order)

            if not best_order:
                self.get_logger().error("Kein gültiger Pfad gefunden.")
                return

            self.get_logger().info(
                f"Beste Route gefunden (Länge: {best_length:.2f} m). Starte Rennen..."
            )
            self._send_navigation_sequence(best_order)
        finally:
            with self._lock:
                self._processing = False

    def _perform_pirouette(self) -> None:
        """
        Führt eine Drehung am Stand aus (sehr platzsparend).
        1. 360 Grad nach links.
        2. 360 Grad nach rechts (zurück zur Orientierung).
        """
        self.get_logger().info("--- Starte Kunststück: Pirouette ---")
        
        angular_speed = 1.5
        target_angle = 2 * math.pi
        duration = target_angle / angular_speed
        
        rate_hz = 10
        steps = int(duration * rate_hz)
        sleep_time = 1.0 / rate_hz

        self.get_logger().info("Drehe links...")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        
        for _ in range(steps):
            if not rclpy.ok(): break
            self.cmd_vel_pub.publish(twist)
            time.sleep(sleep_time)

        self.cmd_vel_pub.publish(Twist()) # Stop senden
        time.sleep(0.5)

        self.get_logger().info("Drehe rechts...")
        twist.angular.z = -angular_speed
        
        for _ in range(steps):
            if not rclpy.ok(): break
            self.cmd_vel_pub.publish(twist)
            time.sleep(sleep_time)

        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info("--- Pirouette beendet ---")
        time.sleep(0.5)

    def _send_navigation_sequence(self, poses: List[PoseStamped]) -> None:
        if not poses:
            return

        # 1. Zum ersten Punkt fahren
        self.get_logger().info("Navigiere zum ersten Punkt...")
        success = self._navigate_to_poses([poses[0]])
        
        if not success:
            self.get_logger().error("Erster Punkt konnte nicht erreicht werden.")
            return

        # 2. Kunststück ausführen
        self._perform_pirouette()

        # 3. Restliche Punkte abfahren
        if len(poses) > 1:
            remaining = poses[1:]
            self.get_logger().info(f"Navigiere zu den verbleibenden {len(remaining)} Punkten...")
            self._navigate_to_poses(remaining)
        
        self.get_logger().info("Rennen erfolgreich beendet!")

    def _navigate_to_poses(self, poses: List[PoseStamped]) -> bool:
        """Sendet Nav2-Goal und wartet blockierend auf Ankunft."""
        while not self.navigate_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().info(
                "'navigate_through_poses' Action Server nicht verfügbar, warte..."
            )

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        if not self._wait_future(send_goal_future, timeout_sec=5.0):
            self.get_logger().error("Navigation Goal Timeout.")
            return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Navigation Goal abgelehnt.")
            return False

        # Warten auf das Ergebnis
        result_future = goal_handle.get_result_async()
        
        # Polling Loop
        while rclpy.ok():
            if result_future.done():
                status = result_future.result().status
                if status == GoalStatus.STATUS_SUCCEEDED:
                    return True
                else:
                    self.get_logger().warn(f"Navigation fehlgeschlagen mit Status: {status}")
                    return False
            time.sleep(0.1)
        
        return False

    def _compute_path(
        self, start: PoseStamped, goals: List[PoseStamped]
    ) -> Optional[Path]:
        """Hilfsfunktion: Fragt Nav2 nach einem Pfad (Plan) für Längenberechnung."""
        while not self.compute_path_client.wait_for_server(timeout_sec=1.0):
            pass # Warten...

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.start = start
        goal_msg.goals = goals
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = True

        send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        if not self._wait_future(send_goal_future, timeout_sec=5.0):
            return None
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=10.0):
            return None
        result = result_future.result()
        if not result or result.status != GoalStatus.STATUS_SUCCEEDED:
            return None

        return result.result.path

    def _wait_future(self, future, timeout_sec: Optional[float]) -> bool:
        start = time.monotonic()
        while rclpy.ok():
            if future.done():
                return True
            if timeout_sec is not None and time.monotonic() - start > timeout_sec:
                return False
            time.sleep(0.05)
        return False

    def _path_length(self, path: Path) -> float:
        if not path or len(path.poses) < 2:
            return 0.0
        length = 0.0
        for idx in range(1, len(path.poses)):
            prev = path.poses[idx - 1].pose.position
            curr = path.poses[idx].pose.position
            dx = curr.x - prev.x
            dy = curr.y - prev.y
            length += math.hypot(dx, dy)
        return length

    def _to_map_frame(self, pose: PoseStamped) -> Optional[PoseStamped]:
        if pose.header.frame_id == self.map_frame:
            return pose
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, pose.header.frame_id, Time()
            )
        except TransformException:
            return None
        return tf2_geometry_msgs.do_transform_pose(pose, transform)

    def _get_robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time()
            )
        except TransformException:
            return None
        pose = PoseStamped()
        pose.header.frame_id = self.map_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SchnitzeljagdRace()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()