#!/usr/bin/env python3

import itertools
import math
import threading
import time
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses, NavigateThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException
import tf2_geometry_msgs  # noqa: F401 (required for do_transform_pose)


class SchnitzeljagdRace(Node):
    def __init__(self) -> None:
        super().__init__("schnitzeljagd_race")

        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.robot_frame = self.declare_parameter("robot_frame", "base_link").value
        self.waypoint_topic = self.declare_parameter(
            "waypoint_topic", "schnitzeljagd/waypoint_pose"
        ).value
        self.waypoint_count = int(self.declare_parameter("waypoint_count", 3).value)
        self.planner_id = self.declare_parameter("planner_id", "").value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.compute_path_client = ActionClient(
            self, ComputePathThroughPoses, "compute_path_through_poses"
        )
        self.navigate_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )

        self.waypoints: List[PoseStamped] = []
        self._lock = threading.Lock()
        self._processing = False

        self.create_subscription(PoseStamped, self.waypoint_topic, self._waypoint_cb, 10)
        self.create_timer(0.2, self._maybe_process)

        self.get_logger().info(
            f"Listening for {self.waypoint_count} waypoints on "
            f"'{self.waypoint_topic}'."
        )

    def _waypoint_cb(self, msg: PoseStamped) -> None:
        pose = self._to_map_frame(msg)
        if pose is None:
            self.get_logger().warn("Failed to transform waypoint to map frame.")
            return

        with self._lock:
            if len(self.waypoints) >= self.waypoint_count:
                return
            self.waypoints.append(pose)
            count = len(self.waypoints)

        self.get_logger().info(f"Waypoint {count}/{self.waypoint_count} recorded.")

    def _maybe_process(self) -> None:
        with self._lock:
            if self._processing or len(self.waypoints) < self.waypoint_count:
                return
            waypoints = list(self.waypoints[: self.waypoint_count])
            self.waypoints = []
            self._processing = True

        thread = threading.Thread(
            target=self._process_waypoints, args=(waypoints,), daemon=True
        )
        thread.start()

    def _process_waypoints(self, waypoints: List[PoseStamped]) -> None:
        try:
            start_pose = self._get_robot_pose()
            if start_pose is None:
                self.get_logger().warn("Could not determine robot pose; aborting.")
                return

            best_order = None
            best_length = math.inf

            for order in itertools.permutations(waypoints):
                path = self._compute_path(start_pose, list(order))
                length = self._path_length(path) if path else math.inf
                if length < best_length:
                    best_length = length
                    best_order = list(order)

            if not best_order:
                self.get_logger().error("No valid path found for any waypoint order.")
                return

            self.get_logger().info(
                f"Best route length: {best_length:.2f} m. Starting race."
            )
            self._send_navigation(best_order)
        finally:
            with self._lock:
                self._processing = False

    def _compute_path(
        self, start: PoseStamped, goals: List[PoseStamped]
    ) -> Optional[Path]:
        while not self.compute_path_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                "'compute_path_through_poses' action server not available, waiting..."
            )

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.start = start
        goal_msg.goals = goals
        goal_msg.planner_id = self.planner_id
        goal_msg.use_start = True

        send_goal_future = self.compute_path_client.send_goal_async(goal_msg)
        if not self._wait_future(send_goal_future, timeout_sec=5.0):
            self.get_logger().warn("Compute path goal did not respond in time.")
            return None
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn("Compute path request was rejected.")
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait_future(result_future, timeout_sec=10.0):
            self.get_logger().warn("Compute path result timed out.")
            return None
        result = result_future.result()
        if not result or result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn("Compute path failed for this permutation.")
            return None

        return result.result.path

    def _send_navigation(self, poses: List[PoseStamped]) -> None:
        while not self.navigate_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(
                "'navigate_through_poses' action server not available, waiting..."
            )

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = poses

        send_goal_future = self.navigate_client.send_goal_async(goal_msg)
        if not self._wait_future(send_goal_future, timeout_sec=5.0):
            self.get_logger().error("NavigateThroughPoses goal timed out.")
            return
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("NavigateThroughPoses request was rejected.")
            return

        self.get_logger().info("Navigation started.")

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
        except TransformException as exc:
            self.get_logger().warn(f"TF transform failed: {exc}")
            return None

        return tf2_geometry_msgs.do_transform_pose(pose, transform)

    def _get_robot_pose(self) -> Optional[PoseStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time()
            )
        except TransformException as exc:
            self.get_logger().warn(f"TF lookup failed: {exc}")
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
