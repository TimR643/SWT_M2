from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription(
        [
            # Node(
            #     package="sopias4_application",
            #     # namespace='turtlesim1',
            #     executable="controller",
            #     name="controller",
            # ),
            Node(
                package="sopias4_application",
                # namespace='layer',
                executable="layer",
                name="layer",
                namespace=namespace,
            ),
            Node(
                package="sopias4_application",
                # namespace='layer',
                executable="planner",
                name="planner",
                namespace=namespace,
            ),
        ]
    )
