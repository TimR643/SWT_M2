from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    namespace = LaunchConfiguration("namespace")

    planner_plugin = Node(
        package="sopias4_application",
        executable="planner_plugin",
        name="planner_plugin",
        namespace=namespace,
        output="screen",
    )

    gui = Node(
        package="sopias4_application",
        executable="gui",
        name="gui",
        namespace=namespace,
        output="screen",
    )

    layer_plugin = Node(
        package="sopias4_application",
        executable="layer_plugin",
        name="layer_plugin",
        namespace=namespace,
        output="screen",
    )

    schnitzeljagd_race = Node(
        package="sopias4_application",
        executable="schnitzeljagd_race",
        name="schnitzeljagd_race",
        namespace=namespace,
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        output="screen",
    )

    ld.add_action(namespace_arg)

    ld.add_action(gui)
    ld.add_action(planner_plugin)
    ld.add_action(layer_plugin)
    ld.add_action(schnitzeljagd_race)

    return ld
