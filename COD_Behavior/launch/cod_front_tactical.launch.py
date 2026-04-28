from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    nav_pose_yaml = PathJoinSubstitution([
        FindPackageShare("cod_behavior"),
        "launch",
        "cod_pose_tactical_front.yaml"
    ])

    serial_node = Node(
        package="cod_serial",
        executable="talker",
        name="cod_serial",
        output="screen",
        parameters=[{
            "enable_downlink_receive": False
        }],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    behavior_node = Node(
        package="cod_behavior",
        executable="tree_1",
        name="cod_behavior",
        output="screen",
        parameters=[nav_pose_yaml],
        arguments=["--ros-args", "--log-level", "warn"]
    )

    return LaunchDescription([
        serial_node,
        behavior_node
    ])
