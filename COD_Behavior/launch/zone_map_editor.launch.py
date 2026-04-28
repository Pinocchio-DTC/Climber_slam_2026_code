from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 直接在这里修改地图路径，不需要在命令行额外传参
    pgm_path = "/home/nucshao/Climber_slam_2026_COD_NAV/src/cod_bringup/maps/2025new1.pgm"
    map_yaml_path = "/home/nucshao/Climber_slam_2026_COD_NAV/src/cod_bringup/maps/2025new1.yaml"
    out_csv_path = PathJoinSubstitution([
        FindPackageShare("cod_behavior"),
        "launch",
        "zone_modes_front.csv",
    ])

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "python3",
                PathJoinSubstitution([
                    FindPackageShare("cod_behavior"),
                    "launch",
                    "zone_map_editor.py",
                ]),
                "--pgm",
                pgm_path,
                "--map-yaml",
                map_yaml_path,
                "--out",
                out_csv_path,
            ],
            output="screen",
        )
    ])
