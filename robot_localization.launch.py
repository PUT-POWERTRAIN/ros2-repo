#!/usr/bin/env python3
from launch import LaunchDescription, LaunchService
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=['./params/my_navsat.yaml',
                        {"use_sim_time": True},],
            remappings=[
                ('/imu/data', '/wamv/sensors/imu/imu/data'),
                ('/gps/fix', '/wamv/sensors/gps/gps/fix')
            ],
        ),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_odom",
            output="screen",
            parameters=["./params/my_ekf.yaml", {"use_sim_time": True}],
        )
    ])

if __name__ == "__main__":
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
