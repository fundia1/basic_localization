"""
dual_ekf_localization.launch.py — Dual EKF (Local + Global) with GPS

TF chain: map → odom → base_link

  ekf_local  : odom → base_link  (wheel odom + IMU)
  ekf_global : map  → odom       (GPS + wheel odom + IMU)
  gps_odometry : GPS /fix → /odometry/gps

Arguments:
  use_sim_time  — Use simulation clock (default: false)
  sync_imu      — Re-stamp IMU to wall clock for bag playback (default: false)

Usage:
  ros2 launch localization dual_ekf_localization.launch.py
  ros2 launch localization dual_ekf_localization.launch.py sync_imu:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    pkg_share = get_package_share_directory('localization')
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    sync_imu = LaunchConfiguration('sync_imu').perform(context).lower() == 'true'

    imu_topic = '/imu/data/synced' if sync_imu else '/imu/data'

    nodes = [
        # ── Wheel Odometry ───────────────────────────────────────────────
        Node(
            package='localization',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen',
            parameters=[{'pub_tf': False, 'use_sim_time': use_sim_time}],
        ),

        # ── EKF Local: odom → base_link ─────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='local_ekf_node',
            output='screen',
            parameters=[ekf_local_config, {'use_sim_time': use_sim_time, 'imu0': imu_topic}],
            remappings=[
                ('odometry/filtered', '/odometry/local'),
                ('/set_pose', '/set_pose_local'),
                ('/diagnostics', '/diagnostics_local'),
            ],
        ),

        # ── EKF Global: map → odom ──────────────────────────────────────
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='global_ekf_node',
            output='screen',
            parameters=[ekf_global_config, {'use_sim_time': use_sim_time, 'imu0': imu_topic}],
            remappings=[
                ('odometry/filtered', '/odometry/global'),
                ('/set_pose', '/set_pose_global'),
                ('/diagnostics', '/diagnostics_global'),
            ],
        ),

        # ── GPS Odometry: /ublox_gps_node/fix → /odometry/gps ───────────
        Node(
            package='localization',
            executable='gps_odometry',
            name='gps_odometry',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Static TF: base_link → imu_link (앞 1m, 위 0.5m) ────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu',
            arguments=['1.0', '0', '0.5', '0', '0', '0', 'base_link', 'imu_link'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Static TF: base_link → gps (앞 0.5m, 위 1.5m) ──────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_gps',
            arguments=['0.5', '0', '1.5', '0', '0', '0', 'base_link', 'gps'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Static TF: base_link → velodyne (앞 1m, 위 1m) ──────────────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_velodyne',
            arguments=['1.0', '0', '1.0', '0', '0', '0', 'base_link', 'velodyne'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ── Static TF: base_link → encoder_link (앞 1m, 오른쪽 0.5m) ────
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_encoder',
            arguments=['1.0', '-0.5', '0', '0', '0', '0', 'base_link', 'encoder_link'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    if sync_imu:
        nodes.append(
            Node(
                package='localization',
                executable='imu_time_sync',
                name='imu_time_sync',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('sync_imu', default_value='false',
                              description='Re-stamp IMU to wall clock (for bag playback)'),
        OpaqueFunction(function=_launch_setup),
    ])
