"""Standalone Manus glove + DG5F hand test rig — no RBY1, no vive, no pedal.

Starts, on one PC:
  manus_data_publisher (manus_ros2)  -> /manus_glove_0, /manus_glove_1
  manus_tesollo_node                 -> /{hand_ns}/{lj,rj}_dg_pospid/reference
  dg5f_driver (ros2_control + PID controllers, vendored from tesollo_ros2)
  manusdelto_gui                     -> Calibrate / Pause Stream / Retarget mode

Use hand_ns to choose which dg5f_driver launch file gets included:
  dg5f_both  (default) -> dg5f_both_pid_all_controller.launch.py
  dg5f_left            -> dg5f_left_pid_all_controller.launch.py
  dg5f_right           -> dg5f_right_pid_all_controller.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    hand_ns = LaunchConfiguration('hand_ns')
    use_ik = LaunchConfiguration('use_ik')
    orientation_weight = LaunchConfiguration('orientation_weight')
    use_gui = LaunchConfiguration('use_gui')

    dg5f_right_ip = LaunchConfiguration('dg5f_right_ip')
    dg5f_right_port = LaunchConfiguration('dg5f_right_port')
    dg5f_left_ip = LaunchConfiguration('dg5f_left_ip')
    dg5f_left_port = LaunchConfiguration('dg5f_left_port')
    delto_ip = LaunchConfiguration('delto_ip')
    delto_port = LaunchConfiguration('delto_port')

    is_both = PythonExpression(["'", hand_ns, "' == 'dg5f_both'"])
    is_left = PythonExpression(["'", hand_ns, "' == 'dg5f_left'"])
    is_right = PythonExpression(["'", hand_ns, "' == 'dg5f_right'"])

    dg5f_driver_share = FindPackageShare('dg5f_driver')

    return LaunchDescription([

        # ── Launch arguments ───────────────────────────────────────────────
        DeclareLaunchArgument(
            'hand_ns', default_value='dg5f_both',
            description='Which dg5f_driver stack to bring up: '
                        'dg5f_both, dg5f_left, or dg5f_right'),
        DeclareLaunchArgument(
            'use_ik', default_value='false',
            description='Start manus_tesollo in ik mode (requires pinocchio)'),
        DeclareLaunchArgument(
            'orientation_weight', default_value='1.0',
            description='IK orientation task weight in manus_tesollo'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Launch manusdelto_gui'),

        # Both-hand IP/port (used only when hand_ns:=dg5f_both). Test rig:
        # left=192.168.1.151, right=192.168.1.152.
        DeclareLaunchArgument('dg5f_right_ip', default_value='192.168.1.152'),
        DeclareLaunchArgument('dg5f_right_port', default_value='502'),
        DeclareLaunchArgument('dg5f_left_ip', default_value='192.168.1.151'),
        DeclareLaunchArgument('dg5f_left_port', default_value='502'),

        # Single-hand IP/port (used only when hand_ns:=dg5f_left or dg5f_right)
        DeclareLaunchArgument('delto_ip', default_value='192.168.1.151'),
        DeclareLaunchArgument('delto_port', default_value='502'),

        # ── Input: Manus glove publisher ────────────────────────────────────
        Node(
            package='manus_ros2',
            executable='manus_data_publisher',
            name='manus_data_publisher',
            output='screen',
        ),

        # ── Retargeting: Manus -> DG5F joint references ─────────────────────
        Node(
            package='manus_tesollo',
            executable='manus_tesollo_node',
            name='manus_tesollo',
            output='screen',
            parameters=[{
                'hand_ns': hand_ns,
                'use_ik': use_ik,
                'orientation_weight': orientation_weight,
            }],
        ),

        # ── Hardware: DG5F driver (ros2_control + PID controllers) ──────────
        GroupAction(
            condition=IfCondition(is_both),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        dg5f_driver_share, '/launch/dg5f_both_pid_all_controller.launch.py']),
                    launch_arguments={
                        'dg5f_right_ip': dg5f_right_ip,
                        'dg5f_right_port': dg5f_right_port,
                        'dg5f_left_ip': dg5f_left_ip,
                        'dg5f_left_port': dg5f_left_port,
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            condition=IfCondition(is_left),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        dg5f_driver_share, '/launch/dg5f_left_pid_all_controller.launch.py']),
                    launch_arguments={
                        'delto_ip': delto_ip, 'delto_port': delto_port,
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            condition=IfCondition(is_right),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        dg5f_driver_share, '/launch/dg5f_right_pid_all_controller.launch.py']),
                    launch_arguments={
                        'delto_ip': delto_ip, 'delto_port': delto_port,
                    }.items(),
                ),
            ],
        ),

        # ── GUI ────────────────────────────────────────────────────────────
        GroupAction(
            condition=IfCondition(use_gui),
            actions=[
                Node(
                    package='manusdelto_gui',
                    executable='manusdelto_gui_node',
                    name='manusdelto_gui',
                    output='screen',
                ),
            ],
        ),
    ])
