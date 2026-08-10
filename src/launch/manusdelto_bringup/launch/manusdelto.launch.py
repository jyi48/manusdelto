"""Standalone Manus glove + DG5F hand test rig — no RBY1, no vive, no pedal.

Starts, on one PC:
  manus_data_publisher (manus_ros2)  -> /manus_glove_0, /manus_glove_1
  manus_tesollo_node                 -> hand reference topics (see below)
  dg5f_driver / dg5f_s_driver (ros2_control + PID, vendored from tesollo_ros2)
  manusdelto_gui                     -> Calibrate / Pause Stream / Retarget mode

hand_ns picks which hands, hand_model picks which hardware:

  hand_model:=m (default)          -> dg5f_driver, one namespace for both hands
    dg5f_both  (default) -> dg5f_both_pid_all_controller.launch.py
    dg5f_left            -> dg5f_left_pid_all_controller.launch.py
    dg5f_right           -> dg5f_right_pid_all_controller.launch.py
    reference: /{hand_ns}/{lj,rj}_dg_pospid/reference

  hand_model:=s                    -> dg5f_s_driver, one namespace per hand
    dg5f_both            -> both single-hand launches (the S ships no
                            both-hand launch: left and right share the same
                            controller and joint names)
    dg5f_left/dg5f_right -> dg5f_s_{left,right}_pid_all_controller.launch.py
    reference: /dg5f_s_{left,right}/joint_pospid/reference
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
    hand_model = LaunchConfiguration('hand_model')
    use_ik = LaunchConfiguration('use_ik')
    orientation_weight = LaunchConfiguration('orientation_weight')
    use_gui = LaunchConfiguration('use_gui')

    dg5f_right_ip = LaunchConfiguration('dg5f_right_ip')
    dg5f_right_port = LaunchConfiguration('dg5f_right_port')
    dg5f_left_ip = LaunchConfiguration('dg5f_left_ip')
    dg5f_left_port = LaunchConfiguration('dg5f_left_port')
    delto_ip = LaunchConfiguration('delto_ip')
    delto_port = LaunchConfiguration('delto_port')

    # hand_ns picks which hands to bring up; hand_model picks which driver.
    # The S has no both-hand launch (left and right share controller and joint
    # names, so they can't live in one namespace) -- "both" runs its two
    # single-hand launches, each in its own namespace.
    def _m(side):
        return PythonExpression(
            ["'", hand_model, "' != 's' and '", hand_ns, "' == '", side, "'"])

    def _s(side):
        return PythonExpression(
            ["'", hand_model, "' == 's' and '", hand_ns, "' in ('", side,
             "', 'dg5f_both')"])

    is_both = _m('dg5f_both')
    is_left = _m('dg5f_left')
    is_right = _m('dg5f_right')
    is_s_left = _s('dg5f_left')
    is_s_right = _s('dg5f_right')

    dg5f_driver_share = FindPackageShare('dg5f_driver')
    dg5f_s_driver_share = FindPackageShare('dg5f_s_driver')

    return LaunchDescription([

        # ── Launch arguments ───────────────────────────────────────────────
        DeclareLaunchArgument(
            'hand_ns', default_value='dg5f_both',
            description='Which dg5f_driver stack to bring up: '
                        'dg5f_both, dg5f_left, or dg5f_right'),
        DeclareLaunchArgument(
            'hand_model', default_value='m',
            description='DG5F variant: m (dg5f_driver, hand_ns namespace, '
                        'lj_/rj_ joints) or s (dg5f_s_driver, one namespace '
                        'per hand, joint_* names). Picks both the driver '
                        'launched here and how manus_tesollo wires itself; '
                        'the retarget side is also switchable live from the '
                        'GUI.'),
        DeclareLaunchArgument(
            'use_ik', default_value='false',
            description='Start manus_tesollo in ik mode (requires pinocchio)'),
        DeclareLaunchArgument(
            'orientation_weight', default_value='1.0',
            description='IK orientation task weight in manus_tesollo'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Launch manusdelto_gui'),

        # Both-hand IP/port (used only when hand_ns:=dg5f_both). Gripper factory
        # link-local defaults: left=169.254.186.73, right=169.254.186.72.
        DeclareLaunchArgument('dg5f_right_ip', default_value='169.254.186.72'),
        DeclareLaunchArgument('dg5f_right_port', default_value='502'),
        DeclareLaunchArgument('dg5f_left_ip', default_value='169.254.186.73'),
        DeclareLaunchArgument('dg5f_left_port', default_value='502'),

        # Single-hand IP/port (used only when hand_ns:=dg5f_left or dg5f_right).
        # Factory default = left hand (169.254.186.73).
        DeclareLaunchArgument('delto_ip', default_value='169.254.186.73'),
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
                'hand_model': hand_model,
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

        # ── Hardware: DG5F-S driver (hand_model:=s) ─────────────────────────
        # One launch per hand, each in its own namespace (dg5f_s_left /
        # dg5f_s_right). hand_ns:=dg5f_both brings up both. The per-hand IPs
        # reuse dg5f_left_ip/dg5f_right_ip so both models are configured the
        # same way; the S launches take them as `delto_ip`.
        GroupAction(
            condition=IfCondition(is_s_left),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        dg5f_s_driver_share,
                        '/launch/dg5f_s_left_pid_all_controller.launch.py']),
                    launch_arguments={
                        'delto_ip': dg5f_left_ip, 'delto_port': dg5f_left_port,
                    }.items(),
                ),
            ],
        ),
        GroupAction(
            condition=IfCondition(is_s_right),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        dg5f_s_driver_share,
                        '/launch/dg5f_s_right_pid_all_controller.launch.py']),
                    launch_arguments={
                        'delto_ip': dg5f_right_ip, 'delto_port': dg5f_right_port,
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
