"""Standalone Manus glove + DG5F hand test rig — no RBY1, no vive, no pedal.

Starts, on one PC:
  manus_data_publisher (manus_ros2)  -> /manus_glove_0, /manus_glove_1
  manus_tesollo_node                 -> hand reference topics (see below)
  dg5f_driver / dg5f_s_driver (ros2_control + PID, vendored from tesollo_ros2)
  manusdelto_gui                     -> Calibrate / Pause Stream / Retarget mode

hands picks which hands, hand_model picks which hardware. Both models run one
vendor launch per hand, each in its own namespace:

  hand_model:=m  -> dg5f_driver   dg5f_{left,right}_pid_all_controller.launch.py
    reference: /dg5f_left/lj_dg_pospid/reference, /dg5f_right/rj_dg_pospid/reference
  hand_model:=s  -> dg5f_s_driver dg5f_s_{left,right}_pid_all_controller.launch.py
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
    hands = LaunchConfiguration('hands')
    hand_model = LaunchConfiguration('hand_model')
    use_ik = LaunchConfiguration('use_ik')
    orientation_weight = LaunchConfiguration('orientation_weight')
    use_gui = LaunchConfiguration('use_gui')

    ip = {'left': LaunchConfiguration('dg5f_left_ip'),
          'right': LaunchConfiguration('dg5f_right_ip')}
    port = {'left': LaunchConfiguration('dg5f_left_port'),
            'right': LaunchConfiguration('dg5f_right_port')}

    # (hand_model, driver package, launch-file prefix)
    drivers = [('m', 'dg5f_driver', 'dg5f'), ('s', 'dg5f_s_driver', 'dg5f_s')]

    def _driver(model, pkg, prefix, side):
        return GroupAction(
            condition=IfCondition(PythonExpression(
                ["'", hand_model, "' == '", model, "' and '", hands,
                 "' in ('", side, "', 'both')"])),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare(pkg),
                        f'/launch/{prefix}_{side}_pid_all_controller.launch.py']),
                    launch_arguments={
                        'delto_ip': ip[side], 'delto_port': port[side],
                    }.items(),
                ),
            ],
        )

    return LaunchDescription([

        # ── Launch arguments ───────────────────────────────────────────────
        DeclareLaunchArgument(
            'hands', default_value='both',
            description='Which hands to bring up: both, left, or right'),
        DeclareLaunchArgument(
            'hand_model', default_value='s',
            description='DG5F variant: m (dg5f_driver, dg5f_left/right, '
                        'lj_/rj_ joints) or s (dg5f_s_driver, '
                        'dg5f_s_left/right, joint_* names). Picks both the '
                        'driver launched here and how manus_tesollo wires '
                        'itself; the retarget side is also switchable live '
                        'from the GUI.'),
        DeclareLaunchArgument(
            'use_ik', default_value='false',
            description='Start manus_tesollo in ik mode (requires pinocchio)'),
        DeclareLaunchArgument(
            'orientation_weight', default_value='1.0',
            description='IK orientation task weight in manus_tesollo'),
        DeclareLaunchArgument(
            'use_gui', default_value='true',
            description='Launch manusdelto_gui'),

        # Per-hand IP/port, same for both models. Gripper factory link-local
        # defaults: left=169.254.186.73, right=169.254.186.72.
        DeclareLaunchArgument('dg5f_left_ip', default_value='169.254.186.73'),
        DeclareLaunchArgument('dg5f_left_port', default_value='502'),
        DeclareLaunchArgument('dg5f_right_ip', default_value='169.254.186.72'),
        DeclareLaunchArgument('dg5f_right_port', default_value='502'),

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
                'hand_model': hand_model,
                'use_ik': use_ik,
                'orientation_weight': orientation_weight,
            }],
        ),

        # ── Hardware: DG5F driver, one launch per hand ──────────────────────
        *[_driver(model, pkg, prefix, side)
          for model, pkg, prefix in drivers
          for side in ('left', 'right')],

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
