"""
Launch file for the Bicycle Gym simulation stack.

Starts: robot_state_publisher, the kinematic bicycle plant (bicycle_sim), the path generator and
lap analyzer (track_environment), RViz2, and ONE of the controllers of bicycle_control:

    controller:=none | teleop | lateral_pid | pure_pursuit | mpc

The Gazebo variant that used to be commented out here is archived in
launch/archive/gazebo_launch_commented.py.txt.
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

AUTONOMOUS = ['lateral_pid', 'pure_pursuit', 'mpc']


def _find(share_rel, source_rel):
    """Installed share path if present, else the source-tree path (for `ros2 launch <file>`)."""
    try:
        path = os.path.join(get_package_share_directory('bicycle_sim'), share_rel)
        if os.path.isfile(path):
            return path
    except (PackageNotFoundError, Exception):
        pass
    return os.path.abspath(os.path.join(os.path.dirname(__file__), source_rel))


def generate_launch_description():
    rviz_config = _find('bicycle.rviz', 'bicycle.rviz')
    urdf_file = _find(os.path.join('urdf', 'racecar.urdf'), os.path.join('..', 'urdf', 'racecar.urdf'))
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    controller = LaunchConfiguration('controller')
    track_file = LaunchConfiguration('track_file')

    is_autonomous = IfCondition(PythonExpression(
        ["'", controller, "'.lower() in ", str(AUTONOMOUS)]))
    is_teleop = IfCondition(PythonExpression(["'", controller, "'.lower() == 'teleop'"]))

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2 for visualization'),
        DeclareLaunchArgument('track_file', default_value='centerline_0.csv',
                              description='CSV track file (path and start pose)'),
        DeclareLaunchArgument('trajectory_type', default_value='centerline',
                              description='Trajectory type: centerline, sp, or iqp'),
        DeclareLaunchArgument('controller', default_value='none',
                              description='none, teleop, lateral_pid, pure_pursuit or mpc'),
        DeclareLaunchArgument('analyzer', default_value='true',
                              description='Launch lap analyzer and HUD'),
        DeclareLaunchArgument('use_cruise_control', default_value='false',
                              description='Closed-loop speed hold in the teleop bridge'),
        DeclareLaunchArgument('target_speed', default_value='4.0',
                              description='Base cruise speed [m/s] for autonomous modes'),
        DeclareLaunchArgument('velocity_mode', default_value='curvature',
                              description="'curvature' (profiled) or 'constant'"),
        DeclareLaunchArgument('summary_file', default_value='/tmp/lap_summary.json',
                              description='Where the lap analyzer writes its JSON summary'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='bicycle_sim', executable='sim_node', name='kinematic_bicycle',
             output='screen', arguments=['--track', track_file],
             parameters=[{'wheelbase_length': 1.25, 'dt': 0.1, 'car_name': 'ego_racecar'}]),

        Node(package='track_environment', executable='path_gen', name='path_gen',
             output='screen',
             parameters=[{'track_file': track_file,
                          'trajectory_type': LaunchConfiguration('trajectory_type'),
                          'close_loop': True}]),

        Node(package='track_environment', executable='lap_analyzer', name='lap_analyzer',
             output='screen',
             parameters=[{'summary_file': LaunchConfiguration('summary_file')}],
             condition=IfCondition(LaunchConfiguration('analyzer'))),

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', rviz_config], condition=IfCondition(LaunchConfiguration('rviz'))),

        # Autonomous modes: one node, mode selected by the `controller` argument
        Node(package='bicycle_control', executable='controller', name='controller',
             output='screen',
             parameters=[{
                 'control_mode': controller,
                 'target_speed': ParameterValue(LaunchConfiguration('target_speed'),
                                                value_type=float),
                 'velocity_mode': LaunchConfiguration('velocity_mode'),
             }],
             condition=is_autonomous),

        # Keyboard teleoperation bridge (drive with: ros2 run teleop_twist_keyboard ...)
        Node(package='bicycle_control', executable='teleop_bridge', name='teleop_bridge',
             output='screen',
             parameters=[{'use_cruise_control': ParameterValue(
                 LaunchConfiguration('use_cruise_control'), value_type=bool)}],
             condition=is_teleop),
    ])
