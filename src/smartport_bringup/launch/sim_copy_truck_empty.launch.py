"""
sim_copy_truck_empty.launch.py
────────────────────────────────────────────────────────────────────────────
Launches the copy_truck into the empty world.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xml.etree.ElementTree as ET
from launch_ros.actions import Node

def generate_launch_description():
    pkg_description = get_package_share_directory('smartport_description')
    pkg_plugins = get_package_share_directory('smartport_gazebo_plugins')
    
    urdf_file  = os.path.join(pkg_description, 'model', 'copy_truck.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'empty.sdf')

    # Add the custom plugin library path to Gazebo's plugin path
    set_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=os.path.join(pkg_plugins, '../../lib/smartport_gazebo_plugins')
    )

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Parse vehicle parameters directly from URDF to feed into nodes
    v_params = {
        'wheelbase': 2.5,
        'track_width': 2.4,
        'max_steer_rad': 0.5236
    }
    try:
        tree = ET.parse(urdf_file)
        root = tree.getroot()
        for plugin in root.iter('plugin'):
            if plugin.get('name') == 'pacejka_plugin':
                if plugin.find('wheelbase') is not None:
                    v_params['wheelbase'] = float(plugin.find('wheelbase').text)
                if plugin.find('track_width') is not None:
                    v_params['track_width'] = float(plugin.find('track_width').text)
                if plugin.find('max_steer_rad') is not None:
                    v_params['max_steer_rad'] = float(plugin.find('max_steer_rad').text)
    except Exception as e:
        print(f"Failed to parse URDF parameters: {e}")

    # ── Gazebo Harmonic simulator ────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ── Robot description (URDF → /robot_description + TF) ──────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    container_urdf = os.path.join(pkg_description, 'model', 'containers', 'container_blue.urdf')

    # ── Spawn tractor entity into Gazebo ─────────────────────────────────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', urdf_file,
            '-name', 'copy_truck',
            '-x', '0',
            '-y', '0',
            '-z', '1.0',
        ],
        output='screen',
    )

    # ── Spawn container 10 m ahead of the truck (+X) ─────────────────────────
    # Z = ground_top(0.05) + half_height(1.0) = 1.05
    spawn_container = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', container_urdf,
            '-name', 'container_0',
            '-x', '10.0',
            '-y', '0.0',
            '-z', '1.05',
        ],
        output='screen',
    )

    # ── ROS ↔ Gazebo bridge ──────────────────────────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
            '/model/copy_truck/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/world/empty/model/copy_truck/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/truck/fl_steer/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/truck/fr_steer/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/truck/drive@std_msgs/msg/Float64]gz.msgs.Double',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/world/empty/model/copy_truck/joint_state', '/joint_states'),
        ],
        output='screen',
    )

    # ── Ackermann controller @ 1000 Hz ──────────────────────────────────────
    ackermann_controller = Node(
        package='smartport_truckcontroller',
        executable='ackermann_controller_node',
        name='ackermann_controller',
        output='screen',
        parameters=[{
            'wheelbase':     v_params['wheelbase'],
            'track_width':   v_params['track_width'],
            'max_steer_rad': v_params['max_steer_rad'],
            'loop_rate_hz':  1000.0,
        }],
    )

    # ── Wheel state node ─────────────────────────────────────────────────────
    wheel_state = Node(
        package='smartport_truckcontroller',
        executable='wheel_state_node',
        name='wheel_state',
        output='screen',
    )

    # ── Vehicle state node ───────────────────────────────────────────────────
    vehicle_state = Node(
        package='smartport_truckcontroller',
        executable='vehicle_state_node',
        name='vehicle_state',
        output='screen',
    )

    # ── RViz2 Visualization ──────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        set_plugin_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        spawn_container,
        bridge,
        ackermann_controller,
        wheel_state,
        vehicle_state,
        rviz,
    ])
