import os
import re
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Container batch layout (mirrors smartport_description/scripts/generate_world.py).
# Containers are laid out as NUM_BATCHES_Y long stripes ("columns" when viewed
# from outside) running along X. Each stripe stacks NUM_BATCHES_X batches of
# BATCH_NX x BATCH_NY containers separated by BATCH_GAP.
CX, CY = 6.0, 1.8
INNER_GAP = 0.2
BATCH_NX = 5
BATCH_NY = 2
BATCH_GAP = 12.0
START_X = 20.0
START_Y = 30.0
NUM_BATCHES_X = 4
NUM_BATCHES_Y = 4   # also == number of container columns == number of RTGs

BATCH_W = BATCH_NX * CX + (BATCH_NX - 1) * INNER_GAP   # 30.8 m along X
BATCH_H = BATCH_NY * CY + (BATCH_NY - 1) * INNER_GAP   #  3.8 m along Y

# Full column span in X: NUM_BATCHES_X batches end-to-end with BATCH_GAP between
# them. Each RTG is parked at the X-center of its column so its 8m carriage and
# 10m wheelbase can roll either way along the ~141m stripe.
COLUMN_X_START = START_X
COLUMN_X_END = START_X + (NUM_BATCHES_X - 1) * (BATCH_W + BATCH_GAP) + BATCH_W
COLUMN_X_CENTER = (COLUMN_X_START + COLUMN_X_END) / 2.0

# RTG geometry from RTG_Crane.urdf: legs at +/-5.2m in Y from the crane's
# base_link origin. We park the gantry so the +Y leg (right side when looking
# down +X) sits CLEARANCE past the +Y container edge; the -Y leg ends up
# ~6.1m past the -Y container edge, opening up the staging lane the user wants
# at "columns 1, 2".
RTG_LEG_OFFSET = 5.2
RTG_RIGHT_CLEARANCE = 0.5

JOINT_TOPICS = ('x_carriage', 'trolley', 'hoist_mid', 'spreader')


def _make_namespaced_urdf(base_text: str, ns: str, out_dir: str) -> str:
    """Rewrite gz transport topics to be unique per RTG and drop the broken
    container_brown DetachableJoint plugin. Returns the path to the new URDF."""
    text = base_text

    for joint in JOINT_TOPICS:
        text = text.replace(f'/rtg/{joint}/cmd_pos', f'/{ns}/{joint}/cmd_pos')
    text = text.replace('/rtg/attach', f'/{ns}/attach')

    # DiffDrive plugin uses relative topic names inside the <plugin> block.
    text = text.replace('<topic>cmd_vel</topic>', f'<topic>/{ns}/cmd_vel</topic>')
    text = text.replace('<odom_topic>odom</odom_topic>',
                        f'<odom_topic>/{ns}/odom</odom_topic>')

    # Strip the DetachableJoint plugin entirely: it hard-codes
    # child_model=container_brown which does not exist in port.sdf.
    text = re.sub(
        r'\s*<plugin\b[^>]*DetachableJoint[^/]*?>.*?</plugin>',
        '',
        text,
        flags=re.DOTALL,
    )

    path = os.path.join(out_dir, f'{ns}.urdf')
    with open(path, 'w') as f:
        f.write(text)
    return path


def generate_launch_description():
    pkg_description = get_package_share_directory('smartport_description')
    base_urdf_file = os.path.join(pkg_description, 'model', 'RTG_Crane.urdf')
    truck_urdf_file = os.path.join(pkg_description, 'model', 'tractor.urdf')
    world_file = os.path.join(pkg_description, 'worlds', 'port.sdf')

    with open(base_urdf_file, 'r') as infp:
        base_urdf = infp.read()

    urdf_dir = tempfile.mkdtemp(prefix='smartport_rtg_')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # Truck staging lane: first RTG column's -Y leg ends at ~23.9 m in Y.
    # Park the truck-trailer in the open lane (Y ≈ 15) centred along X.
    truck_x = COLUMN_X_CENTER
    truck_y = START_Y - 15.0

    spawn_truck = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', truck_urdf_file,
            '-name', 'truck_trailer',
            '-x', f'{truck_x:.3f}',
            '-y', f'{truck_y:.3f}',
            '-z', '1.0',
        ],
        output='screen'
    )

    spawn_nodes = []
    controller_nodes = []
    bridge_topics = [
        '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/gps/fix@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat',
    ]

    for col in range(NUM_BATCHES_Y):
        rtg_id = col + 1
        ns = f'rtg_{rtg_id}'

        # Y position: park so the +Y RTG leg sits RTG_RIGHT_CLEARANCE past the
        # +Y container edge. The -Y leg opens up the staging lane to the left.
        column_y0 = START_Y + col * (BATCH_H + BATCH_GAP)
        container_right_edge = column_y0 + BATCH_H
        rtg_y = container_right_edge + RTG_RIGHT_CLEARANCE - RTG_LEG_OFFSET
        rtg_x = COLUMN_X_CENTER

        urdf_path = _make_namespaced_urdf(base_urdf, ns, urdf_dir)

        spawn_nodes.append(Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', urdf_path,
                '-name', ns,
                '-x', f'{rtg_x:.3f}',
                '-y', f'{rtg_y:.3f}',
                '-z', '0.5',
            ],
            output='screen'
        ))

        controller_nodes.append(Node(
            package='smartport_rtgcontroller',
            executable='rtg_controller_node',
            name=f'controller_{ns}',
            remappings=[
                ('/rtg/cmd_pos', f'/{ns}/cmd_pos'),
                ('/rtg/x_carriage/cmd_pos', f'/{ns}/x_carriage/cmd_pos'),
                ('/rtg/trolley/cmd_pos', f'/{ns}/trolley/cmd_pos'),
                ('/rtg/hoist_mid/cmd_pos', f'/{ns}/hoist_mid/cmd_pos'),
                ('/rtg/spreader/cmd_pos', f'/{ns}/spreader/cmd_pos'),
                ('/rtg/attach', f'/{ns}/attach'),
            ],
            output='screen'
        ))

        bridge_topics += [
            f'/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{ns}/x_carriage/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/{ns}/trolley/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/{ns}/hoist_mid/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/{ns}/spreader/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            f'/{ns}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
        ]

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_topics,
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_truck,
        *spawn_nodes,
        bridge,
        *controller_nodes,
    ])
