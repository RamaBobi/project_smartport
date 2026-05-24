#!/usr/bin/env python3
"""Generate port.sdf with water, port surface, and dynamic container batches.

Each container is a separate dynamic SDF model so it can be individually
selected and moved in Gazebo. They have full mass + inertia + collision and
spawn slightly above the port surface so they drop and settle on play.

Edit the constants at the top to change layout, then run:
    python3 generate_world.py > ../worlds/port.sdf
"""
import random
import sys

random.seed(42)

# Container dimensions (per spec: max X=6, Y locked at 1.8, max Z=2)
CX, CY, CZ = 6.0, 1.8, 2.0

# 5 muted color variants (matches the URDFs in model/containers/)
VARIANTS = {
    'red':   (0.55, 0.15, 0.15),
    'blue':  (0.20, 0.30, 0.50),
    'green': (0.20, 0.40, 0.25),
    'brown': (0.45, 0.35, 0.20),
    'grey':  (0.35, 0.35, 0.40),
}

# Container physical properties
CONTAINER_MASS = 2300.0  # empty 20-ft container, kg
# Box inertia: ixx = m/12*(Y^2+Z^2), iyy = m/12*(X^2+Z^2), izz = m/12*(X^2+Y^2)
IXX = round(CONTAINER_MASS / 12.0 * (CY * CY + CZ * CZ), 2)
IYY = round(CONTAINER_MASS / 12.0 * (CX * CX + CZ * CZ), 2)
IZZ = round(CONTAINER_MASS / 12.0 * (CX * CX + CY * CY), 2)

# Batch layout: 5 along X, 2 along Y, with 0.2m gap between containers in a batch
BATCH_NX = 5
BATCH_NY = 2
INNER_GAP = 0.2

# Number of batches (each batch has BATCH_NX*BATCH_NY = 10 dynamic containers)
NUM_BATCHES_X = 4
NUM_BATCHES_Y = 4

# Spacing and origin (per spec: containers within x=[20,480], y=[30,470]).
# BATCH_GAP must be > RTG_WIDTH(10.4) - BATCH_H(3.8) = 6.6m or adjacent RTGs
# collide. 12m gives ~5.4m of clear space between adjacent RTG legs.
BATCH_GAP = 12.0
START_X = 20.0
START_Y = 30.0

# Drop height: containers spawn this much above resting position
DROP_HEIGHT = 0.05

# Footprint of one batch (with the inner 0.2m gaps)
BATCH_W = BATCH_NX * CX + (BATCH_NX - 1) * INNER_GAP   # 5*6 + 4*0.2 = 30.8 m
BATCH_H = BATCH_NY * CY + (BATCH_NY - 1) * INNER_GAP   # 2*1.8 + 1*0.2 = 3.8 m

# Port surface
PORT_SIZE = 500.0
PORT_THICKNESS = 1.0


def container_block(name, x, y, z, variant):
    r, g, b = VARIANTS[variant]
    return (
        f'    <model name="{name}">\n'
        f'      <pose>{x:.3f} {y:.3f} {z:.3f} 0 0 0</pose>\n'
        f'      <link name="link">\n'
        f'        <inertial>\n'
        f'          <mass>{CONTAINER_MASS}</mass>\n'
        f'          <inertia>\n'
        f'            <ixx>{IXX}</ixx><ixy>0</ixy><ixz>0</ixz>\n'
        f'            <iyy>{IYY}</iyy><iyz>0</iyz>\n'
        f'            <izz>{IZZ}</izz>\n'
        f'          </inertia>\n'
        f'        </inertial>\n'
        f'        <collision name="collision">\n'
        f'          <geometry><box><size>{CX} {CY} {CZ}</size></box></geometry>\n'
        f'        </collision>\n'
        f'        <visual name="visual">\n'
        f'          <geometry><box><size>{CX} {CY} {CZ}</size></box></geometry>\n'
        f'          <material>\n'
        f'            <ambient>{r} {g} {b} 1</ambient>\n'
        f'            <diffuse>{r} {g} {b} 1</diffuse>\n'
        f'          </material>\n'
        f'        </visual>\n'
        f'      </link>\n'
        f'    </model>'
    )


containers = []
for bi in range(NUM_BATCHES_X):
    for bj in range(NUM_BATCHES_Y):
        batch_x0 = START_X + bi * (BATCH_W + BATCH_GAP)
        batch_y0 = START_Y + bj * (BATCH_H + BATCH_GAP)
        for ci in range(BATCH_NX):
            for cj in range(BATCH_NY):
                cx = batch_x0 + ci * (CX + INNER_GAP) + CX / 2
                cy = batch_y0 + cj * (CY + INNER_GAP) + CY / 2
                cz = CZ / 2 + DROP_HEIGHT
                variant = random.choice(list(VARIANTS.keys()))
                name = f"container_{bi:02d}_{bj:02d}_{ci}_{cj}"
                containers.append(container_block(name, cx, cy, cz, variant))

containers_block = '\n'.join(containers)
total = len(containers)

sdf = f'''<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="port">

    <!-- 1. PHYSICS AND SOLVER -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- 2. GAZEBO PLUGINS -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <!-- 3. GPS REFERENCE DATUM (Jakarta) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>-6.200000</latitude_deg>
      <longitude_deg>106.816666</longitude_deg>
      <elevation>0.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- 4. LIGHTING -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>250 250 100 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation><range>2000</range><constant>0.9</constant><linear>0.001</linear><quadratic>0.0001</quadratic></attenuation>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- 5. WATER (visualization only, no collision) -->
    <model name="water">
      <static>true</static>
      <pose>250 250 -0.3 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>2000 2000</size></plane></geometry>
          <material>
            <ambient>0.05 0.15 0.25 1</ambient>
            <diffuse>0.10 0.25 0.40 1</diffuse>
            <specular>0.20 0.30 0.45 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 6. PORT SURFACE ({PORT_SIZE:.0f}x{PORT_SIZE:.0f} m concrete platform, corner at origin) -->
    <model name="port_surface">
      <static>true</static>
      <pose>{PORT_SIZE/2} {PORT_SIZE/2} {-PORT_THICKNESS/2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{PORT_SIZE} {PORT_SIZE} {PORT_THICKNESS}</size></box></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><box><size>{PORT_SIZE} {PORT_SIZE} {PORT_THICKNESS}</size></box></geometry>
          <material>
            <ambient>0.45 0.45 0.45 1</ambient>
            <diffuse>0.55 0.55 0.55 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- 7. DYNAMIC CONTAINERS: {NUM_BATCHES_X}x{NUM_BATCHES_Y} batches of {BATCH_NX}x{BATCH_NY} containers = {total} total -->
    <!--    Each container is a separate movable model with mass={CONTAINER_MASS} kg, full inertia, collision. -->
    <!--    Inner gap within a batch: {INNER_GAP} m. Gap between batches: {BATCH_GAP} m. -->
{containers_block}

  </world>
</sdf>
'''

sys.stdout.write(sdf)
sys.stderr.write(
    f"Generated {total} dynamic containers ({NUM_BATCHES_X}x{NUM_BATCHES_Y} batches of "
    f"{BATCH_NX}x{BATCH_NY}, {INNER_GAP}m inner gap, {BATCH_GAP}m between batches)\n"
)
