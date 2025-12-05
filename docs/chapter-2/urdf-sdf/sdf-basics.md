# SDF Basics - Simulation Description Format

SDF (Simulation Description Format) is Gazebo's native format for describing robots, worlds, and simulation environments. While URDF is great for ROS integration, SDF provides more powerful features specifically designed for simulation.

---

## URDF vs SDF: Quick Comparison

| Feature | URDF | SDF |
|---------|------|-----|
| **Primary Use** | ROS robot description | Gazebo simulation |
| **World Description** | ❌ No | ✅ Yes |
| **Multiple Robots** | ❌ No | ✅ Yes |
| **Plugins** | Limited | ✅ Extensive |
| **Closed Kinematic Loops** | ❌ No | ✅ Yes |
| **Physics Engine Config** | Basic | ✅ Advanced |
| **Sensor Noise Models** | ❌ No | ✅ Yes |

**When to use each:**
- **URDF**: ROS-centric projects, need RViz visualization
- **SDF**: Complex simulations, multiple robots, advanced physics

---

## SDF Structure

An SDF file can describe an entire **world** (environment + robots + lighting + physics) or a single **model** (robot).

### World Definition

```xml
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="my_world">

    <!-- Physics engine configuration -->
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Lighting -->
    <light type="directional" name="sun">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
    </light>

    <!-- Ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Include a robot model -->
    <include>
      <uri>model://my_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>

  </world>
</sdf>
```

---

## Model Definition

A **model** in SDF is analogous to a robot in URDF, but with more features:

```xml
<sdf version="1.7">
  <model name="simple_box">

    <!-- Model pose in world -->
    <pose>0 0 0.5 0 0 0</pose>

    <!-- Static vs dynamic -->
    <static>false</static>

    <!-- Links -->
    <link name="box_link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.166</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>0.166</iyy>
          <iyz>0</iyz>
          <izz>0.166</izz>
        </inertia>
      </inertial>

      <collision name="box_collision">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
        </surface>
      </collision>

      <visual name="box_visual">
        <geometry>
          <box>
            <size>1 1 1</size>
          </box>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>

      <!-- Sensor attached to link -->
      <sensor name="box_camera" type="camera">
        <camera>
          <horizontal_fov>1.047</horizontal_fov>
          <image>
            <width>640</width>
            <height>480</height>
          </image>
          <clip>
            <near>0.1</near>
            <far>100</far>
          </clip>
        </camera>
        <always_on>1</always_on>
        <update_rate>30</update_rate>
      </sensor>
    </link>

    <!-- Plugins (for behavior/control) -->
    <plugin name="my_plugin" filename="libmy_plugin.so">
      <param>value</param>
    </plugin>

  </model>
</sdf>
```

---

## Key SDF Features

### 1. Physics Configuration

SDF allows fine-grained physics control:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000</real_time_update_rate>

  <ode>
    <solver>
      <type>quick</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

**Parameters:**
- `max_step_size`: Simulation timestep (smaller = more accurate, slower)
- `real_time_factor`: Target simulation speed (1.0 = realtime, 2.0 = 2x faster)
- `iters`: Solver iterations (more = accurate, slower)

### 2. Advanced Materials

```xml
<material>
  <script>
    <uri>file://media/materials/scripts/gazebo.material</uri>
    <name>Gazebo/Wood</name>
  </script>
  <ambient>0.5 0.3 0.1 1</ambient>
  <diffuse>0.8 0.5 0.2 1</diffuse>
  <specular>0.1 0.1 0.1 1</specular>
  <emissive>0 0 0 1</emissive>
</material>
```

### 3. Sensor Noise Models

Unlike URDF, SDF supports realistic sensor noise:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>640</samples>
        <resolution>1</resolution>
        <min_angle>-1.57</min_angle>
        <max_angle>1.57</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </ray>
  <update_rate>20</update_rate>
</sensor>
```

---

## Plugin System

Plugins add custom behavior to models. Common use cases:

### ROS 2 Integration Plugin

```xml
<plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
  <robot_param>robot_description</robot_param>
  <robot_param_node>robot_state_publisher</robot_param_node>
</plugin>
```

### Differential Drive Controller

```xml
<plugin name="differential_drive" filename="libgazebo_ros_diff_drive.so">
  <ros>
    <namespace>/demo</namespace>
  </ros>
  <update_rate>50</update_rate>
  <left_joint>left_wheel_joint</left_joint>
  <right_joint>right_wheel_joint</right_joint>
  <wheel_separation>0.4</wheel_separation>
  <wheel_diameter>0.3</wheel_diameter>
  <max_wheel_torque>20</max_wheel_torque>
  <command_topic>cmd_vel</command_topic>
  <odometry_topic>odom</odometry_topic>
</plugin>
```

---

## Including Models

SDF supports modular model composition:

```xml
<world name="my_world">

  <!-- Include from Gazebo model database -->
  <include>
    <uri>model://sun</uri>
  </include>

  <!-- Include local model -->
  <include>
    <uri>model://my_custom_robot</uri>
    <pose>1 2 0 0 0 1.57</pose>
    <name>robot_1</name>
  </include>

  <!-- Include same model multiple times -->
  <include>
    <uri>model://my_custom_robot</uri>
    <pose>5 2 0 0 0 0</pose>
    <name>robot_2</name>
  </include>

</world>
```

**Model Path**: Gazebo searches `~/.gazebo/models/` and `GAZEBO_MODEL_PATH` environment variable.

---

## Closed Kinematic Loops

SDF supports parallel linkages (impossible in URDF):

```xml
<joint name="loop_joint" type="revolute">
  <parent>link_a</parent>
  <child>link_b</child>
  <axis>
    <xyz>0 0 1</xyz>
  </axis>
</joint>

<joint name="parallel_joint" type="revolute">
  <parent>link_a</parent>
  <child>link_b</child>
  <axis>
    <xyz>0 1 0</xyz>
  </axis>
</joint>
```

This creates a 4-bar linkage mechanism, common in robotic grippers and walking robots.

---

## Converting URDF to SDF

Gazebo automatically converts URDF to SDF when loading. You can also manually convert:

```bash
gz sdf -p my_robot.urdf > my_robot.sdf
```

**Note**: Conversion loses some URDF-specific features (XACRO macros) but gains SDF features (plugins, noise models).

---

## Next Steps

Now that you understand both URDF and SDF, let's compare them directly to know when to use each format.

---

{/*
  TODO: Add PersonalizeButton component when implemented
  <PersonalizeButton chapterId="chapter-2" contentId="sdf-basics" />
*/}

{/*
  TODO: Add TranslateButton component when implemented
  <TranslateButton chapterId="chapter-2" contentId="sdf-basics" targetLanguage="ur" />
*/}

**Continue to**: [URDF vs SDF](./urdf-vs-sdf.md) →
