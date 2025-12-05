# URDF Basics - Unified Robot Description Format

URDF (Unified Robot Description Format) is an XML-based format for describing robot structures. It defines your robot's physical geometry, joints, sensors, and visual appearance.

---

## Why URDF?

Before URDF, every robotics framework had its own robot description format. URDF became the standard for ROS because it provides:

- **Human-readable XML**: Easy to write and understand
- **ROS Integration**: Native support in ROS/ROS 2 tools (RViz, robot_state_publisher)
- **Modularity**: Reusable robot parts via XACRO macros
- **Visualization**: Automatic rendering in RViz and Gazebo

---

## URDF Structure

Every URDF file has a root `<robot>` element containing:

1. **Links**: Physical parts of the robot (chassis, wheels, arms)
2. **Joints**: Connections between links (revolute, prismatic, fixed)
3. **Sensors**: Optional sensor definitions (cameras, LiDAR)

**Basic Structure:**
```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Links define robot parts -->
  <link name="base_link">
    <!-- Visual, collision, inertial properties -->
  </link>

  <!-- Joints connect links -->
  <joint name="wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="wheel_link"/>
    <!-- Joint properties -->
  </joint>

  <link name="wheel_link">
    <!-- Wheel properties -->
  </link>

</robot>
```

---

## Links: Robot Parts

A **link** represents a rigid body in your robot. Each link can have three properties:

### 1. Visual (How it looks)
```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <box size="0.6 0.4 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1.0"/>
    </material>
  </visual>
</link>
```

**Geometry Options:**
- `<box size="x y z"/>` - Rectangular box
- `<cylinder radius="r" length="l"/>` - Cylinder
- `<sphere radius="r"/>` - Sphere
- `<mesh filename="model.dae"/>` - Custom 3D model

### 2. Collision (For physics)
```xml
<collision>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <geometry>
    <box size="0.6 0.4 0.2"/>
  </geometry>
</collision>
```

**Best Practice**: Use simplified collision geometry (boxes/cylinders) instead of complex meshes for better physics performance.

### 3. Inertial (Mass and inertia)
```xml
<inertial>
  <mass value="5.0"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <inertia ixx="0.1" ixy="0.0" ixz="0.0"
           iyy="0.1" iyz="0.0" izz="0.1"/>
</inertial>
```

**Inertia Tensor**: Describes how mass is distributed. Use online calculators for standard shapes.

---

## Joints: Connecting Links

Joints define how links move relative to each other. There are 6 joint types:

### Joint Types

| Type | Description | Example Use |
|------|-------------|-------------|
| **revolute** | Rotates around axis (with limits) | Robot arm elbow |
| **continuous** | Rotates around axis (no limits) | Wheel axle |
| **prismatic** | Slides along axis | Linear actuator |
| **fixed** | No movement | Sensor mount |
| **floating** | 6-DOF free movement | Drone in air |
| **planar** | Moves in a plane | Air hockey puck |

### Example: Continuous Joint (Wheel)
```xml
<joint name="left_wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <origin xyz="-0.15 0.2 0" rpy="-1.5707 0 0"/>
  <axis xyz="0 0 1"/>
</joint>
```

**Key Elements:**
- `<parent>`: Base link
- `<child>`: Moving link
- `<origin>`: Position/orientation of child relative to parent
- `<axis>`: Rotation/translation axis (unit vector)

### Example: Revolute Joint (Arm)
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="base_link"/>
  <child link="upper_arm"/>
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>
```

**Limit Parameters:**
- `lower/upper`: Joint angle limits (radians)
- `effort`: Max torque (N⋅m)
- `velocity`: Max angular velocity (rad/s)

---

## Complete Example: Simple Two-Wheeled Robot

```xml
<?xml version="1.0"?>
<robot name="simple_robot">

  <!-- Base chassis -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.6 0.4 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.4" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.8"/>
    </inertial>
  </link>

  <!-- Left wheel -->
  <link name="left_wheel">
    <visual>
      <geometry>
        <cylinder radius="0.15" length="0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.15" length="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joint connecting left wheel to chassis -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="-0.15 0.225 0" rpy="-1.5707 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>

  <!-- Right wheel (similar structure) -->
  <!-- ... -->

</robot>
```

---

## Visualizing URDF in RViz

To see your URDF in action:

```bash
# Launch RViz with joint state publisher
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

**What you'll see:**
- 3D model of your robot
- Joint sliders to test joint movements
- TF frames showing coordinate systems

---

## Common URDF Mistakes

1. **Missing Collision Geometry**: Robot falls through ground in Gazebo
2. **Zero Mass Links**: Physics engine errors
3. **Incorrect Inertia**: Robot wobbles unrealistically
4. **Wrong Joint Axis**: Joints rotate in unexpected directions
5. **Parent/Child Order**: Kinematic chain breaks

**Debugging Tip**: Use `check_urdf` command:
```bash
check_urdf my_robot.urdf
```

---

## Next Steps

Now that you understand URDF basics, learn about **SDF** (Simulation Description Format), which offers more advanced features for Gazebo simulations.

---

{/*
  TODO: Add PersonalizeButton component when implemented
  <PersonalizeButton chapterId="chapter-2" contentId="urdf-basics" />
*/}

{/*
  TODO: Add TranslateButton component when implemented
  <TranslateButton chapterId="chapter-2" contentId="urdf-basics" targetLanguage="ur" />
*/}

**Continue to**: [SDF Basics](./sdf-basics.md) →
