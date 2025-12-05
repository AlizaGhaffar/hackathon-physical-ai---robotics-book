---
sidebar_position: 8
---

# URDF Basics: Describing Your Robot

## What is URDF?

**URDF (Unified Robot Description Format)** is an XML-based language for describing the physical structure of a robot. Think of it as a **blueprint** that tells ROS 2:

- What parts (links) your robot has
- How they're connected (joints)
- Physical properties (mass, dimensions, collision boundaries)
- Visual appearance (colors, 3D models)

### Why URDF Matters for Humanoid Robots

For humanoid robots, URDF is critical because:

- 🦿 **Complex Kinematics**: Humanoids have 20-30+ joints (arms, legs, torso, head)
- 🎯 **Motion Planning**: URDF enables inverse kinematics for reaching and grasping
- 👁️ **Simulation**: Test behaviors in Gazebo before deploying to real hardware
- 🤖 **Visualization**: See your robot's state in RViz during development

---

## Core URDF Concepts

### 1. Links (Robot Parts)

A **link** represents a rigid body part of your robot:

```xml
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>

  <collision>
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
  </collision>

  <inertial>
    <mass value="10.0"/>
    <inertia ixx="1.0" ixy="0" ixz="0"
             iyy="1.0" iyz="0" izz="1.0"/>
  </inertial>
</link>
```

**Key Components:**
- `<visual>`: How the link appears (shape, color, 3D mesh)
- `<collision>`: Simplified geometry for collision detection
- `<inertial>`: Mass and inertia for physics simulation

---

### 2. Joints (Connections Between Links)

A **joint** defines how two links move relative to each other:

```xml
<joint name="arm_shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>

  <origin xyz="0.2 0 0.3" rpy="0 0 0"/>

  <axis xyz="0 1 0"/>

  <limit lower="-1.57" upper="1.57"
         effort="100" velocity="1.0"/>
</joint>
```

**Joint Types:**
- **revolute**: Rotates around an axis (like elbow, knee)
- **prismatic**: Slides along an axis (like telescoping antenna)
- **fixed**: No movement (like mounting plate)
- **continuous**: Rotates infinitely (like wheels)
- **floating**: 6 degrees of freedom (rarely used)

---

## Simple Humanoid Arm Example

Let's model a basic robot arm with shoulder, elbow, and wrist:

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.1"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Upper Arm -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <origin xyz="0 0 0.2"/>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  </joint>

  <!-- Forearm -->
  <link name="forearm">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.3"/>
      </geometry>
      <origin xyz="0 0 0.15"/>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
  </link>

  <!-- Elbow Joint -->
  <joint name="elbow_joint" type="revolute">
    <parent link="upper_arm"/>
    <child link="forearm"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="30" velocity="1.0"/>
  </joint>

  <!-- Hand -->
  <link name="hand">
    <visual>
      <geometry>
        <box size="0.08 0.12 0.03"/>
      </geometry>
      <material name="skin">
        <color rgba="0.9 0.7 0.5 1"/>
      </material>
    </visual>
  </link>

  <!-- Wrist Joint -->
  <joint name="wrist_joint" type="revolute">
    <parent link="forearm"/>
    <child link="hand"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.0" upper="1.0" effort="20" velocity="1.0"/>
  </joint>

</robot>
```

---

## Understanding Coordinate Frames

### Origin and Orientation

Every joint has an `<origin>` that defines:

```xml
<origin xyz="0.2 0 0.3" rpy="0 1.57 0"/>
```

- **xyz**: Position offset (X, Y, Z in meters)
- **rpy**: Orientation (Roll, Pitch, Yaw in radians)

**Example**: For a shoulder joint at 20cm to the right, 30cm up:
```xml
<origin xyz="0.2 0 0.3" rpy="0 0 0"/>
```

---

## Visualizing Your URDF

### Using RViz

RViz is ROS 2's 3D visualization tool:

```bash
# Launch RViz with your URDF
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

**What you'll see:**
- 3D model of your robot
- TF frames showing coordinate systems
- Joint sliders to test motion

### Using Gazebo Simulator

Test physics and sensors:

```bash
# Spawn robot in Gazebo
ros2 launch gazebo_ros spawn_entity.py -entity my_robot -file my_robot.urdf
```

---

## Best Practices for Humanoid URDF

### 1. Start Simple
Begin with a stick figure (cylinders for limbs, spheres for joints), then add detail.

### 2. Use Consistent Naming
```
torso
├── left_shoulder_joint → left_upper_arm
│   └── left_elbow_joint → left_forearm
│       └── left_wrist_joint → left_hand
└── right_shoulder_joint → right_upper_arm
    └── right_elbow_joint → right_forearm
        └── right_wrist_joint → right_hand
```

### 3. Define Collision Geometry
Always include `<collision>` tags - simplified shapes improve simulation performance.

### 4. Set Realistic Limits
Joint limits should match real-world constraints:
```xml
<!-- Human knee can't bend backwards -->
<limit lower="0" upper="2.35" effort="100" velocity="2.0"/>
```

### 5. Add Inertial Properties
For realistic physics simulation:
```xml
<inertial>
  <mass value="2.5"/>
  <inertia ixx="0.01" ixy="0" ixz="0"
           iyy="0.01" iyz="0" izz="0.01"/>
</inertial>
```

---

## Common URDF Mistakes

### ❌ Missing Parent Link
```xml
<!-- WRONG: No parent link defined -->
<joint name="elbow" type="revolute">
  <child link="forearm"/>
  ...
</joint>
```

### ✅ Correct Version
```xml
<joint name="elbow" type="revolute">
  <parent link="upper_arm"/>
  <child link="forearm"/>
  ...
</joint>
```

### ❌ Incorrect Axis
```xml
<!-- WRONG: Knee rotating on X axis (sideways) -->
<joint name="knee" type="revolute">
  <axis xyz="1 0 0"/>
  ...
</joint>
```

### ✅ Correct Version
```xml
<!-- Knee should rotate on Y axis (forward/backward) -->
<joint name="knee" type="revolute">
  <axis xyz="0 1 0"/>
  ...
</joint>
```

---

## Tools for URDF Development

### 1. URDF Checker
Validate your URDF syntax:
```bash
check_urdf my_robot.urdf
```

### 2. URDF to Graph
Visualize link-joint structure:
```bash
urdf_to_graphiz my_robot.urdf
```

### 3. Xacro (URDF Macros)
For complex robots, use Xacro to reduce repetition:
```xml
<xacro:macro name="leg" params="prefix">
  <link name="${prefix}_thigh"/>
  <joint name="${prefix}_hip" type="revolute">
    ...
  </joint>
</xacro:macro>

<!-- Use macro twice for left and right legs -->
<xacro:leg prefix="left"/>
<xacro:leg prefix="right"/>
```

---

## Next Steps

Now that you understand URDF:

1. ✅ Model a simple robot (start with 2-3 links)
2. ✅ Visualize it in RViz
3. ✅ Add sensors (cameras, IMU)
4. ✅ Simulate in Gazebo

**Coming up**: We'll use URDF to build a complete humanoid robot model in Chapter 3!

---

## Quick Reference

**Basic URDF Structure:**
```xml
<robot name="my_robot">
  <link name="...">...</link>
  <joint name="..." type="revolute">
    <parent link="..."/>
    <child link="..."/>
    <origin xyz="..." rpy="..."/>
    <axis xyz="..."/>
    <limit lower="..." upper="..."/>
  </joint>
</robot>
```

**Joint Types:** revolute, prismatic, fixed, continuous, floating

**Visualization:**
```bash
ros2 launch urdf_tutorial display.launch.py model:=robot.urdf
```
