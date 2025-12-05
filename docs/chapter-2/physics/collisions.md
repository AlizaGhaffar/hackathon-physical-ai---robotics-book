---
sidebar_position: 3
---

# Collision Detection and Contact Forces

Collision detection is fundamental to realistic robot simulation. It determines when objects touch, how they interact, and what forces are generated during contact.

---

## What is Collision Detection?

Collision detection in Gazebo has two phases:

1. **Broad Phase**: Quickly identifies potential collisions using bounding boxes (AABB - Axis-Aligned Bounding Boxes)
2. **Narrow Phase**: Precise collision detection using actual geometry shapes

**Performance Tip**: Use simple collision shapes (boxes, cylinders, spheres) even if visual geometry is complex.

---

## Collision Geometry vs Visual Geometry

Every link in a robot has **two** separate geometries:

### Visual Geometry
- What you **see** in the simulation
- Can be complex meshes (.stl, .dae files)
- Does NOT affect physics

### Collision Geometry
- What the **physics engine** uses
- Should be simple shapes for performance
- Affects physics calculations

```xml
<link name="body">
  <!-- High-detail visual (complex mesh) -->
  <visual name="visual">
    <geometry>
      <mesh filename="model://robot/meshes/body.dae"/>
    </geometry>
  </visual>

  <!-- Simple collision shape (box approximation) -->
  <collision name="collision">
    <geometry>
      <box size="0.5 0.3 0.2"/>
    </geometry>
  </collision>
</link>
```

**Why separate?**
- Complex meshes slow down physics (expensive collision checks)
- Simple shapes provide 80% accuracy at 10x speed

---

## Collision Shapes

### 1. Box
```xml
<collision name="box_collision">
  <geometry>
    <box size="1.0 0.5 0.3"/>  <!-- width depth height in meters -->
  </geometry>
</collision>
```
**Best for**: Chassis, containers, walls

### 2. Cylinder
```xml
<collision name="cylinder_collision">
  <geometry>
    <cylinder radius="0.1" length="0.5"/>  <!-- radius and length -->
  </geometry>
</collision>
```
**Best for**: Wheels, pipes, poles

### 3. Sphere
```xml
<collision name="sphere_collision">
  <geometry>
    <sphere radius="0.15"/>
  </geometry>
</collision>
```
**Best for**: Balls, caster wheels, simple heads

### 4. Mesh (Complex)
```xml
<collision name="mesh_collision">
  <geometry>
    <mesh filename="model://my_robot/meshes/body_collision.stl"/>
  </geometry>
</collision>
```
**Best for**: Irregular shapes where accuracy matters
**Warning**: Use sparingly - can slow simulation significantly

### 5. Compound Shapes
Combine multiple simple shapes for better approximation:

```xml
<link name="robot_arm">
  <!-- Upper arm segment -->
  <collision name="upper_segment">
    <pose>0 0 0.15 0 0 0</pose>
    <geometry>
      <cylinder radius="0.05" length="0.3"/>
    </geometry>
  </collision>

  <!-- Elbow joint -->
  <collision name="elbow">
    <pose>0 0 0.3 0 0 0</pose>
    <geometry>
      <sphere radius="0.06"/>
    </geometry>
  </collision>
</link>
```

---

## Collision Properties

### Surface Friction (Review)
```xml
<collision name="collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

### Contact Stiffness
Controls how "hard" or "soft" collisions feel:

```xml
<collision name="soft_padding">
  <surface>
    <contact>
      <ode>
        <kp>1e3</kp>   <!-- Low stiffness (soft) -->
        <kd>10</kd>    <!-- Damping -->
        <soft_cfm>0.1</soft_cfm>
        <soft_erp>0.2</soft_erp>
      </ode>
    </contact>
  </surface>
</collision>
```

**Stiffness Values:**
- `kp = 1e10`: Rigid (steel, concrete)
- `kp = 1e6`: Moderate (wood, plastic)
- `kp = 1e3`: Soft (foam, rubber)

---

## Self-Collision

By default, adjacent links in the same robot **do not** collide with each other (to prevent numerical instability).

### Enable Self-Collision
```xml
<model name="robot">
  <self_collide>true</self_collide>  <!-- Enable for whole model -->

  <link name="link1">...</link>
  <link name="link2">...</link>
</model>
```

**Use Case**: Humanoid robots where arms can collide with torso

### Disable Specific Collisions
```xml
<gazebo>
  <collision>
    <max_contacts>0</max_contacts>  <!-- Disable collisions for this link -->
  </collision>
</gazebo>
```

---

## Contact Forces

When objects collide, Gazebo computes contact forces based on:

1. **Penetration Depth**: How much objects overlap
2. **Contact Normal**: Direction perpendicular to collision surface
3. **Friction Forces**: Tangential forces resisting sliding

### Viewing Contact Forces

Launch Gazebo with contact visualization:
```bash
gazebo --verbose
```

In the GUI:
1. **View** → **Contacts**
2. Collisions show as pink/red contact points

### Reading Contact Data in ROS 2

Use the Bumper Sensor plugin:
```xml
<sensor name="bumper" type="contact">
  <contact>
    <collision>collision_name</collision>
  </contact>
  <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so">
    <ros>
      <namespace>/robot</namespace>
      <remapping>bumper_states:=bumper</remapping>
    </ros>
    <frame_name>world</frame_name>
  </plugin>
</sensor>
```

Subscribe to `/robot/bumper` topic to read collision events.

---

## Collision Filtering (Performance)

### Disable Unnecessary Collisions

For visual-only objects (decorations):
```xml
<link name="decoration">
  <collision name="collision">
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
    <surface>
      <bounce>
        <restitution_coefficient>0</restitution_coefficient>
      </bounce>
      <friction>
        <ode>
          <mu>0</mu>
          <mu2>0</mu2>
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

Or remove collision entirely:
```xml
<link name="marker">
  <visual>...</visual>
  <!-- No collision tag = no physics interaction -->
</link>
```

---

## Advanced: Collision Masks (Bitwise Filtering)

Control which objects can collide using category/mask bits:

```xml
<collision name="robot_collision">
  <surface>
    <contact>
      <collide_bitmask>0x01</collide_bitmask>  <!-- Category: Robot (bit 0) -->
    </contact>
  </surface>
</collision>

<collision name="obstacle_collision">
  <surface>
    <contact>
      <collide_bitmask>0x02</collide_bitmask>  <!-- Category: Obstacle (bit 1) -->
    </contact>
  </surface>
</collision>
```

**Use Case**: Multi-robot simulations where robots should not collide with each other but should collide with obstacles.

---

## Common Collision Problems

### Problem: Robot Falls Through Ground

**Symptoms**: Robot sinks into ground plane
**Cause**: Ground plane has no collision geometry
**Fix**: Ensure ground plane model includes collision:
```xml
<model name="ground_plane">
  <static>true</static>
  <link name="link">
    <collision name="collision">
      <geometry>
        <plane><normal>0 0 1</normal></plane>
      </geometry>
    </collision>
  </link>
</model>
```

### Problem: Robot Jitters/Vibrates

**Symptoms**: Continuous small oscillations when stationary
**Cause**: Insufficient contact damping
**Fix**: Add damping to collision surface:
```xml
<contact>
  <ode>
    <kd>100.0</kd>   <!-- Increase damping -->
    <soft_cfm>0.001</soft_cfm>
  </ode>
</contact>
```

### Problem: Robot Passes Through Walls

**Symptoms**: Robot clips through thin walls at high speed
**Cause**: Collision detection missed (tunneling effect)
**Fix 1**: Increase physics update rate:
```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>  <!-- Smaller timestep -->
</physics>
```

**Fix 2**: Use thicker collision walls
**Fix 3**: Enable Continuous Collision Detection (CCD) if using Bullet physics

---

## Practical Example: Mobile Robot with Bumper

```xml
<model name="bumper_robot">
  <!-- Main body -->
  <link name="chassis">
    <collision name="chassis_collision">
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </collision>
    <visual name="chassis_visual">
      <geometry>
        <box size="0.4 0.3 0.2"/>
      </geometry>
    </visual>
  </link>

  <!-- Front bumper with contact sensor -->
  <link name="bumper">
    <pose>0.21 0 0 0 0 0</pose>  <!-- Slightly in front -->
    <collision name="bumper_collision">
      <geometry>
        <box size="0.02 0.3 0.15"/>  <!-- Thin vertical plate -->
      </geometry>
      <surface>
        <contact>
          <ode>
            <kp>1e6</kp>
            <kd>100</kd>
          </ode>
        </contact>
      </surface>
    </collision>

    <!-- Contact sensor -->
    <sensor name="bumper_sensor" type="contact">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <contact>
        <collision>bumper_collision</collision>
      </contact>
    </sensor>
  </link>

  <!-- Joint connecting bumper to chassis -->
  <joint name="bumper_joint" type="fixed">
    <parent>chassis</parent>
    <child>bumper</child>
  </joint>
</model>
```

---

## Testing Collision Detection

### Visual Inspection
1. Launch Gazebo
2. **View** → **Wireframe** to see collision geometry
3. **View** → **Contacts** to see active collision points
4. Drop objects and observe contact points turn pink/red

### ROS 2 Echo Test
```bash
# Terminal 1: Launch simulation
ros2 launch my_robot simulation.launch.py

# Terminal 2: Monitor contact sensor
ros2 topic echo /robot/bumper
```

Push robot into obstacle and verify contact messages appear.

---

## Key Takeaways

✅ Use simple collision shapes (box, cylinder, sphere) for performance
✅ Collision geometry can differ from visual geometry
✅ Tune contact stiffness (`kp`, `kd`) for realistic behavior
✅ Enable self-collision only when necessary
✅ Use contact sensors to detect collisions in your ROS 2 code
✅ Adjust physics timestep if tunneling occurs

---

## Next Steps

Now that you understand physics simulation, let's learn about **sensor simulation** to give your robot perception capabilities!

**Continue to**: [Cameras](../sensors/cameras.md) →
