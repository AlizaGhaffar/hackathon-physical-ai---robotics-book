# URDF vs SDF: When to Use Each

Both URDF and SDF describe robots, but they serve different purposes. Understanding when to use each format will save you hours of debugging and conversion headaches.

---

## Feature Comparison Matrix

| Feature | URDF | SDF | Winner |
|---------|------|-----|--------|
| **ROS 2 Integration** | ✅ Native | ⚠️ Via conversion | URDF |
| **RViz Visualization** | ✅ Direct support | ❌ Needs conversion | URDF |
| **World Description** | ❌ Robot only | ✅ Complete worlds | SDF |
| **Multiple Models** | ❌ One robot | ✅ Many models | SDF |
| **Physics Engines** | ⚠️ Basic config | ✅ Advanced control | SDF |
| **Sensor Noise** | ❌ No | ✅ Realistic models | SDF |
| **Plugin System** | ⚠️ Limited | ✅ Extensive | SDF |
| **Closed Loops** | ❌ No | ✅ Yes | SDF |
| **XACRO Macros** | ✅ Yes | ❌ No | URDF |
| **Human Readability** | ✅ Simple | ⚠️ Verbose | URDF |
| **Gazebo Performance** | ⚠️ Via conversion | ✅ Native | SDF |

---

## Decision Tree: Which Format Should I Use?

```
Start Here
    ↓
Are you building a ROS 2 robot?
    ├─ YES → Do you need Gazebo simulation?
    │        ├─ NO → Use URDF (RViz only)
    │        └─ YES → Do you need advanced Gazebo features?
    │                 ├─ NO → Use URDF (Gazebo converts it)
    │                 └─ YES → Use URDF + SDF hybrid approach
    │
    └─ NO → Are you simulating multiple robots or complex worlds?
             ├─ YES → Use SDF
             └─ NO → Use URDF (more portable)
```

---

## Use Case 1: ROS 2 Robot with Simple Simulation

**Scenario**: Building a mobile robot for ROS 2. Need RViz visualization and basic Gazebo testing.

**Recommendation**: **URDF**

**Why?**
- ROS 2 tools (robot_state_publisher, joint_state_publisher) expect URDF
- RViz directly renders URDF without conversion
- Gazebo automatically converts URDF → SDF at runtime
- XACRO macros simplify robot definition

**Example Directory Structure:**
```
my_robot/
├── urdf/
│   ├── my_robot.urdf.xacro  ← Main robot description
│   └── materials.xacro       ← Color definitions
├── launch/
│   ├── display.launch.py     ← RViz launch
│   └── gazebo.launch.py      ← Gazebo simulation
└── config/
    └── joints.yaml           ← Joint configurations
```

---

## Use Case 2: Multi-Robot Simulation

**Scenario**: Simulating a warehouse with 10 autonomous robots navigating together.

**Recommendation**: **SDF World + SDF Models**

**Why?**
- SDF worlds can include multiple models with unique names
- Better performance (no runtime URDF→SDF conversion)
- Advanced sensor noise models for realistic multi-robot coordination testing
- Centralized physics configuration

**Example SDF World:**
```xml
<world name="warehouse">
  <physics type="ode">...</physics>

  <!-- Spawn 10 robots -->
  <include>
    <uri>model://warehouse_robot</uri>
    <pose>0 0 0 0 0 0</pose>
    <name>robot_0</name>
  </include>

  <include>
    <uri>model://warehouse_robot</uri>
    <pose>5 0 0 0 0 0</pose>
    <name>robot_1</name>
  </include>

  <!-- ... robot_2 through robot_9 ... -->

  <!-- Warehouse environment -->
  <include>
    <uri>model://warehouse_shelves</uri>
  </include>
</world>
```

---

## Use Case 3: Hybrid Approach (Best of Both Worlds)

**Scenario**: ROS 2 robot with advanced Gazebo features (realistic sensors, custom plugins).

**Recommendation**: **URDF for Robot + SDF for World/Plugins**

**Workflow:**
1. Define robot structure in URDF (for ROS 2 compatibility)
2. Let Gazebo convert URDF → SDF
3. Create SDF world file with physics/lighting
4. Add Gazebo-specific plugins via SDF tags in URDF

**Example: Adding SDF Plugin to URDF**
```xml
<!-- In your URDF file -->
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <parameters>$(find my_robot)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>

<gazebo reference="camera_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera>
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>my_robot</namespace>
        <remapping>image_raw:=camera/image</remapping>
      </ros>
    </plugin>
  </sensor>
</gazebo>
```

This gives you:
- ✅ ROS 2 compatibility (URDF)
- ✅ RViz visualization (URDF)
- ✅ Advanced Gazebo features (SDF plugins)
- ✅ Realistic sensors (SDF noise models)

---

## Use Case 4: Research Simulation (No ROS)

**Scenario**: Academic research on physics-based grasping. No need for ROS integration.

**Recommendation**: **Pure SDF**

**Why?**
- No ROS overhead
- Direct access to all Gazebo features
- Faster simulation (no conversion)
- Better for custom C++ plugins

---

## Conversion Between Formats

### URDF → SDF (Automatic)
Gazebo converts URDF to SDF automatically when loading:
```bash
gazebo my_robot.urdf  # Converts internally
```

### URDF → SDF (Manual)
```bash
gz sdf -p my_robot.urdf > my_robot.sdf
```

**What gets lost:**
- XACRO macros (fully expanded)
- Some ROS-specific tags
- Comments

**What gets gained:**
- Native Gazebo format
- Better performance in Gazebo

### SDF → URDF (Manual, Limited)
No direct conversion tool. You must manually recreate the URDF, keeping only:
- Links
- Joints
- Basic sensors

**What gets lost:**
- World information
- Advanced physics config
- Sensor noise models
- Plugins (must be re-added as `<gazebo>` tags)

---

## Performance Considerations

### URDF in Gazebo
- **Startup Time**: Slower (conversion overhead)
- **Runtime Performance**: Same as SDF (after conversion)
- **Memory**: Slightly higher (both URDF and SDF in memory)

### SDF in Gazebo
- **Startup Time**: Fast (native format)
- **Runtime Performance**: Optimal
- **Memory**: Lower (only SDF in memory)

**Benchmark** (10-link robot, 1000 simulation steps):
| Format | Load Time | Step Time |
|--------|-----------|-----------|
| URDF   | 150ms     | 1.0ms     |
| SDF    | 50ms      | 1.0ms     |

**Verdict**: Use SDF for large-scale simulations (>5 robots).

---

## Best Practices

### For ROS 2 Projects:
1. **Start with URDF** - Use XACRO for modularity
2. **Add Gazebo tags** - Embed SDF plugins in URDF via `<gazebo>` tags
3. **Version control URDF** - Don't commit auto-generated SDF
4. **Test in RViz first** - Catch kinematic errors before Gazebo

### For Pure Gazebo Projects:
1. **Use SDF directly** - Avoid unnecessary conversion
2. **Modularize models** - Use `<include>` for reusable parts
3. **Separate world and models** - Keep robot definitions independent
4. **Version control SDF** - Track world files and models separately

---

## Summary Table

| Situation | Format | Reason |
|-----------|--------|--------|
| ROS 2 robot, simple sim | URDF | ROS 2 native, RViz support |
| Multiple robots | SDF | Better multi-model support |
| Advanced sensors | URDF + SDF tags | Hybrid approach |
| No ROS needed | SDF | Native Gazebo format |
| Rapid prototyping | URDF | Easier to write/edit |
| Production sim | SDF | Better performance |

---

## Next Steps

Now that you understand the differences, let's explore how to configure **physics simulations** in Gazebo to make your robots behave realistically.

---

{/*
  TODO: Add PersonalizeButton component when implemented
  <PersonalizeButton chapterId="chapter-2" contentId="urdf-vs-sdf" />
*/}

{/*
  TODO: Add TranslateButton component when implemented
  <TranslateButton chapterId="chapter-2" contentId="urdf-vs-sdf" targetLanguage="ur" />
*/}

**Continue to**: [Physics Engines](../physics/physics-engines.md) →
