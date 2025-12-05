# Physics Engines in Gazebo

Gazebo supports multiple physics engines, each with different strengths. Choosing the right engine for your simulation can dramatically affect accuracy, performance, and stability.

---

## Available Physics Engines

Gazebo supports three physics engines:

| Engine | Strengths | Weaknesses | Best For |
|--------|-----------|------------|----------|
| **ODE** | Balanced, stable, default | Moderate accuracy | General robotics |
| **Bullet** | Fast collision detection | Less accurate contacts | Fast-moving robots, games |
| **Simbody** | High accuracy, biomechanics | Slower, complex setup | Research, humanoids |

---

## ODE (Open Dynamics Engine)

**Default engine** in Gazebo. Best all-around choice for most robotics applications.

### Configuration

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

### Key Parameters

**Solver Settings:**
- `type`: `quick` (fast) or `world` (accurate)
- `iters`: Solver iterations (50-100 typical)
- `sor`: Successive Over-Relaxation factor (1.0-1.3)

**Constraint Parameters:**
- `cfm`: Constraint Force Mixing (0 = rigid, 0.001 = soft)
- `erp`: Error Reduction Parameter (0.2 typical, higher = stiffer)

---

## Bullet Physics

**Fast and stable**, especially for collision-heavy scenarios.

### Configuration

```xml
<physics type="bullet">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>

  <bullet>
    <solver>
      <type>sequential_impulse</type>
      <iters>50</iters>
      <sor>1.3</sor>
    </solver>
    <constraints>
      <cfm>0.0</cfm>
      <erp>0.2</erp>
      <contact_surface_layer>0.001</contact_surface_layer>
      <split_impulse>true</split_impulse>
      <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
    </constraints>
  </bullet>
</physics>
```

**Unique Features:**
- `split_impulse`: Separate position/velocity corrections (more stable)
- Better handling of stacked objects (e.g., towers of blocks)

---

## Simbody

**Research-grade accuracy** for complex biomechanical systems.

### Configuration

```xml
<physics type="simbody">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>

  <simbody>
    <accuracy>0.001</accuracy>
    <max_transient_velocity>0.01</max_transient_velocity>
    <contact>
      <stiffness>1e8</stiffness>
      <dissipation>10</dissipation>
      <static_friction>0.9</static_friction>
      <dynamic_friction>0.9</dynamic_friction>
      <viscous_friction>0.0</viscous_friction>
    </contact>
  </simbody>
</physics>
```

**Use Cases:**
- Humanoid robots with many degrees of freedom
- Biomechanics research
- Soft robotics with compliant joints

---

## Performance Comparison

### Benchmark: 4-Wheeled Robot (1000 steps)

| Engine | Step Time (ms) | Real-Time Factor | Accuracy (RMS error) |
|--------|----------------|------------------|----------------------|
| ODE (quick) | 1.0 | 1.0x | 0.01m |
| ODE (world) | 2.5 | 0.4x | 0.003m |
| Bullet | 0.8 | 1.2x | 0.02m |
| Simbody | 4.0 | 0.25x | 0.001m |

**Legend:**
- **Step Time**: CPU time per simulation step (lower = faster)
- **Real-Time Factor**: Simulation speed vs real time (1.0 = realtime)
- **Accuracy**: Position error after 1000 steps

---

## Choosing the Right Engine

### Use ODE When:
✅ Building general-purpose robots (mobile robots, arms)
✅ Need balanced performance and accuracy
✅ Working with standard Gazebo tutorials (most use ODE)
✅ Unsure which to choose (safest default)

### Use Bullet When:
✅ Simulating fast-moving objects (racing robots, drones)
✅ Many collision events (robots navigating cluttered environments)
✅ Need real-time performance on slower hardware
✅ Stacking/grasping scenarios with contact-heavy interactions

### Use Simbody When:
✅ Research requiring high accuracy
✅ Humanoid robots with 20+ joints
✅ Biomechanics or medical robotics
✅ Soft robotics with compliant actuators
✅ Performance is not critical (can run slower than realtime)

---

## Switching Physics Engines

### Method 1: SDF World File

```xml
<world name="my_world">
  <physics type="bullet">  <!-- Change this -->
    <max_step_size>0.001</max_step_size>
    <real_time_factor>1.0</real_time_factor>
  </physics>

  <!-- Rest of world -->
</world>
```

### Method 2: Command Line

```bash
# Launch with ODE (default)
gazebo my_world.world

# Launch with Bullet
gazebo my_world.world --physics bullet

# Launch with Simbody
gazebo my_world.world --physics simbody
```

### Method 3: Runtime (GUI)

1. Open Gazebo GUI
2. Go to **World** → **Physics**
3. Select engine from dropdown
4. Click **Apply** (requires world reset)

---

## Common Physics Parameters

### Time Step (`max_step_size`)

```xml
<max_step_size>0.001</max_step_size>  <!-- 1ms, typical -->
```

**Smaller = more accurate, slower:**
- `0.01` (10ms): Fast, unstable for fast robots
- `0.001` (1ms): **Recommended default**
- `0.0001` (0.1ms): High accuracy, 10x slower

### Real-Time Factor (`real_time_factor`)

```xml
<real_time_factor>1.0</real_time_factor>
```

**Target simulation speed:**
- `1.0`: Realtime (matches wall clock)
- `2.0`: 2x faster than realtime
- `0.5`: Half speed (more accurate)
- `0.0`: Run as fast as possible

---

## Troubleshooting Physics Issues

### Problem: Robot Falls Through Ground

**Symptoms**: Robot model drops infinitely, no collision
**Causes**:
- Missing `<collision>` geometry
- Zero or negative mass
- `<static>false</static>` on ground plane

**Fix:**
```xml
<!-- Ensure ground plane is static -->
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

### Problem: Robot Vibrates/Jitters

**Symptoms**: Model shakes even when stationary
**Causes**:
- Time step too large
- Insufficient solver iterations
- High `erp` value

**Fix:**
```xml
<ode>
  <solver>
    <iters>100</iters>  <!-- Increase from 50 -->
  </solver>
  <constraints>
    <erp>0.1</erp>  <!-- Decrease from 0.2 -->
  </constraints>
</ode>
```

### Problem: Simulation Runs Slower Than Realtime

**Symptoms**: `real_time_factor < 1.0` in Gazebo stats
**Causes**:
- Complex collision meshes
- Too many contacts
- Small time step

**Fix:**
1. **Simplify collision geometry** (use boxes/cylinders instead of meshes)
2. **Increase time step** to 0.002 or 0.005
3. **Switch to Bullet** for better performance
4. **Reduce sensor update rates**

---

## Next Steps

Now that you understand physics engines, learn how to configure specific physics parameters like **gravity, friction, and collisions**.

---

{/*
  TODO: Add PersonalizeButton component when implemented
  <PersonalizeButton chapterId="chapter-2" contentId="physics-engines" />
*/}

{/*
  TODO: Add TranslateButton component when implemented
  <TranslateButton chapterId="chapter-2" contentId="physics-engines" targetLanguage="ur" />
*/}

**Continue to**: [Gravity and Friction](./gravity-friction.md) →
