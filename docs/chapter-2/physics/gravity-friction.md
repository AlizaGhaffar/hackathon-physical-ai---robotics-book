# Gravity and Friction Configuration

Realistic robot simulation requires accurate modeling of gravity and friction. These parameters determine how your robot interacts with the environment and whether it behaves like its physical counterpart.

---

## Gravity Configuration

### Setting Global Gravity

Gravity is defined per-world in SDF:

```xml
<world name="my_world">
  <gravity>0 0 -9.81</gravity>  <!-- Earth gravity in m/s² -->

  <!-- Rest of world definition -->
</world>
```

**Format**: `<gravity>x y z</gravity>` (vector in m/s²)

### Common Gravity Values

| Environment | X | Y | Z | Use Case |
|-------------|---|---|---|----------|
| **Earth** | 0 | 0 | -9.81 | Standard robotics |
| **Moon** | 0 | 0 | -1.62 | Lunar rovers |
| **Mars** | 0 | 0 | -3.71 | Mars rovers |
| **Microgravity** | 0 | 0 | -0.01 | Space robotics |
| **Zero-G** | 0 | 0 | 0 | ISS/orbital robots |
| **Custom Test** | 0 | 0 | -5.0 | Debugging |

### Non-Standard Gravity Directions

Simulate robots on slopes or rotating platforms:

```xml
<!-- 30-degree slope (gravity tilted) -->
<gravity>4.905 0 -8.495</gravity>

<!-- Calculation: -->
<!-- g_x = -9.81 * sin(30°) = -4.905 -->
<!-- g_z = -9.81 * cos(30°) = -8.495 -->
```

---

## Friction Basics

Friction prevents sliding and enables traction. Gazebo uses two friction models:

### 1. Coulomb Friction (ODE/Bullet)

Defined per-surface using friction coefficients:

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>   <!-- Friction coefficient (direction 1) -->
      <mu2>1.0</mu2> <!-- Friction coefficient (direction 2) -->
      <fdir1>1 0 0</fdir1>  <!-- Direction 1 (optional) -->
      <slip1>0.0</slip1>    <!-- Slip compliance (optional) -->
      <slip2>0.0</slip2>    <!-- Slip compliance (optional) -->
    </ode>
  </friction>
</surface>
```

**Friction Coefficient (μ):**
- `μ = 0.0`: Frictionless (ice)
- `μ = 0.1`: Low friction (wet tile)
- `μ = 1.0`: **Standard default** (dry concrete)
- `μ = 1.5`: High friction (rubber on pavement)
- `μ = ∞` (large value like 100): No slip (welded joint)

### 2. Real Friction Values

| Material Pair | μ (Coefficient) | Example |
|---------------|-----------------|---------|
| Steel on steel | 0.15 - 0.25 | Metal robot chassis |
| Rubber on concrete | 1.0 - 1.5 | Robot wheels |
| Rubber on ice | 0.02 - 0.05 | Slippery surfaces |
| Wood on wood | 0.3 - 0.5 | Wooden robot parts |
| Teflon on steel | 0.04 - 0.1 | Low-friction joints |

---

## Configuring Wheel Friction

### Mobile Robot Wheels

```xml
<link name="left_wheel">
  <collision name="collision">
    <geometry>
      <cylinder radius="0.1" length="0.05"/>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.5</mu>    <!-- Longitudinal friction (rolling direction) -->
          <mu2>0.5</mu2>  <!-- Lateral friction (sideways slip) -->
          <fdir1>0 1 0</fdir1>  <!-- Define longitudinal axis -->
        </ode>
      </friction>
    </surface>
  </collision>
</link>
```

**Why different μ and μ2?**
- `mu` (longitudinal): High friction for traction (prevents wheel slip)
- `mu2` (lateral): Lower friction allows sliding turns

### Omni-Directional Wheels

```xml
<surface>
  <friction>
    <ode>
      <mu>0.1</mu>   <!-- Low friction (passive rollers) -->
      <mu2>0.1</mu2> <!-- Isotropic friction -->
    </ode>
  </friction>
</surface>
```

---

## Advanced Friction: Slip and Bounce

### Slip Parameters

Allow small amounts of sliding even with friction:

```xml
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
      <slip1>0.01</slip1>  <!-- 1% slip in direction 1 -->
      <slip2>0.01</slip2>  <!-- 1% slip in direction 2 -->
    </ode>
  </friction>
</surface>
```

**Use Case**: Simulate tire deformation or conveyor belts.

### Bounce (Restitution)

Control how much energy is retained after collisions:

```xml
<surface>
  <bounce>
    <restitution_coefficient>0.8</restitution_coefficient>
    <threshold>0.1</threshold>  <!-- Min velocity for bounce (m/s) -->
  </bounce>
</surface>
```

**Restitution Coefficient (e):**
- `e = 0.0`: Perfectly inelastic (no bounce, like clay)
- `e = 0.5`: **Typical default** (moderate bounce)
- `e = 0.9`: High bounce (rubber ball)
- `e = 1.0`: Perfectly elastic (ideal physics, unrealistic)

**Example: Basketball**
```xml
<bounce>
  <restitution_coefficient>0.85</restitution_coefficient>
  <threshold>0.01</threshold>
</bounce>
```

---

## Material Combinations

Define reusable material profiles:

```xml
<!-- In world file -->
<surface>
  <friction>
    <ode>
      <mu>1.0</mu>
      <mu2>1.0</mu2>
    </ode>
  </friction>
  <bounce>
    <restitution_coefficient>0.5</restitution_coefficient>
  </bounce>
  <contact>
    <ode>
      <soft_cfm>0.001</soft_cfm>
      <soft_erp>0.2</soft_erp>
      <kp>1e6</kp>
      <kd>100</kd>
    </ode>
  </contact>
</surface>
```

### Contact Parameters

**Soft Contact (ODE):**
- `soft_cfm`: Constraint Force Mixing (higher = softer)
- `soft_erp`: Error Reduction Parameter (higher = stiffer)
- `kp`: Contact stiffness (spring constant)
- `kd`: Contact damping

**Rigid Contact:**
```xml
<contact>
  <ode>
    <soft_cfm>0.0</soft_cfm>
    <soft_erp>0.2</soft_erp>
    <kp>1e10</kp>  <!-- Very high stiffness -->
    <kd>1.0</kd>
  </ode>
</contact>
```

**Soft Contact (e.g., foam pad):**
```xml
<contact>
  <ode>
    <soft_cfm>0.1</soft_cfm>
    <soft_erp>0.1</soft_erp>
    <kp>1e3</kp>  <!-- Low stiffness -->
    <kd>10.0</kd>
  </ode>
</contact>
```

---

## Complete Example: Configurable Robot

```xml
<model name="mobile_robot">

  <!-- Chassis (low friction on bottom) -->
  <link name="base_link">
    <collision name="chassis_collision">
      <geometry>
        <box size="0.5 0.3 0.1"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.3</mu>   <!-- Low friction (avoid dragging) -->
            <mu2>0.3</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

  <!-- Wheels (high traction) -->
  <link name="left_wheel">
    <collision name="wheel_collision">
      <geometry>
        <cylinder radius="0.1" length="0.05"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.5</mu>    <!-- High longitudinal traction -->
            <mu2>0.5</mu2>  <!-- Allow lateral sliding for turns -->
            <fdir1>0 1 0</fdir1>
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.1</restitution_coefficient>
        </bounce>
      </surface>
    </collision>
  </link>

  <!-- Caster wheel (low friction, omnidirectional) -->
  <link name="caster">
    <collision name="caster_collision">
      <geometry>
        <sphere radius="0.03"/>
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.05</mu>   <!-- Very low friction -->
            <mu2>0.05</mu2>
          </ode>
        </friction>
      </surface>
    </collision>
  </link>

</model>
```

---

## Testing Friction

### Slope Test

Measure friction by tilting a plane until robot slides:

```bash
# Formula: μ_static = tan(θ)
# If robot slides at 30°: μ = tan(30°) ≈ 0.577
```

### Acceleration Test

Measure wheel slip during acceleration:
1. Command high velocity
2. Observe wheel angular velocity vs linear velocity
3. If ω_wheel >> v_linear, friction is too low

---

## Common Friction Problems

### Problem: Robot Slides Downhill

**Symptoms**: Robot can't climb slopes, slides backwards
**Fix**: Increase wheel friction `mu` to 1.5 or higher

### Problem: Robot Can't Turn

**Symptoms**: Robot moves straight even with differential drive
**Fix**: Decrease lateral friction `mu2` to 0.3-0.5

### Problem: Robot Vibrates on Ground

**Symptoms**: Continuous jittering when stationary
**Fix**: Add contact damping:
```xml
<contact>
  <ode>
    <kd>100.0</kd>  <!-- Increase damping -->
  </ode>
</contact>
```

---

## Next Steps

Now that you understand gravity and friction, learn about **collision detection** to model complex interactions.

---

{/*
  TODO: Add PersonalizeButton component when implemented
  <PersonalizeButton chapterId="chapter-2" contentId="gravity-friction" />
*/}

{/*
  TODO: Add TranslateButton component when implemented
  <TranslateButton chapterId="chapter-2" contentId="gravity-friction" targetLanguage="ur" />
*/}

**Continue to**: [Collisions](./collisions.md) →
