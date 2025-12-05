# What is Gazebo?

Gazebo is a powerful, open-source 3D robot simulator that has become the industry standard for robotics development. It allows you to test robots in realistic virtual environments before deploying them in the real world.

---

## Why Simulation Matters

Imagine building a robot that costs thousands of dollars, only to discover your navigation algorithm crashes it into walls. Or spending weeks debugging hardware issues that could have been caught in minutes with a simulator. This is where Gazebo shines.

**Key Benefits of Robot Simulation:**

1. **Risk-Free Testing**: Experiment with dangerous scenarios (high speeds, collisions, edge cases) without risking expensive hardware
2. **Rapid Iteration**: Test algorithm changes in seconds instead of hours of hardware setup
3. **Cost Savings**: Develop and validate designs before building physical prototypes
4. **Reproducibility**: Run identical test scenarios thousands of times for statistical validation
5. **Scale**: Simulate multiple robots, complex environments, or scenarios impossible to replicate physically

---

## Gazebo Architecture

Gazebo is built on a modular architecture with several key components:

### 1. **Physics Engine**
Simulates real-world physics including gravity, friction, collisions, and forces. Gazebo supports multiple physics engines:
- **ODE (Open Dynamics Engine)**: Default, balanced performance
- **Bullet**: Fast, good for collision-heavy scenarios
- **Simbody**: Accurate, ideal for biomechanics and complex kinematics

### 2. **Rendering Engine**
Provides 3D visualization using OGRE (Object-Oriented Graphics Rendering Engine). Renders realistic lighting, shadows, and textures for sensor simulation.

### 3. **Sensor Models**
Simulates real-world sensors with realistic noise and limitations:
- Cameras (RGB, depth, stereo)
- LiDAR (2D/3D laser scanners)
- IMU (Inertial Measurement Units)
- GPS, Sonar, Force/Torque sensors

### 4. **Plugin System**
Extensible architecture allowing custom behaviors, sensors, and control logic through C++ or Python plugins.

---

## Gazebo + ROS 2 Integration

Gazebo integrates seamlessly with ROS 2 through the **gazebo_ros_pkgs** bridge. This integration allows:

- **Topic Communication**: Gazebo sensors publish to ROS 2 topics (e.g., `/camera/image`, `/scan`)
- **Service Calls**: Control simulation state (spawn models, reset world) via ROS 2 services
- **TF Broadcasting**: Gazebo publishes robot transforms to the ROS 2 TF tree
- **Plugin Compatibility**: Use ROS 2 nodes directly as Gazebo plugins

**Example Workflow:**
```
Gazebo Simulation → Sensor Data → ROS 2 Topics → Your Robot Code → Control Commands → Gazebo Actuators
```

This creates a **digital twin** - a virtual replica of your robot that behaves identically to the physical version.

---

## Installing Gazebo

### Ubuntu (Recommended for ROS 2)

Gazebo comes bundled with ROS 2 Desktop installation. If you installed ROS 2 from Chapter 1, you already have Gazebo!

**Verify Installation:**
```bash
gazebo --version
```

**If not installed, install Gazebo Fortress (compatible with ROS 2 Humble):**
```bash
sudo apt update
sudo apt install gazebo
```

**Launch Gazebo:**
```bash
gazebo
```

You should see the Gazebo GUI with an empty world and a ground plane.

---

### Windows (WSL2 or Native)

**Option 1: WSL2 (Recommended)**
Follow the Ubuntu instructions above inside your WSL2 Ubuntu environment. Use **WSLg** for GUI support (built into Windows 11).

**Option 2: Native Windows Build**
Download the Windows installer from [gazebosim.org](https://gazebosim.org/). Note: ROS 2 integration is better supported on Linux/WSL2.

---

### macOS

Gazebo support on macOS is limited. Use Docker or a Linux virtual machine for the best experience.

---

## Gazebo Versions

**Important**: Gazebo has two major versions:

| Version | Status | ROS 2 Compatibility | Notes |
|---------|--------|---------------------|-------|
| **Gazebo Classic (11.x)** | Legacy | ROS 2 Humble | Older, stable, widely documented |
| **Gazebo Sim (Ignition)** | Current | ROS 2 Humble+ | Modern rewrite, better performance |

**For this textbook, we use Gazebo Classic 11** for maximum compatibility with ROS 2 Humble and existing tutorials.

---

## Your First Gazebo World

Let's verify your installation by launching a simple world:

```bash
# Launch Gazebo with an empty world
gazebo

# Launch Gazebo with a pre-built world (café environment)
gazebo worlds/cafe.world
```

**What you should see:**
- 3D viewport showing the simulated environment
- Timeline controls (play/pause/step)
- Model tree (left panel)
- Insert tab (add models to world)

---

## Next Steps

Now that you understand what Gazebo is and have it installed, let's learn how to describe robots using **URDF** and **SDF** formats in the next sections.

---

{/*
  TODO: Add PersonalizeButton component when implemented
  <PersonalizeButton chapterId="chapter-2" contentId="what-is-gazebo" />
*/}

{/*
  TODO: Add TranslateButton component when implemented
  <TranslateButton chapterId="chapter-2" contentId="what-is-gazebo" targetLanguage="ur" />
*/}

**Continue to**: [URDF Basics](./urdf-sdf/urdf-basics.md) →
