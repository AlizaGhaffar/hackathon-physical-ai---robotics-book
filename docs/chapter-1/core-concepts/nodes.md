---
sidebar_position: 1
---

# Nodes

## The Building Blocks of ROS 2 Systems

**Nodes** are the fundamental computational units in ROS 2. Think of them as individual programs that perform specific tasks and communicate with each other.

## 🎯 What is a Node?

A **Node** is:
- An independent process running in your robot system
- Responsible for a single, modular task
- Capable of communicating with other nodes
- Can be written in Python, C++, or other supported languages

### Analogy

Think of a robot like a company:
```
Company (Robot System)
├── Marketing Department (Camera Node)
├── Engineering Department (Processing Node)
├── Sales Department (Control Node)
└── HR Department (Status Monitor Node)
```

Each department is independent but communicates with others to achieve company goals.

## 🏗️ Node Architecture

### Basic Structure

```
┌─────────────────────────────┐
│         Node Name           │
│  "camera_driver_node"       │
├─────────────────────────────┤
│  Publishers                 │
│  ├─ /image_raw   (publishes)│
│  └─ /camera_info (publishes)│
├─────────────────────────────┤
│  Subscribers                │
│  └─ /cmd_camera  (receives) │
├─────────────────────────────┤
│  Services                   │
│  └─ /calibrate_camera       │
└─────────────────────────────┘
```

## 💻 Creating Your First Node

### Simple Python Node

```python
import rclpy
from rclpy.node import Node

class MyFirstNode(Node):
    def __init__(self):
        # Initialize node with name
        super().__init__('my_first_node')

        # Log startup message
        self.get_logger().info('My first node is running!')

        # Create a timer that calls a function every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Called every second"""
        self.counter += 1
        self.get_logger().info(f'Timer callback {self.counter}')

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node instance
    node = MyFirstNode()

    # Spin (keep node alive and processing)
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Node

```bash
# Make sure ROS 2 is sourced
source /opt/ros/humble/setup.bash

# Run the node
python3 my_first_node.py
```

**Expected Output:**
```
[INFO] [my_first_node]: My first node is running!
[INFO] [my_first_node]: Timer callback 1
[INFO] [my_first_node]: Timer callback 2
[INFO] [my_first_node]: Timer callback 3
```

## 🔍 Node Introspection

ROS 2 provides powerful tools to inspect running nodes:

### List All Nodes

```bash
ros2 node list
```

**Output Example:**
```
/my_first_node
/camera_driver
/robot_controller
```

### Get Node Information

```bash
ros2 node info /my_first_node
```

**Output:**
```
/my_first_node
  Subscribers:
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /rosout: rcl_interfaces/msg/Log
  Service Servers:

  Service Clients:

  Action Servers:

  Action Clients:
```

## 🎨 Node Design Patterns

### 1. Single Responsibility Principle

**Bad Design:**
```
┌─────────────────────────┐
│   "robot_node"          │
│  ├─ Camera capture      │
│  ├─ Image processing    │
│  ├─ Object detection    │
│  ├─ Path planning       │
│  └─ Motor control       │
└─────────────────────────┘
```

**Good Design:**
```
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│ camera_node   │──▶│ vision_node   │──▶│ control_node  │
│ (capture)     │   │ (detect)      │   │ (move)        │
└───────────────┘   └───────────────┘   └───────────────┘
```

**Why?**
- Easier to debug (isolate problems)
- Reusable components
- Test individual nodes
- Scale performance (run on different machines)

### 2. Lifecycle Nodes (Advanced)

For production robots, use **Lifecycle Nodes** for controlled startup/shutdown:

```
┌──────┐  configure  ┌──────┐  activate  ┌────────┐
│ Init │─────────────▶│Config│───────────▶│ Active │
└──────┘              └──────┘             └────────┘
                         │                      │
                         │ cleanup         deactivate
                         ▼                      ▼
                     ┌──────┐              ┌──────┐
                     │ End  │◀─────────────│Inactive│
                     └──────┘   shutdown   └──────┘
```

## 🚗 Real-World Example: Self-Driving Car

A simplified autonomous vehicle might have these nodes:

```
┌─────────────┐
│ camera_node │──┐
└─────────────┘  │
                 ▼
┌─────────────┐  ┌──────────────┐  ┌─────────────┐
│ lidar_node  │─▶│ fusion_node  │─▶│ planner_node│
└─────────────┘  └──────────────┘  └─────────────┘
                                          │
┌─────────────┐                           ▼
│  gps_node   │───────────────────▶┌─────────────┐
└─────────────┘                    │control_node │
                                   └─────────────┘
                                          │
                                          ▼
                                   ┌─────────────┐
                                   │ motor_driver│
                                   └─────────────┘
```

**Responsibilities:**
- `camera_node`: Capture images
- `lidar_node`: Capture 3D point clouds
- `gps_node`: Get location data
- `fusion_node`: Combine sensor data
- `planner_node`: Decide where to go
- `control_node`: Compute motor commands
- `motor_driver`: Send signals to hardware

## 🧪 Testing Nodes

### Unit Test Example

```python
import unittest
import rclpy
from my_package.my_first_node import MyFirstNode

class TestMyFirstNode(unittest.TestCase):
    def test_node_creation(self):
        rclpy.init()
        node = MyFirstNode()

        # Test that node name is correct
        self.assertEqual(node.get_name(), 'my_first_node')

        # Test that counter starts at 0
        self.assertEqual(node.counter, 0)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
```

## 📊 Node Performance Tips

### 1. Use Timers for Periodic Tasks

```python
# Good: Timer-based
self.timer = self.create_timer(0.1, self.control_loop)

# Bad: Blocking loop
while True:
    self.control_loop()
    time.sleep(0.1)
```

### 2. Avoid Blocking Operations

```python
# Good: Non-blocking
self.create_subscription(Image, '/camera', self.callback, 10)

# Bad: Blocking read
while True:
    image = camera.read()  # Blocks execution
```

### 3. Use Parameters for Configuration

```python
class ConfigurableNode(Node):
    def __init__(self):
        super().__init__('configurable_node')

        # Declare parameter with default
        self.declare_parameter('update_rate', 10.0)

        # Get parameter value
        rate = self.get_parameter('update_rate').value
        self.timer = self.create_timer(1.0/rate, self.callback)
```

## 🎯 Key Takeaways

- 🤖 Nodes are **independent processes** that perform specific tasks
- 🔌 Nodes communicate via **Topics, Services, and Actions**
- 📦 Follow **Single Responsibility Principle** - one task per node
- 🔍 Use `ros2 node list` and `ros2 node info` to inspect nodes
- ⚡ Use **timers** instead of blocking loops
- 🎛️ Use **parameters** for runtime configuration

---

## Practice Exercise

**Challenge:** Create a node that:
1. Prints "Hello ROS 2!" every 2 seconds
2. Has a counter that increments with each message
3. Uses the node name "hello_timer_node"

**Hint:** Modify the `MyFirstNode` example above!

---

## What's Next?

Now that you understand nodes, let's learn how they **communicate** with each other using **Topics**!

Click **"Topics"** in the sidebar to continue. 🚀
