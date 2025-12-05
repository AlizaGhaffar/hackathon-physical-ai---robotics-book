---
sidebar_position: 1
---

# Example 1: Hello World - Your First ROS 2 Node

## What You'll Learn

- Create a minimal ROS 2 node
- Understand node lifecycle
- Use ROS 2 logging system
- Run and verify your node

---

## The Concept

A **Node** is the fundamental building block in ROS 2. Think of it as a small program that performs one specific task. In a humanoid robot:

- One node might handle camera vision
- Another controls arm movements
- A third processes speech recognition

This example creates the simplest possible node - one that just says "Hello, Humanoid World!" to prove everything works.

---

## The Code

Create a file called `hello_node.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class HelloNode(Node):
    """
    A minimal ROS 2 node that prints a greeting message.

    This demonstrates:
    - Node creation and initialization
    - ROS 2 logging system
    - Basic node lifecycle
    """

    def __init__(self):
        # Initialize the node with a unique name
        super().__init__('hello_humanoid_node')

        # Log an informational message
        self.get_logger().info('Hello, Humanoid World!')
        self.get_logger().info('This node is ready to become part of a robot brain!')

        # Create a timer that runs every 2 seconds
        self.timer = self.create_timer(2.0, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        """Called every 2 seconds by the timer"""
        self.counter += 1
        self.get_logger().info(f'Heartbeat #{self.counter}: Still alive and thinking...')

def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create an instance of our node
    node = HelloNode()

    # Keep the node running and processing callbacks
    # This is like keeping the robot's brain active
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Code Breakdown

### 1. Imports
```python
import rclpy  # ROS 2 Python library
from rclpy.node import Node  # Base class for nodes
```

### 2. Node Class
```python
class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_humanoid_node')
```
Every node needs a **unique name**. This helps identify it when debugging.

### 3. Logging
```python
self.get_logger().info('Hello, Humanoid World!')
```
ROS 2 has 5 log levels:
- `debug()` - Detailed debugging info
- `info()` - General information
- `warn()` - Warning messages
- `error()` - Error messages
- `fatal()` - Critical failures

### 4. Timer
```python
self.timer = self.create_timer(2.0, self.timer_callback)
```
Creates a timer that calls `timer_callback()` every 2 seconds. This is like a heartbeat for the node.

### 5. Spin
```python
rclpy.spin(node)
```
This keeps the node alive and processing events (timers, messages, etc.)

---

## Running the Node

### Step 1: Make the file executable
```bash
chmod +x hello_node.py
```

### Step 2: Run it
```bash
python3 hello_node.py
```

### Expected Output
```
[INFO] [1234567890.123456789] [hello_humanoid_node]: Hello, Humanoid World!
[INFO] [1234567890.123456789] [hello_humanoid_node]: This node is ready to become part of a robot brain!
[INFO] [1234567892.123456789] [hello_humanoid_node]: Heartbeat #1: Still alive and thinking...
[INFO] [1234567894.123456789] [hello_humanoid_node]: Heartbeat #2: Still alive and thinking...
[INFO] [1234567896.123456789] [hello_humanoid_node]: Heartbeat #3: Still alive and thinking...
```

### Step 3: Stop it
Press `Ctrl+C` to stop the node.

---

## Inspecting Your Node

While the node is running, open a new terminal:

### List all running nodes
```bash
ros2 node list
```
**Output**: `/hello_humanoid_node`

### Get node information
```bash
ros2 node info /hello_humanoid_node
```

This shows:
- Subscriptions (none yet)
- Publications (none yet)
- Services
- Actions

---

## Experiments to Try

### 1. Change the timer frequency
```python
# Make it run every 0.5 seconds (faster)
self.timer = self.create_timer(0.5, self.timer_callback)
```

### 2. Add different log levels
```python
self.get_logger().debug('This is debug info')
self.get_logger().warn('This is a warning!')
self.get_logger().error('This is an error!')
```

### 3. Multiple nodes
Run the same node with different names:
```python
# Terminal 1
node = HelloNode()

# Terminal 2
node2 = HelloNode()  # Creates another instance
```

---

## Humanoid Robot Connection

In a real humanoid robot, you'd have dozens of nodes like this:

```
Robot System
├── vision_node (processes camera)
├── motion_planning_node (plans movements)
├── joint_control_node (controls motors)
├── speech_recognition_node (listens)
├── speech_synthesis_node (speaks)
└── balance_control_node (prevents falling)
```

Each node is independent but they communicate via **Topics** (which we'll learn next!).

---

## Key Takeaways

✅ A node is a self-contained program that does one job
✅ `rclpy.init()` starts the ROS 2 system
✅ `rclpy.spin()` keeps the node alive
✅ Logging helps debug and monitor node behavior
✅ Timers let nodes perform periodic tasks

---

## Next Steps

Now that you can create a basic node, let's learn how to make nodes **communicate** using Publishers and Subscribers! 🚀

**Continue to Example 2 →**
