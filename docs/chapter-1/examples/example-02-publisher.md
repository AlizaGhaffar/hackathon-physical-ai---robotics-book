---
sidebar_position: 2
---

# Example 2: Publisher - Sending Messages

## What You'll Learn

- Create a publisher node
- Define and publish messages
- Use standard ROS 2 message types
- Understand Topics

---

## The Concept

A **Publisher** is a node that broadcasts information on a **Topic**. Think of it like a radio station:
- The radio station = Publisher
- The frequency/channel = Topic
- The music/speech = Message

In a humanoid robot, publishers send:
- Sensor data (camera images, IMU readings)
- Joint positions
- Battery status
- Voice commands

---

## The Code

`sensor_publisher.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class SensorPublisher(Node):
    """
    Simulates a humanoid robot's sensor publishing temperature data.

    In a real robot, this would read from actual sensors.
    """

    def __init__(self):
        super().__init__('sensor_publisher')

        # Create a publisher
        # - Topic name: 'robot_temperature'
        # - Message type: String
        # - Queue size: 10 (buffer for messages)
        self.publisher = self.create_publisher(
            String,
            'robot_temperature',
            10
        )

        # Publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_temperature)
        self.get_logger().info('Sensor publisher started!')

    def publish_temperature(self):
        """Publish simulated temperature data"""
        # Simulate temperature reading (36-40°C for robot motors)
        temp = 36 + random.uniform(0, 4)

        # Create message
        msg = String()
        msg.data = f'Motor temperature: {temp:.2f}°C'

        # Publish it
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Running the Publisher

```bash
python3 sensor_publisher.py
```

**Output:**
```
[INFO] [sensor_publisher]: Sensor publisher started!
[INFO] [sensor_publisher]: Publishing: "Motor temperature: 37.23°C"
[INFO] [sensor_publisher]: Publishing: "Motor temperature: 38.91°C"
[INFO] [sensor_publisher]: Publishing: "Motor temperature: 36.45°C"
```

---

## Inspecting Topics

While the publisher runs, open another terminal:

### List all topics
```bash
ros2 topic list
```
**Output:**
```
/parameter_events
/robot_temperature
/rosout
```

### See topic details
```bash
ros2 topic info /robot_temperature
```

### Listen to messages
```bash
ros2 topic echo /robot_temperature
```

This shows all messages being published in real-time!

---

## Message Types

We used `std_msgs/String`, but ROS 2 has many built-in types:

```python
from std_msgs.msg import String, Int32, Float64, Bool
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image, LaserScan, Imu
```

**For humanoid robots:**
- `sensor_msgs/Image` - Camera images
- `sensor_msgs/Imu` - Balance/orientation
- `sensor_msgs/JointState` - Joint positions
- `geometry_msgs/Twist` - Velocity commands

---

## Key Takeaways

✅ Publishers broadcast data on Topics
✅ Multiple nodes can publish to the same Topic
✅ Messages have specific types (String, Int, custom)
✅ Use `ros2 topic` commands to inspect communication

**Next: Example 3 - Create a Subscriber to receive these messages! →**
