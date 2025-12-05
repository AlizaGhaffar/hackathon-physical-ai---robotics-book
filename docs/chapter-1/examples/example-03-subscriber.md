---
sidebar_position: 3
---

# Example 3: Subscriber - Receiving Messages

## What You'll Learn

- Create a subscriber node
- Process incoming messages
- Connect publishers and subscribers

---

## The Concept

A **Subscriber** listens to a Topic and processes messages. If a Publisher is a radio station, a Subscriber is a radio receiver.

In a humanoid robot:
- Vision node subscribes to camera images
- Safety node subscribes to temperature alerts
- Motion controller subscribes to joint commands

---

## The Code

`monitor_subscriber.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TemperatureMonitor(Node):
    """
    Subscribes to temperature data and monitors for overheating.

    In a real robot, this would trigger cooling or shutdown protocols.
    """

    def __init__(self):
        super().__init__('temperature_monitor')

        # Create a subscriber
        # - Topic: 'robot_temperature'
        # - Message type: String
        # - Callback function: temperature_callback
        self.subscription = self.create_subscription(
            String,
            'robot_temperature',
            self.temperature_callback,
            10
        )

        self.get_logger().info('Temperature monitor started. Watching for overheating...')

    def temperature_callback(self, msg):
        """Called whenever a message arrives on the topic"""
        # Extract temperature value from message
        temp_str = msg.data.split(':')[1].strip()
        temp_value = float(temp_str.replace('°C', ''))

        # Check for overheating
        if temp_value > 38.5:
            self.get_logger().warn(f'⚠️  OVERHEATING DETECTED: {temp_value:.2f}°C')
        else:
            self.get_logger().info(f'✓ Temperature normal: {temp_value:.2f}°C')

def main(args=None):
    rclpy.init(args=args)
    node = TemperatureMonitor()

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

## Running Publisher + Subscriber

### Terminal 1: Start Publisher
```bash
python3 sensor_publisher.py
```

### Terminal 2: Start Subscriber
```bash
python3 monitor_subscriber.py
```

**Subscriber Output:**
```
[INFO] [temperature_monitor]: Temperature monitor started...
[INFO] [temperature_monitor]: ✓ Temperature normal: 37.23°C
[WARN] [temperature_monitor]: ⚠️  OVERHEATING DETECTED: 38.91°C
[INFO] [temperature_monitor]: ✓ Temperature normal: 36.45°C
```

---

## Visualizing Communication

```bash
# See the connection graph
ros2 run rqt_graph rqt_graph
```

You'll see:
```
[sensor_publisher] --robot_temperature--> [temperature_monitor]
```

---

## Multiple Subscribers

You can run multiple subscribers on the same topic!

```bash
# Terminal 3: Another monitor
python3 monitor_subscriber.py
```

Both subscribers receive ALL messages - this is pub/sub pattern power!

---

## Key Takeaways

✅ Subscribers listen to Topics
✅ Callbacks process each incoming message
✅ Multiple subscribers can listen to one Topic
✅ Publishers don't know who's listening (decoupled)

**Next: Example 4 - Services for request-response patterns! →**
