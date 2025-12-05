---
sidebar_position: 2
---

# Topics

## The Communication Highway of ROS 2

**Topics** are named channels that nodes use to exchange messages using a **publish-subscribe** pattern. They're the primary way nodes communicate in ROS 2.

## 🎯 What is a Topic?

Think of topics like **radio stations**:
- Publishers broadcast messages (like radio transmitters)
- Subscribers listen to messages (like radio receivers)
- Multiple nodes can publish/subscribe to the same topic
- Messages are **one-way** and **asynchronous**

### Visual Representation

```
Publisher Node          Topic              Subscriber Nodes
┌────────────┐      ┌──────────┐         ┌──────────────┐
│  Camera    │─────▶│ /image   │────────▶│  Viewer      │
│  Driver    │      │ (Image)  │    │    └──────────────┘
└────────────┘      └──────────┘    │
                                    │    ┌──────────────┐
                                    └───▶│  Detector    │
                                         └──────────────┘
```

## 🔄 Publish-Subscribe Pattern

### Why Publish-Subscribe?

**Alternative: Direct Connection**
```
❌ Camera → Viewer (tightly coupled)
```
- Camera needs to know about Viewer
- Hard to add new subscribers
- Changes require modifying both nodes

**Better: Topic-Based**
```
✅ Camera → /image → [Viewer, Detector, Logger]
```
- Camera just publishes, doesn't care who listens
- Add subscribers without changing publisher
- Loose coupling between nodes

## 💻 Creating Publishers

### Simple Publisher Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')

        # Create publisher
        # Arguments: message_type, topic_name, queue_size
        self.publisher = self.create_publisher(String, 'greeting', 10)

        # Publish every 0.5 seconds
        self.timer = self.create_timer(0.5, self.publish_message)
        self.counter = 0

    def publish_message(self):
        """Callback to publish messages"""
        msg = String()
        msg.data = f'Hello ROS 2! Count: {self.counter}'

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: "{msg.data}"')
        self.counter += 1

def main():
    rclpy.init()
    node = SimplePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running the Publisher

```bash
python3 simple_publisher.py
```

**Output:**
```
[INFO] [simple_publisher]: Published: "Hello ROS 2! Count: 0"
[INFO] [simple_publisher]: Published: "Hello ROS 2! Count: 1"
[INFO] [simple_publisher]: Published: "Hello ROS 2! Count: 2"
```

## 📡 Creating Subscribers

### Simple Subscriber Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')

        # Create subscriber
        # Arguments: message_type, topic_name, callback, queue_size
        self.subscription = self.create_subscription(
            String,
            'greeting',
            self.message_callback,
            10
        )

    def message_callback(self, msg):
        """Called when a message is received"""
        self.get_logger().info(f'Received: "{msg.data}"')

def main():
    rclpy.init()
    node = SimpleSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running Publisher + Subscriber

**Terminal 1:**
```bash
python3 simple_publisher.py
```

**Terminal 2:**
```bash
python3 simple_subscriber.py
```

**Terminal 2 Output:**
```
[INFO] [simple_subscriber]: Received: "Hello ROS 2! Count: 0"
[INFO] [simple_subscriber]: Received: "Hello ROS 2! Count: 1"
[INFO] [simple_subscriber]: Received: "Hello ROS 2! Count: 2"
```

## 🔍 Topic Introspection

### List All Active Topics

```bash
ros2 topic list
```

**Output:**
```
/greeting
/parameter_events
/rosout
```

### Show Topic Information

```bash
ros2 topic info /greeting
```

**Output:**
```
Type: std_msgs/msg/String
Publisher count: 1
Subscription count: 1
```

### Echo Topic Messages

```bash
ros2 topic echo /greeting
```

**Output:**
```
data: 'Hello ROS 2! Count: 0'
---
data: 'Hello ROS 2! Count: 1'
---
```

### Measure Topic Frequency

```bash
ros2 topic hz /greeting
```

**Output:**
```
average rate: 2.000
  min: 0.499s max: 0.501s std dev: 0.001s window: 10
```

## 📦 Message Types

Topics carry messages of specific types. Common types:

### Standard Messages (`std_msgs`)

```python
from std_msgs.msg import String, Int32, Float64, Bool

# String message
msg = String()
msg.data = "Hello"

# Integer message
msg = Int32()
msg.data = 42
```

### Sensor Messages (`sensor_msgs`)

```python
from sensor_msgs.msg import Image, LaserScan, Imu

# Image message
img_msg = Image()
img_msg.height = 480
img_msg.width = 640
img_msg.encoding = "rgb8"

# LaserScan message
scan_msg = LaserScan()
scan_msg.ranges = [1.0, 1.5, 2.0, ...]
```

### Geometry Messages (`geometry_msgs`)

```python
from geometry_msgs.msg import Twist, Pose, Point

# Velocity command
cmd_vel = Twist()
cmd_vel.linear.x = 1.0  # Move forward
cmd_vel.angular.z = 0.5  # Turn right
```

## 🎨 Real-World Example: Robot Vision

### Camera Publisher Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # Publisher for camera images
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)

        # OpenCV camera capture
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()

        # Publish at 30 FPS
        self.timer = self.create_timer(1.0/30.0, self.capture_and_publish)

    def capture_and_publish(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert OpenCV image to ROS message
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher.publish(img_msg)
            self.get_logger().info('Published camera frame')

def main():
    rclpy.init()
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Object Detector Subscriber Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')

        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.detect_objects,
            10
        )
        self.bridge = CvBridge()

    def detect_objects(self, msg):
        # Convert ROS message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run object detection (simplified)
        # In real code: use YOLO, TensorFlow, etc.
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 50)

        if circles is not None:
            self.get_logger().info(f'Detected {len(circles[0])} objects')

def main():
    rclpy.init()
    node = ObjectDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## ⚙️ Quality of Service (QoS)

QoS policies control how messages are delivered:

### Reliability

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Best Effort (fast, may lose messages)
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    depth=10
)

# Reliable (slower, guarantees delivery)
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    depth=10
)

# Use in publisher/subscriber
self.publisher = self.create_publisher(Image, 'camera', qos_profile)
```

### Durability

```python
from rclpy.qos import DurabilityPolicy

# Volatile (new subscribers miss old messages)
qos_profile = QoSProfile(
    durability=DurabilityPolicy.VOLATILE,
    depth=10
)

# Transient Local (new subscribers get last message)
qos_profile = QoSProfile(
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)
```

### When to Use What?

| Use Case | Reliability | Durability | Depth |
|----------|-------------|------------|-------|
| **Sensor data** | Best Effort | Volatile | 10 |
| **Commands** | Reliable | Volatile | 10 |
| **Status** | Reliable | Transient Local | 1 |

## 🧪 Testing Topics

### Manual Testing with Command Line

```bash
# Publish a single message
ros2 topic pub /greeting std_msgs/msg/String "data: 'Test message'"

# Publish continuously at 1 Hz
ros2 topic pub -r 1 /greeting std_msgs/msg/String "data: 'Periodic test'"
```

### Automated Testing

```python
import unittest
import rclpy
from std_msgs.msg import String

class TestTopicCommunication(unittest.TestCase):
    def test_publish_subscribe(self):
        rclpy.init()

        # Create publisher
        pub_node = rclpy.create_node('test_publisher')
        publisher = pub_node.create_publisher(String, 'test_topic', 10)

        # Create subscriber
        sub_node = rclpy.create_node('test_subscriber')
        received_msgs = []

        def callback(msg):
            received_msgs.append(msg.data)

        subscriber = sub_node.create_subscription(
            String, 'test_topic', callback, 10
        )

        # Publish message
        msg = String()
        msg.data = 'Test'
        publisher.publish(msg)

        # Spin to process message
        rclpy.spin_once(sub_node, timeout_sec=1.0)

        # Assert message received
        self.assertEqual(len(received_msgs), 1)
        self.assertEqual(received_msgs[0], 'Test')

        pub_node.destroy_node()
        sub_node.destroy_node()
        rclpy.shutdown()
```

## 🎯 Key Takeaways

- 📡 Topics use **publish-subscribe** pattern for one-way communication
- 🔀 **Many-to-many**: Multiple publishers and subscribers per topic
- 📦 Messages have **specific types** (String, Image, Twist, etc.)
- ⚙️ **QoS policies** control reliability, durability, and history
- 🔍 Use `ros2 topic` commands to inspect, echo, and test topics
- 🎨 Topics enable **loose coupling** between nodes

---

## Practice Exercise

**Challenge:** Create a temperature monitoring system:

1. **Publisher Node**: Publishes random temperature values (20-30°C) to `/temperature` topic every second
2. **Subscriber Node**: Subscribes to `/temperature` and logs a warning if temperature > 28°C

**Bonus:** Add a third node that calculates the average temperature over the last 10 readings!

---

## What's Next?

Topics are great for streaming data, but what about **request-response** patterns? That's where **Services** come in!

Click **"Services"** in the sidebar to learn more. 🚀
