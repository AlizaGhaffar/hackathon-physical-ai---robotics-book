---
sidebar_position: 3
---

# Services

## Request-Response Communication in ROS 2

While **Topics** are great for continuous data streams, **Services** provide **synchronous request-response** communication between nodes.

## 🎯 What is a Service?

Think of services like **phone calls**:
- **Client** calls the service (makes a request)
- **Server** processes the request and sends back a response
- Communication is **two-way** and **synchronous** (client waits for response)

### Topics vs Services

```
TOPIC (Asynchronous)           SERVICE (Synchronous)
Publisher → Topic → Subscriber   Client → Request → Server
                                         ← Response ←
Continuous streaming             One-time request-response
Many-to-many                     One-to-one
Fire and forget                  Wait for result
```

## 📞 When to Use Services?

Use services for:
- ✅ **Triggering actions** - Start/stop motors, capture image
- ✅ **Configuration** - Set parameters, calibrate sensors
- ✅ **Computations** - "Calculate path from A to B"
- ✅ **Queries** - "What's the robot's current status?"

Use topics for:
- ✅ **Sensor data** - Camera images, IMU readings
- ✅ **Commands** - Velocity commands, joint positions
- ✅ **Status updates** - Battery level, temperature

## 💻 Creating a Service Server

### Simple Service Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class AdditionServer(Node):
    def __init__(self):
        super().__init__('addition_server')

        # Create service
        # Arguments: service_type, service_name, callback
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )
        self.get_logger().info('Addition service ready!')

    def add_callback(self, request, response):
        """Handle service requests"""
        # Access request data
        a = request.a
        b = request.b

        # Compute result
        response.sum = a + b

        # Log the operation
        self.get_logger().info(f'Request: {a} + {b} = {response.sum}')

        # Return response
        return response

def main():
    rclpy.init()
    node = AdditionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 📱 Creating a Service Client

### Simple Client Example

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys

class AdditionClient(Node):
    def __init__(self):
        super().__init__('addition_client')

        # Create client
        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

    def send_request(self, a, b):
        """Send addition request"""
        # Create request
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        # Call service (async)
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()

    # Get numbers from command line
    if len(sys.argv) != 3:
        print('Usage: python3 client.py <num1> <num2>')
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    # Create client and send request
    client = AdditionClient()
    future = client.send_request(a, b)

    # Wait for response
    rclpy.spin_until_future_complete(client, future)

    # Get result
    if future.result() is not None:
        response = future.result()
        client.get_logger().info(f'Result: {a} + {b} = {response.sum}')
    else:
        client.get_logger().error('Service call failed')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Running Server + Client

**Terminal 1 (Server):**
```bash
python3 addition_server.py
```

**Terminal 2 (Client):**
```bash
python3 addition_client.py 5 7
```

**Output:**
```
# Terminal 1 (Server)
[INFO] [addition_server]: Addition service ready!
[INFO] [addition_server]: Request: 5 + 7 = 12

# Terminal 2 (Client)
[INFO] [addition_client]: Result: 5 + 7 = 12
```

## 🔍 Service Introspection

### List All Services

```bash
ros2 service list
```

**Output:**
```
/add_two_ints
/addition_server/describe_parameters
/addition_server/get_parameters
/addition_server/list_parameters
/addition_server/set_parameters
```

### Show Service Type

```bash
ros2 service type /add_two_ints
```

**Output:**
```
example_interfaces/srv/AddTwoInts
```

### Call Service from Command Line

```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 10, b: 20}"
```

**Output:**
```
waiting for service to become available...
requester: making request: example_interfaces.srv.AddTwoInts_Request(a=10, b=20)

response:
example_interfaces.srv.AddTwoInts_Response(sum=30)
```

## 📦 Service Message Structure

Services have **two parts**: Request and Response

### Standard Service Types

```python
# AddTwoInts.srv
int64 a
int64 b
---
int64 sum

# SetBool.srv
bool data
---
bool success
string message

# Trigger.srv
---
bool success
string message
```

### Custom Service Definition

Create `my_interfaces/srv/ComputePath.srv`:

```
# Request
float64 start_x
float64 start_y
float64 goal_x
float64 goal_y
---
# Response
float64[] path_x
float64[] path_y
bool success
string message
```

## 🤖 Real-World Example: Robot Controller

### Camera Capture Service

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import cv2

class CameraService(Node):
    def __init__(self):
        super().__init__('camera_service')

        # Create service for capturing images
        self.srv = self.create_service(
            Trigger,
            'capture_image',
            self.capture_callback
        )

        self.cap = cv2.VideoCapture(0)
        self.get_logger().info('Camera service ready!')

    def capture_callback(self, request, response):
        """Capture and save an image"""
        ret, frame = self.cap.read()

        if ret:
            # Save image
            filename = 'captured_image.jpg'
            cv2.imwrite(filename, frame)

            response.success = True
            response.message = f'Image saved to {filename}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = 'Failed to capture image'
            self.get_logger().error(response.message)

        return response

def main():
    rclpy.init()
    node = CameraService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

### Robot Status Service

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        self.motor_enabled = False

        # Service to enable/disable motors
        self.srv = self.create_service(
            SetBool,
            'enable_motors',
            self.enable_callback
        )
        self.get_logger().info('Motor controller ready!')

    def enable_callback(self, request, response):
        """Enable or disable motors"""
        self.motor_enabled = request.data

        if self.motor_enabled:
            response.success = True
            response.message = 'Motors enabled'
            self.get_logger().info('Motors ENABLED')
        else:
            response.success = True
            response.message = 'Motors disabled'
            self.get_logger().info('Motors DISABLED')

        return response

def main():
    rclpy.init()
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## ⚠️ Async vs Sync Service Calls

### Asynchronous (Recommended)

```python
# Non-blocking call
future = client.call_async(request)

# Continue doing other work...
self.get_logger().info('Request sent, waiting for response...')

# Check if response is ready
if future.done():
    response = future.result()
```

### Synchronous (Blocking)

```python
# Blocking call - NOT RECOMMENDED in callbacks
response = client.call(request)  # Waits until response received
```

**Why Async?**
- Node can process other callbacks while waiting
- Prevents deadlocks
- Better performance for multiple service calls

## 🧪 Testing Services

### Manual Testing

```bash
# List services
ros2 service list

# Check service type
ros2 service type /enable_motors

# Call service
ros2 service call /enable_motors example_interfaces/srv/SetBool "{data: true}"
```

### Automated Testing

```python
import unittest
import rclpy
from std_srvs.srv import Trigger

class TestCameraService(unittest.TestCase):
    def test_service_call(self):
        rclpy.init()

        # Create test client
        node = rclpy.create_node('test_client')
        client = node.create_client(Trigger, 'capture_image')

        # Wait for service
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))

        # Send request
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        # Check response
        response = future.result()
        self.assertTrue(response.success)
        self.assertIn('saved', response.message)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    unittest.main()
```

## 🎯 Best Practices

### 1. Service Naming Conventions

```python
# Good naming
/robot/enable_motors
/camera/capture_image
/navigation/compute_path

# Bad naming
/service1
/do_stuff
/thing
```

### 2. Timeout Handling

```python
# Always set timeouts
if not client.wait_for_service(timeout_sec=5.0):
    self.get_logger().error('Service not available!')
    return

# Set timeout for call
future = client.call_async(request)
rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

if future.result() is None:
    self.get_logger().error('Service call timed out!')
```

### 3. Error Handling

```python
def service_callback(self, request, response):
    try:
        # Process request
        result = self.complex_operation(request.data)
        response.success = True
        response.message = 'Success'
    except Exception as e:
        response.success = False
        response.message = f'Error: {str(e)}'
        self.get_logger().error(f'Service error: {e}')

    return response
```

## 📊 Services vs Topics vs Actions

| Feature | Topics | Services | Actions |
|---------|--------|----------|---------|
| **Pattern** | Pub-Sub | Request-Response | Goal-Feedback-Result |
| **Synchronous** | No | Yes | No |
| **Many-to-many** | Yes | No | No |
| **Continuous** | Yes | No | Yes |
| **Cancellable** | N/A | No | Yes |
| **Feedback** | No | No | Yes |
| **Use case** | Sensor data | Compute/trigger | Long tasks |

## 🎯 Key Takeaways

- 📞 Services provide **synchronous request-response** communication
- 🔄 Use services for **one-time operations**, topics for **continuous data**
- ⏱️ Services **block** the client until response is received
- 🎯 Best for **triggering actions**, **queries**, and **computations**
- 🔍 Use `ros2 service` commands to list, call, and test services
- ⚡ Prefer **async calls** to avoid blocking node execution

---

## Practice Exercise

**Challenge:** Create a calculator service that supports multiple operations:

1. **Service Definition**: Create `Calculator.srv` with:
   - Request: `float64 a`, `float64 b`, `string operation` (add/subtract/multiply/divide)
   - Response: `float64 result`, `bool success`, `string message`

2. **Server Node**: Implement calculator logic with error handling (e.g., divide by zero)

3. **Client Node**: Send requests from command line

**Bonus:** Add logging to track all calculations performed!

---

## What's Next?

You now understand the three core communication patterns in ROS 2:
- ✅ **Nodes** - Independent computational units
- ✅ **Topics** - Continuous data streaming
- ✅ **Services** - Request-response operations

Next, learn about **Actions** (for long-running tasks with feedback) and dive into hands-on examples!

Continue exploring the sidebar for installation guides and practical examples. 🚀
