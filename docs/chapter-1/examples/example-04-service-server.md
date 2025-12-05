---
sidebar_position: 4
---

# Example 4: Service Server - Handling Requests

## What You'll Learn

- Create a service server
- Define service interfaces
- Handle synchronous requests
- Difference between Topics and Services

---

## Topics vs Services

| **Topics** | **Services** |
|------------|--------------|
| One-way broadcast | Request-Response |
| Continuous stream | On-demand |
| Many-to-many | One-to-one |
| Example: Sensor data | Example: "Calculate path" |

In a humanoid robot:
- **Topics**: Continuous sensor data, joint states
- **Services**: "Move arm to position X", "Take photo now"

---

## The Code

`gripper_service.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class GripperService(Node):
    """
    Service server that controls a robot gripper (hand).

    Service: /control_gripper
    Request: SetBool (True = close, False = open)
    Response: success (bool), message (string)
    """

    def __init__(self):
        super().__init__('gripper_service')

        # Create service
        self.service = self.create_service(
            SetBool,
            'control_gripper',
            self.handle_gripper_request
        )

        self.gripper_closed = False  # Initial state: open
        self.get_logger().info('Gripper service ready! Waiting for commands...')

    def handle_gripper_request(self, request, response):
        """
        Handle incoming service requests.

        Args:
            request: SetBool.Request (contains 'data' field: bool)
            response: SetBool.Response (contains 'success' and 'message' fields)

        Returns:
            response: Filled response object
        """

        # Log the request
        action = "CLOSE" if request.data else "OPEN"
        self.get_logger().info(f'Received request: {action} gripper')

        # Simulate gripper action
        if request.data:  # Close gripper
            if self.gripper_closed:
                response.success = False
                response.message = 'Gripper is already closed!'
            else:
                self.gripper_closed = True
                response.success = True
                response.message = 'Gripper closed successfully. Object grasped!'
                self.get_logger().info('🤏 Gripper CLOSED')

        else:  # Open gripper
            if not self.gripper_closed:
                response.success = False
                response.message = 'Gripper is already open!'
            else:
                self.gripper_closed = False
                response.success = True
                response.message = 'Gripper opened successfully. Object released!'
                self.get_logger().info('✋ Gripper OPEN')

        return response

def main(args=None):
    rclpy.init(args=args)
    node = GripperService()

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

## Running the Service Server

```bash
python3 gripper_service.py
```

**Output:**
```
[INFO] [gripper_service]: Gripper service ready! Waiting for commands...
```

The server is now waiting for requests!

---

## Testing the Service

### List Available Services
```bash
ros2 service list
```

**Output:**
```
/control_gripper
/gripper_service/describe_parameters
...
```

### Get Service Info
```bash
ros2 service type /control_gripper
```

**Output**: `example_interfaces/srv/SetBool`

### Call the Service (Command Line)

**Close gripper:**
```bash
ros2 service call /control_gripper example_interfaces/srv/SetBool "{data: true}"
```

**Server logs:**
```
[INFO] [gripper_service]: Received request: CLOSE gripper
[INFO] [gripper_service]: 🤏 Gripper CLOSED
```

**Response:**
```
success: True
message: 'Gripper closed successfully. Object grasped!'
```

**Open gripper:**
```bash
ros2 service call /control_gripper example_interfaces/srv/SetBool "{data: false}"
```

---

## Service Interface

The `SetBool` service interface has:

**Request:**
```
bool data  # True or False
```

**Response:**
```
bool success  # Did it work?
string message  # Description
```

### Custom Service Files

For complex robots, create custom services:

`GripperCommand.srv`:
```
# Request
float32 position  # 0.0 = open, 1.0 = fully closed
float32 force     # Maximum force in Newtons
---
# Response
bool success
string message
float32 actual_position  # Where gripper actually ended up
```

---

## Real Humanoid Robot Example

```python
# In a real humanoid robot control system:

# Service 1: Move arm to position
/move_arm_to_position
Request: target_x, target_y, target_z
Response: success, final_position

# Service 2: Take photo
/capture_image
Request: camera_id, resolution
Response: success, image_data

# Service 3: Check balance
/check_stability
Request: (empty)
Response: stable (bool), center_of_mass
```

---

## Key Takeaways

✅ Services are request-response (like function calls)
✅ Server waits for requests, processes them, returns response
✅ Use services for on-demand actions, not continuous data
✅ One server can handle multiple sequential requests

**Next: Example 5 - Create a Service Client! →**
