---
sidebar_position: 5
---

# Example 5: Service Client - Making Requests

## What You'll Learn

- Create a service client
- Send requests to service servers
- Handle responses
- Build complete service workflows

---

## The Concept

A **Service Client** sends requests to a Service Server and receives responses. Think of it like making a phone call:
- Client = Caller
- Server = Receiver
- Request = Question
- Response = Answer

---

## The Code

`gripper_client.py`:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
import sys

class GripperClient(Node):
    """
    Service client that sends commands to the gripper service.

    Usage:
        python3 gripper_client.py close
        python3 gripper_client.py open
    """

    def __init__(self):
        super().__init__('gripper_client')

        # Create client
        self.client = self.create_client(SetBool, 'control_gripper')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for gripper service...')

        self.get_logger().info('Connected to gripper service!')

    def send_request(self, close_gripper):
        """
        Send a request to control the gripper.

        Args:
            close_gripper (bool): True to close, False to open

        Returns:
            SetBool.Response: The server's response
        """

        # Create request
        request = SetBool.Request()
        request.data = close_gripper

        action = "close" if close_gripper else "open"
        self.get_logger().info(f'Sending command: {action} gripper...')

        # Send request (asynchronously)
        future = self.client.call_async(request)

        # Wait for response
        rclpy.spin_until_future_complete(self, future)

        # Get response
        response = future.result()

        if response.success:
            self.get_logger().info(f'✓ Success: {response.message}')
        else:
            self.get_logger().warn(f'✗ Failed: {response.message}')

        return response

def main(args=None):
    rclpy.init(args=args)

    # Check command line arguments
    if len(sys.argv) < 2:
        print('Usage: python3 gripper_client.py [open|close]')
        return

    command = sys.argv[1].lower()

    if command not in ['open', 'close']:
        print('Invalid command. Use "open" or "close"')
        return

    # Create client node
    client = GripperClient()

    # Send request
    close_gripper = (command == 'close')
    response = client.send_request(close_gripper)

    # Cleanup
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Running Client + Server

### Terminal 1: Start Server
```bash
python3 gripper_service.py
```

### Terminal 2: Use Client

**Close gripper:**
```bash
python3 gripper_client.py close
```

**Output:**
```
[INFO] [gripper_client]: Connected to gripper service!
[INFO] [gripper_client]: Sending command: close gripper...
[INFO] [gripper_client]: ✓ Success: Gripper closed successfully. Object grasped!
```

**Open gripper:**
```bash
python3 gripper_client.py open
```

---

## Advanced: Sequence of Actions

Create a script that performs multiple actions:

```python
def pick_and_place_sequence(client):
    """
    Simulate a pick-and-place task.

    1. Open gripper
    2. Move to object (simulated)
    3. Close gripper (grasp)
    4. Move to target (simulated)
    5. Open gripper (release)
    """

    print("Starting pick-and-place sequence...")

    # Step 1: Open gripper
    print("1. Opening gripper...")
    client.send_request(False)
    time.sleep(1)

    # Step 2: Move to object
    print("2. Moving to object...")
    time.sleep(2)  # Simulate movement

    # Step 3: Close gripper
    print("3. Grasping object...")
    client.send_request(True)
    time.sleep(1)

    # Step 4: Move to target
    print("4. Moving to target location...")
    time.sleep(2)

    # Step 5: Release object
    print("5. Releasing object...")
    client.send_request(False)

    print("✓ Pick-and-place complete!")
```

---

## Timeout Handling

What if the server is offline?

```python
# Wait maximum 5 seconds for service
if not self.client.wait_for_service(timeout_sec=5.0):
    self.get_logger().error('Service not available!')
    return None
```

---

## Blocking vs Non-Blocking Calls

### Blocking (Simple)
```python
# Wait for response (blocks execution)
response = self.client.call(request)
```

### Non-Blocking (Async)
```python
# Send request, continue doing other things
future = self.client.call_async(request)

# Do other work here...

# Later, check if response is ready
if future.done():
    response = future.result()
```

---

## Real Humanoid Robot Workflow

```python
# Example: Make robot wave hand

# 1. Move shoulder
shoulder_client.call(SetJointPosition, angle=45)

# 2. Move elbow
elbow_client.call(SetJointPosition, angle=90)

# 3. Wave (repeat)
for _ in range(3):
    wrist_client.call(SetJointPosition, angle=30)
    time.sleep(0.5)
    wrist_client.call(SetJointPosition, angle=-30)
    time.sleep(0.5)

# 4. Return to rest position
reset_client.call(ResetArm)
```

---

## Key Takeaways

✅ Clients send requests to servers
✅ Wait for service availability before calling
✅ Handle responses (success/failure)
✅ Use services for coordinated actions

---

## Complete System

Now you can build complete robotic behaviors:

```
[Sensor Publisher] --temp--> [Monitor Subscriber]
                                     |
                                     v
                            (if overheating)
                                     |
                                     v
                            [Emergency Client]
                                     |
                                     v
                            [Shutdown Service]
```

**🎉 Congratulations! You understand ROS 2 communication!**

**Next: Hands-on Exercise to build your own system! →**
