---
sidebar_position: 1
---

# Exercise 1: Build Your First Robot System

## 🎯 Challenge

Build a complete minirobotic system that demonstrates all ROS 2 concepts you've learned:

- Nodes
- Publishers & Subscribers (Topics)
- Service Servers & Clients

---

## 📋 Project: Simple Robot Arm Controller

You'll create a system that:
1. **Simulates** a robot arm with a gripper
2. **Publishes** arm joint positions
3. **Subscribes** to command messages
4. **Provides services** for gripper control

---

## Part 1: Joint State Publisher (20 minutes)

Create `arm_joint_publisher.py`:

**Requirements:**
- Node name: `arm_joint_publisher`
- Topic: `/arm/joint_states`
- Message type: `sensor_msgs/JointState`
- Frequency: 10 Hz

**What to publish:**
- Joint names: `['shoulder', 'elbow', 'wrist']`
- Joint positions: Simulate smooth motion (use sine waves)
- Joint velocities: Calculate from position changes

**Hints:**
```python
from sensor_msgs.msg import JointState
import math

# In timer callback:
joint_state = JointState()
joint_state.header.stamp = self.get_clock().now().to_msg()
joint_state.name = ['shoulder', 'elbow', 'wrist']

# Use sine waves for smooth motion:
t = self.counter * 0.1  # time
joint_state.position = [
    math.sin(t),           # shoulder
    math.sin(t * 0.5),     # elbow
    math.sin(t * 0.25)     # wrist
]
```

**Test it:**
```bash
# Terminal 1
python3 arm_joint_publisher.py

# Terminal 2
ros2 topic echo /arm/joint_states
```

---

## Part 2: Safety Monitor (20 minutes)

Create `arm_safety_monitor.py`:

**Requirements:**
- Node name: `arm_safety_monitor`
- Subscribe to: `/arm/joint_states`
- Check if any joint exceeds safe limits
- Log warnings when limits exceeded

**Safety Limits:**
```python
SAFE_LIMITS = {
    'shoulder': (-1.57, 1.57),  # -90° to +90°
    'elbow': (0, 3.14),          # 0° to 180°
    'wrist': (-0.785, 0.785)     # -45° to +45°
}
```

**Expected behavior:**
- If joint within limits: Log INFO message
- If joint exceeds limits: Log WARNING message
- Count total violations

**Test it:**
```bash
# Terminal 1
python3 arm_joint_publisher.py

# Terminal 2
python3 arm_safety_monitor.py
```

---

## Part 3: Emergency Stop Service (20 minutes)

Create `emergency_stop_service.py`:

**Requirements:**
- Node name: `emergency_stop_service`
- Service name: `/arm/emergency_stop`
- Service type: `std_srvs/Trigger`
- Action: Set a flag that stops the arm

**Response:**
- `success`: True if stopped
- `message`: "Emergency stop activated" or "Already stopped"

**Hints:**
```python
from std_srvs.srv import Trigger

class EmergencyStopService(Node):
    def __init__(self):
        super().__init__('emergency_stop_service')
        self.stopped = False

        self.service = self.create_service(
            Trigger,
            '/arm/emergency_stop',
            self.handle_emergency_stop
        )

    def handle_emergency_stop(self, request, response):
        if not self.stopped:
            self.stopped = True
            response.success = True
            response.message = "🛑 Emergency stop activated!"
        else:
            response.success = False
            response.message = "Already stopped"

        return response
```

**Test it:**
```bash
# Terminal 1
python3 emergency_stop_service.py

# Terminal 2
ros2 service call /arm/emergency_stop std_srvs/srv/Trigger
```

---

## Part 4: Emergency Client (15 minutes)

Create `emergency_client.py`:

**Requirements:**
- Node name: `emergency_client`
- Call `/arm/emergency_stop` service
- Accept command-line argument: `python3 emergency_client.py`

**Expected output:**
```
[INFO] Calling emergency stop service...
[INFO] Response: Emergency stop activated!
```

---

## Part 5: Integration Test (15 minutes)

Run all components together:

```bash
# Terminal 1: Joint Publisher
python3 arm_joint_publisher.py

# Terminal 2: Safety Monitor
python3 arm_safety_monitor.py

# Terminal 3: Emergency Service
python3 emergency_stop_service.py

# Terminal 4: Visualize system
ros2 run rqt_graph rqt_graph
```

You should see:
```
[arm_joint_publisher] --/arm/joint_states--> [arm_safety_monitor]

[emergency_client] --/arm/emergency_stop--> [emergency_stop_service]
```

---

## Bonus Challenges

### Level 1: Add Home Position Service (⭐)
Create a service that moves all joints to position 0.

**Service:** `/arm/go_home`
**Type:** `std_srvs/Trigger`

### Level 2: Add Velocity Calculation (⭐⭐)
Modify the joint publisher to calculate and publish velocities based on position changes.

### Level 3: Visualize in RViz (⭐⭐⭐)
Create a URDF model of your arm and visualize it in RViz with actual joint positions.

**Hint:**
```bash
ros2 launch urdf_tutorial display.launch.py model:=my_arm.urdf
```

### Level 4: Add Custom Service (⭐⭐⭐⭐)
Create a custom service that moves the arm to a specific position.

**Service file:** `MoveArm.srv`
```
# Request
string joint_name
float64 target_position
---
# Response
bool success
string message
float64 actual_position
```

---

## Success Criteria

You've completed the exercise when:

- ✅ All 4 required nodes run without errors
- ✅ Publisher sends joint states at 10 Hz
- ✅ Monitor detects and logs violations
- ✅ Emergency stop service works correctly
- ✅ Can call service from command line
- ✅ System graph shows correct connections

---

## Common Issues & Solutions

### Issue: "ros2: command not found"
**Solution:**
```bash
source /opt/ros/humble/setup.bash
```

### Issue: Service not found
**Solution:** Make sure service server is running first
```bash
ros2 service list  # Check if service is available
```

### Issue: Message type not found
**Solution:** Import correct message:
```bash
pip3 install sensor-msgs  # If needed
```

---

## What You've Learned

🎓 **Congratulations!** You've built a complete ROS 2 system demonstrating:

- ✅ Node creation and lifecycle
- ✅ Publishing sensor data
- ✅ Subscribing and processing messages
- ✅ Creating service servers
- ✅ Calling services from clients
- ✅ System integration and testing

**You're now ready for Chapter 2: Advanced Humanoid Robotics!** 🚀

---

## Share Your Solution

Once complete, you can:
1. Take a screenshot of `rqt_graph`
2. Record a demo video
3. Share your code on GitHub

**Next:** Move on to Chapter 2 where we'll build real humanoid robot behaviors!
