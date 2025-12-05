# Vision Integration with GPT-4 Vision

## Introduction

**GPT-4 Vision** enables robots to "see" and understand their environment by analyzing camera feeds. This closes the loop: Voice → Vision → Language → Action.

## What is GPT-4 Vision?

GPT-4 Vision (GPT-4V) is a multi-modal model that can:
- **Identify objects** in images
- **Describe scenes** in natural language
- **Answer questions** about visual content
- **Understand spatial relationships** between objects

## Why Vision Matters for Robotics

Traditional computer vision requires:
- Training custom object detection models
- Labeling thousands of images
- Retraining for new objects

**GPT-4 Vision** provides zero-shot object recognition - it can identify objects it has never been explicitly trained on.

## Architecture

```
┌──────────────────────────────────────────────────┐
│         Gazebo Camera (640x480 RGB)              │
└────────────────┬─────────────────────────────────┘
                 │ ROS 2: /camera/image_raw
                 ▼
┌──────────────────────────────────────────────────┐
│    ROS 2 Image Subscriber (Python)               │
│    Convert: ROS Image → JPEG → Base64           │
└────────────────┬─────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────┐
│         GPT-4 Vision API                         │
│    Prompt: "List all objects with positions"    │
└────────────────┬─────────────────────────────────┘
                 │
                 ▼
┌──────────────────────────────────────────────────┐
│         Structured Scene Description             │
│    {                                             │
│      "objects": [                                │
│        {"label": "red_cube", "position": "left"},│
│        {"label": "table", "position": "center"}  │
│      ]                                           │
│    }                                             │
└──────────────────────────────────────────────────┘
```

## Implementation

### Capturing Camera Feed

```python
# backend/src/services/ros2_bridge.py
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64

class ROS2VisionBridge:
    def __init__(self):
        self.node = rclpy.create_node('vision_bridge')
        self.bridge = CvBridge()
        self.latest_image = None

        # Subscribe to camera topic
        self.subscription = self.node.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

    def image_callback(self, msg: Image):
        """Store latest camera frame"""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def get_camera_image_base64(self) -> str:
        """Get current camera frame as base64 JPEG"""
        if self.latest_image is None:
            raise Exception("No camera image available")

        # Encode as JPEG
        _, buffer = cv2.imencode('.jpg', self.latest_image)
        jpeg_bytes = buffer.tobytes()

        # Convert to base64
        return base64.b64encode(jpeg_bytes).decode('utf-8')
```

### GPT-4 Vision Client

```python
# backend/src/services/gpt4_vision_client.py
import openai
from typing import Dict, List

VISION_ANALYSIS_PROMPT = """
You are analyzing a robot's camera feed from a Gazebo simulation.

TASK: List all visible objects with their approximate positions.

OUTPUT FORMAT (JSON):
{
  "objects": [
    {
      "label": "object_name",
      "color": "color if visible",
      "position": "relative position (left/center/right/ahead)",
      "distance": "approximate distance (close/medium/far)",
      "confidence": 0.9
    }
  ],
  "spatial_relations": [
    {"object1": "red_cube", "relation": "left_of", "object2": "blue_cylinder"}
  ],
  "scene_summary": "Brief description of the scene"
}

Only include objects you are confident about (>80% confidence).
"""

class GPT4VisionClient:
    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)

    def analyze_scene(self, image_base64: str, query: str = None) -> Dict:
        """
        Analyze camera image with GPT-4 Vision
        """
        messages = [
            {
                "role": "system",
                "content": VISION_ANALYSIS_PROMPT
            },
            {
                "role": "user",
                "content": [
                    {
                        "type": "text",
                        "text": query or "What objects do you see?"
                    },
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": f"data:image/jpeg;base64,{image_base64}"
                        }
                    }
                ]
            }
        ]

        response = self.client.chat.completions.create(
            model="gpt-4-vision-preview",
            messages=messages,
            max_tokens=500,
            temperature=0
        )

        result = json.loads(response.choices[0].message.content)
        return result
```

### API Endpoint

```python
# backend/src/api/routes/vision.py
from fastapi import APIRouter, HTTPException
import hashlib

router = APIRouter()

@router.post("/vision/analyze")
async def analyze_camera_feed():
    """
    Analyze current Gazebo camera feed with GPT-4 Vision
    """
    try:
        # Capture current frame
        image_base64 = ros2_bridge.get_camera_image_base64()

        # Check cache (avoid repeated API calls for same image)
        cache_key = hashlib.md5(image_base64.encode()).hexdigest()
        cached_result = redis_client.get(f"vision:{cache_key}")

        if cached_result:
            return json.loads(cached_result)

        # Call GPT-4 Vision
        scene_analysis = gpt4_vision_client.analyze_scene(image_base64)

        # Cache for 30 seconds
        redis_client.setex(
            f"vision:{cache_key}",
            30,
            json.dumps(scene_analysis)
        )

        # Save to database
        visual_scene = VisualScene(
            camera_image=base64.b64decode(image_base64),
            scene_description=scene_analysis["scene_summary"],
            detected_objects=scene_analysis["objects"],
            cache_key=cache_key
        )
        db.add(visual_scene)
        db.commit()

        return {
            "visual_scene_id": visual_scene.id,
            "objects": scene_analysis["objects"],
            "spatial_relations": scene_analysis["spatial_relations"],
            "scene_summary": scene_analysis["scene_summary"]
        }

    except Exception as e:
        raise HTTPException(500, f"Vision analysis failed: {str(e)}")
```

## Example Scene Analysis

### Input Image
![Robot camera view showing a table with colored objects]

### GPT-4 Vision Output
```json
{
  "objects": [
    {
      "label": "red_cube",
      "color": "red",
      "position": "left",
      "distance": "close",
      "confidence": 0.92
    },
    {
      "label": "blue_cylinder",
      "color": "blue",
      "position": "center",
      "distance": "medium",
      "confidence": 0.88
    },
    {
      "label": "wooden_table",
      "color": "brown",
      "position": "below_objects",
      "distance": "close",
      "confidence": 0.95
    }
  ],
  "spatial_relations": [
    {"object1": "red_cube", "relation": "left_of", "object2": "blue_cylinder"},
    {"object1": "red_cube", "relation": "on_top_of", "object2": "wooden_table"}
  ],
  "scene_summary": "A table with a red cube on the left and a blue cylinder in the center"
}
```

## Integrating Vision with Task Planning

```python
# Combine vision and language for smart commands
@router.post("/robot/command_with_vision")
async def execute_vision_aware_command(command: str):
    """
    Execute command using both vision and language understanding
    """
    # 1. Analyze current scene
    vision_result = await analyze_camera_feed()

    # 2. Plan task with vision context
    robot_state = ros2_bridge.get_robot_state()

    plan = gpt4_planner.plan_task(
        command=command,
        robot_state=robot_state,
        scene_description=vision_result["scene_summary"]
    )

    # 3. Execute validated actions
    for action in plan["plan"]:
        # Vision-aware safety checks
        if action["action"] == "grasp":
            # Check if target object is actually visible
            target = action["parameters"].get("target_object")
            if not any(obj["label"] == target for obj in vision_result["objects"]):
                raise HTTPException(400, f"Object '{target}' not visible in scene")

        ros2_bridge.execute_action(action)

    return {"status": "success", "plan": plan}
```

## Example: Vision-Guided Grasping

```python
# Command: "Pick up the red cube"

# 1. Vision analysis
vision = await analyze_camera_feed()
# Found: red_cube at position "left", distance "close"

# 2. Task planning with vision context
plan = gpt4_planner.plan_task(
    command="Pick up the red cube",
    robot_state=robot_state,
    scene_description="Red cube on the left, 30cm away"
)

# Plan:
# [
#   {"action": "rotate", "parameters": {"degrees": -15}},  # Turn to face cube
#   {"action": "move_forward", "parameters": {"distance": 0.3}},  # Approach
#   {"action": "grasp", "parameters": {"target_object": "red_cube"}}
# ]
```

## Best Practices

### 1. Optimize Image Quality

```python
# Resize image for faster API calls
def prepare_image_for_vision(image_cv2):
    # Resize to 640x480 (balance quality vs API cost)
    resized = cv2.resize(image_cv2, (640, 480))

    # Compress JPEG (quality 85 is optimal)
    encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
    _, buffer = cv2.imencode('.jpg', resized, encode_params)

    return base64.b64encode(buffer).decode('utf-8')
```

### 2. Cache Aggressively

```python
# Vision API is expensive ($0.01 per image)
# Cache results for 30 seconds if scene hasn't changed
def analyze_with_cache(image_base64: str) -> Dict:
    cache_key = hashlib.md5(image_base64.encode()).hexdigest()

    cached = redis_client.get(f"vision:{cache_key}")
    if cached:
        return json.loads(cached)

    result = gpt4_vision_client.analyze_scene(image_base64)
    redis_client.setex(f"vision:{cache_key}", 30, json.dumps(result))

    return result
```

### 3. Validate Vision Results

```python
# Cross-validate with simple computer vision
def validate_vision_result(gpt4_result: Dict, image: np.ndarray) -> bool:
    """Sanity check GPT-4 Vision with OpenCV"""
    # Example: Check if claimed red object has red pixels
    for obj in gpt4_result["objects"]:
        if obj["color"] == "red":
            # Check HSV red range
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))

            if cv2.countNonZero(red_mask) < 1000:  # Threshold
                logger.warning("GPT-4 claimed red object but no red pixels found")
                return False

    return True
```

## Cost Optimization

GPT-4 Vision costs (~$0.01 per image):

```python
# Track vision API usage
vision_call_count = 0
total_vision_cost = 0.0

def track_vision_cost():
    global vision_call_count, total_vision_cost
    vision_call_count += 1
    total_vision_cost += 0.01

    logger.info(f"Vision calls: {vision_call_count}, Cost: ${total_vision_cost:.2f}")
```

## Next Steps

You've completed the VLA pipeline! Now let's build the **Capstone Project** to demonstrate the complete system.

Continue to: [Capstone Project →](../capstone/overview.md)
