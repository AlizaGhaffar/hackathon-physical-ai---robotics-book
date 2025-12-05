# LLM Task Planning with GPT-4

## Introduction

**Large Language Models (LLMs)** like GPT-4 enable robots to understand natural language commands and break them down into executable actions.

## From Language to Actions

### The Challenge

Human commands are often:
- **Vague**: "Go over there"
- **Multi-step**: "Turn right and move to the red cube"
- **Context-dependent**: "Pick up the object" (which object?)

GPT-4 bridges the gap between human language and robot commands.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│         Natural Language Command                        │
│   "Turn right 90 degrees and move forward 2 meters"    │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│              GPT-4 Task Planner                         │
│   System Prompt: "You are a robot task planner..."     │
│   Input: Command + Robot State + Scene Description     │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│            Structured Action Plan (JSON)                │
│   [                                                      │
│     {"action": "rotate", "degrees": 90},                │
│     {"action": "move_forward", "distance": 2.0}         │
│   ]                                                      │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│            Safety Validation Layer                      │
│   Check: collision risk, speed limits, workspace       │
└────────────────────┬────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────┐
│            ROS 2 Execution                              │
│   Publish actions to robot topics                      │
└─────────────────────────────────────────────────────────┘
```

## Implementation

### System Prompt Engineering

```python
ROBOT_TASK_PLANNER_PROMPT = """
You are an expert robot task planner. Convert natural language commands into structured action sequences.

AVAILABLE ACTIONS:
1. move_forward(distance: float) - Move forward N meters
2. move_backward(distance: float) - Move backward N meters
3. rotate(degrees: float) - Rotate N degrees (positive = right, negative = left)
4. grasp() - Close gripper to grasp object
5. release() - Open gripper to release object
6. wait(seconds: float) - Wait N seconds

SAFETY CONSTRAINTS:
- Maximum speed: 1.0 m/s for navigation, 0.3 m/s for manipulation
- Workspace bounds: [-5, 5] meters in X and Y
- Obstacle clearance: Minimum 0.5 meters
- Battery reserve: Keep >15% for emergency stop

OUTPUT FORMAT (JSON):
{
  "plan": [
    {"action": "action_name", "parameters": {...}, "reasoning": "why this step"}
  ],
  "estimated_duration": 10.5,
  "safety_notes": "Obstacles detected ahead",
  "clarification_needed": null
}

If the command is unclear, set "clarification_needed" with a question.
"""
```

### GPT-4 Client Implementation

```python
# backend/src/services/gpt4_client.py
import openai
from typing import Dict, List

class GPT4TaskPlanner:
    def __init__(self, api_key: str):
        self.client = openai.OpenAI(api_key=api_key)

    def plan_task(
        self,
        command: str,
        robot_state: Dict,
        scene_description: str = None
    ) -> Dict:
        """
        Convert natural language command to structured action plan
        """
        # Build context
        context = f"""
        ROBOT STATE:
        - Position: ({robot_state['x']:.2f}, {robot_state['y']:.2f})
        - Heading: {robot_state['theta']:.1f}°
        - Battery: {robot_state['battery']}%
        - Gripper: {'holding object' if robot_state['gripper_closed'] else 'empty'}
        """

        if scene_description:
            context += f"\n\nSCENE DESCRIPTION:\n{scene_description}"

        # Call GPT-4
        response = self.client.chat.completions.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": ROBOT_TASK_PLANNER_PROMPT},
                {"role": "user", "content": f"{context}\n\nCOMMAND: {command}"}
            ],
            temperature=0,  # Deterministic for safety
            response_format={"type": "json_object"}
        )

        plan = json.loads(response.choices[0].message.content)
        return plan
```

### API Endpoint

```python
# backend/src/api/routes/robot.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

router = APIRouter()

class RobotCommandRequest(BaseModel):
    command: str
    visual_scene_id: Optional[str] = None

@router.post("/robot/command")
async def execute_robot_command(request: RobotCommandRequest):
    """
    Process natural language command and execute on robot
    """
    # Get current robot state
    robot_state = ros2_bridge.get_robot_state()

    # Get scene description if vision was used
    scene_description = None
    if request.visual_scene_id:
        scene = db.query(VisualScene).filter_by(id=request.visual_scene_id).first()
        scene_description = scene.scene_description

    # Plan with GPT-4
    plan = gpt4_planner.plan_task(
        command=request.command,
        robot_state=robot_state,
        scene_description=scene_description
    )

    # Check if clarification needed
    if plan.get("clarification_needed"):
        return {
            "status": "clarification_required",
            "question": plan["clarification_needed"]
        }

    # Validate each action for safety
    validated_actions = []
    for action in plan["plan"]:
        validation = safety_validator.validate_action(action, robot_state)
        if not validation.valid:
            raise HTTPException(400, f"Unsafe action: {validation.reason}")
        validated_actions.append(action)

    # Execute actions
    action_ids = []
    for action in validated_actions:
        action_id = ros2_bridge.execute_action(action)
        action_ids.append(action_id)

    return {
        "status": "executing",
        "action_ids": action_ids,
        "estimated_duration": plan["estimated_duration"],
        "plan": plan["plan"]
    }
```

## Example Task Plans

### Simple Command
```json
Input: "Move forward 3 meters"

Output:
{
  "plan": [
    {
      "action": "move_forward",
      "parameters": {"distance": 3.0, "speed": 0.5},
      "reasoning": "Direct forward movement as requested"
    }
  ],
  "estimated_duration": 6.0,
  "safety_notes": "Clear path ahead",
  "clarification_needed": null
}
```

### Multi-Step Command
```json
Input: "Turn right and move to the red cube"

Output:
{
  "plan": [
    {
      "action": "rotate",
      "parameters": {"degrees": 90},
      "reasoning": "Turn to face right direction"
    },
    {
      "action": "move_forward",
      "parameters": {"distance": 1.5, "speed": 0.4},
      "reasoning": "Approach red cube at (1.5, 0) based on vision"
    },
    {
      "action": "wait",
      "parameters": {"seconds": 1.0},
      "reasoning": "Stabilize before manipulation"
    }
  ],
  "estimated_duration": 12.5,
  "safety_notes": "Red cube detected ahead",
  "clarification_needed": null
}
```

### Unclear Command
```json
Input: "Go over there"

Output:
{
  "plan": [],
  "estimated_duration": 0,
  "safety_notes": "Command is ambiguous",
  "clarification_needed": "I see multiple directions. Can you specify: 'forward', 'left', 'right', or point to coordinates?"
}
```

## Best Practices

### 1. Use Temperature=0 for Safety

```python
# Deterministic responses for safety-critical systems
response = client.chat.completions.create(
    model="gpt-4",
    messages=messages,
    temperature=0  # No randomness
)
```

### 2. Validate All Outputs

```python
def validate_plan(plan: Dict) -> bool:
    """Never trust LLM output directly"""
    required_keys = ["plan", "estimated_duration"]
    if not all(k in plan for k in required_keys):
        return False

    for action in plan["plan"]:
        if "action" not in action or "parameters" not in action:
            return False

        # Check action is in allowed list
        if action["action"] not in ALLOWED_ACTIONS:
            return False

    return True
```

### 3. Provide Context

```python
# Include robot state and environment for better planning
context = {
    "robot_position": (x, y, theta),
    "battery_level": battery_percent,
    "nearby_objects": vision_results,
    "workspace_bounds": {"x": [-5, 5], "y": [-5, 5]}
}
```

## Performance Optimization

### Caching Common Commands

```python
COMMON_COMMANDS = {
    "stop": {"plan": [{"action": "stop", "parameters": {}}]},
    "turn around": {"plan": [{"action": "rotate", "parameters": {"degrees": 180}}]},
}

def plan_task_with_cache(command: str, **kwargs):
    # Check cache first
    if command.lower() in COMMON_COMMANDS:
        return COMMON_COMMANDS[command.lower()]

    # Otherwise call GPT-4
    return gpt4_planner.plan_task(command, **kwargs)
```

### Parallel Processing

```python
import asyncio

async def plan_and_execute(commands: List[str]):
    # Plan all commands in parallel
    tasks = [gpt4_planner.plan_task_async(cmd) for cmd in commands]
    plans = await asyncio.gather(*tasks)

    # Execute sequentially
    for plan in plans:
        await execute_plan(plan)
```

## Cost Management

GPT-4 API costs (~$0.03 per 1K tokens):

```python
# Track token usage
def plan_task_with_tracking(command: str, **kwargs):
    response = gpt4_planner.plan_task(command, **kwargs)

    tokens_used = response.usage.total_tokens
    cost = (tokens_used / 1000) * 0.03

    logger.info(f"GPT-4 call: {tokens_used} tokens, ${cost:.4f}")

    return response
```

## Next Steps

You've learned how to use GPT-4 for task planning. Next, we'll add **vision capabilities** so the robot can understand what it sees!

Continue to: [Multi-modal Integration →](../multimodal/gpt4-vision.md)
