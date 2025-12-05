# Example: Complete Voice-to-Robot Demo

## Overview

This example demonstrates the complete VLA pipeline: Voice → Vision → Language → Action.

## Scenario

Control a robot in Gazebo using only your voice.

## Prerequisites

- Chapters 1 & 2 completed
- OpenAI API key configured
- Gazebo running with robot

## Step 1: Launch the System

```bash
# Terminal 1: Start Gazebo
ros2 launch my_robot_description gazebo.launch.py

# Terminal 2: Start ROS 2 bridge
cd backend
python -m src.services.ros2_bridge

# Terminal 3: Start FastAPI backend
cd backend
uvicorn src.main:app --reload

# Terminal 4: Start Docusaurus frontend
npm start
```

## Step 2: Test Voice Command

1. Open browser to `http://localhost:3000/chapter-3/capstone`
2. Click **🎤 Press to Speak** button
3. Say clearly: **"Move forward 2 meters"**
4. Click **Stop Recording**

**Expected Result**:
```
Transcription: "Move forward 2 meters"
Status: ✓ Transcribed successfully
```

## Step 3: View Task Plan

The system automatically sends the transcription to GPT-4:

```json
{
  "plan": [
    {
      "action": "move_forward",
      "parameters": {"distance": 2.0, "speed": 0.5},
      "reasoning": "Execute forward movement as requested"
    }
  ],
  "estimated_duration": 4.0,
  "safety_validated": true
}
```

## Step 4: Watch Execution

The robot in Gazebo will:
1. Start moving forward smoothly
2. Progress bar shows movement (0% → 100%)
3. Stops exactly at 2.0 meters
4. Status changes to ✓ Complete

## Step 5: Try Complex Command

Say: **"Turn right 90 degrees and move to the red cube"**

**Expected Plan**:
```json
{
  "plan": [
    {"action": "rotate", "parameters": {"degrees": 90}},
    {"action": "move_forward", "parameters": {"distance": 1.5}},
    {"action": "wait", "parameters": {"seconds": 1.0}}
  ],
  "estimated_duration": 12.0
}
```

Watch the robot execute all three actions sequentially!

## Step 6: Test Vision Integration

1. Click **👁️ Analyze Scene** button
2. Wait 2-3 seconds for GPT-4 Vision

**Expected Vision Result**:
```json
{
  "objects": [
    {"label": "red_cube", "position": "ahead", "confidence": 0.92},
    {"label": "table", "position": "below", "confidence": 0.88}
  ],
  "scene_summary": "A red cube sits on a wooden table ahead of the robot"
}
```

## Step 7: Vision-Guided Grasping

Say: **"Pick up the red cube"**

The system will:
1. Use vision to locate the red cube
2. Plan approach path
3. Execute grasp action
4. Lift the cube

**Expected Actions**:
```json
[
  {"action": "move_forward", "parameters": {"distance": 0.5}},
  {"action": "lower_arm", "parameters": {"height": 0.1}},
  {"action": "grasp", "parameters": {"target": "red_cube"}},
  {"action": "lift_arm", "parameters": {"height": 0.3}}
]
```

## Step 8: Test Safety Validator

Say: **"Drive off the platform"**

**Expected Rejection**:
```
❌ Command rejected by safety validator
Reason: Requested movement would exit workspace bounds ([-5, 5]m)
Suggestion: Try "Move forward 2 meters" instead
```

## Complete Demo Script (90 seconds)

Perfect for demonstrations:

```
00:00 - 00:15  Voice Command: "Describe what you see"
                → Robot lists objects detected by GPT-4 Vision

00:15 - 00:30  Voice Command: "Move forward 2 meters"
                → Robot navigates smoothly

00:30 - 00:50  Voice Command: "Turn right and pick up the red cube"
                → Robot plans multi-step task, executes grasping

00:50 - 01:15  Voice Command: "Place the cube on the table"
                → Robot releases object, returns to idle

01:15 - 01:30  Demonstrate safety: "Drive off platform"
                → System rejects with explanation
```

## Troubleshooting

### Issue: "No audio detected"
**Solution**: Check microphone permissions in browser settings

### Issue: "Whisper API timeout"
**Solution**: Check internet connection, ensure API key is valid

### Issue: "Robot not moving"
**Solution**: Verify ROS 2 bridge is running (`ros2 topic list` should show `/cmd_vel`)

### Issue: "Vision analysis failed"
**Solution**: Ensure Gazebo camera plugin is active (`ros2 topic echo /camera/image_raw`)

## Code Reference

Full implementation:
- Voice UI: `frontend/src/components/chapter3/VoiceCommandUI.tsx`
- Vision: `backend/src/services/gpt4_vision_client.py`
- Planning: `backend/src/services/gpt4_client.py`
- ROS 2: `backend/src/services/ros2_bridge.py`

## Next Steps

- Customize the system prompt for different tasks
- Add new robot actions (e.g., waving, dancing)
- Train on your own voice for better accuracy
- Deploy to a real robot!

Congratulations on completing the VLA pipeline! 🎉
