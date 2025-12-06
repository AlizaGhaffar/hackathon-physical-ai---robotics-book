# Capstone Project: Autonomous Humanoid System

## Introduction

Congratulations! You've learned Voice AI, LLM Task Planning, and Computer Vision. Now it's time to **put it all together** in a complete autonomous system.

## Project Goal

Build a **fully autonomous robot** that:

1. ✅ **Listens** to voice commands (Whisper)
2. ✅ **Sees** its environment (GPT-4 Vision)
3. ✅ **Thinks** and plans tasks (GPT-4)
4. ✅ **Acts** safely in simulation (ROS 2 + Gazebo)

## Capstone Structure

The project is divided into **6 modular phases**. Each phase builds on the previous one and can be completed independently.

### Phase 1: Voice Input Module (20 minutes)
**Goal**: Implement voice command capture and transcription

**Tasks**:
- Set up browser audio recording
- Integrate Whisper API
- Display transcriptions in UI
- Test with sample commands

**Test**: Record "Move forward 2 meters" → See transcription displayed

---

### Phase 2: Vision Analysis Module (25 minutes)
**Goal**: Capture Gazebo camera feed and analyze with GPT-4 Vision

**Tasks**:
- Subscribe to ROS 2 camera topic
- Convert images to base64
- Call GPT-4 Vision API
- Display detected objects

**Test**: Point camera at objects → See "red cube, blue cylinder" listed

---

### Phase 3: Task Planning Module (30 minutes)
**Goal**: Convert natural language to structured robot actions

**Tasks**:
- Implement GPT-4 task planner
- Define robot action types
- Parse JSON responses
- Handle multi-step commands

**Test**: Input "Turn right and move forward" → See action plan generated

---

### Phase 4: Safety Validation Module (20 minutes)
**Goal**: Implement 5 safety rules to protect robot

**Tasks**:
- Collision detection check
- Speed limit enforcement
- Workspace boundary validation
- Battery level check
- Gripper state validation

**Test**: Try unsafe command → See rejection with explanation

---

### Phase 5: ROS 2 Execution Module (30 minutes)
**Goal**: Execute validated actions on robot in Gazebo

**Tasks**:
- Implement ROS 2 action publishers
- Handle /cmd_vel for navigation
- Handle /gripper_command for manipulation
- Real-time status updates

**Test**: Execute plan → Watch robot move in Gazebo

---

### Phase 6: Full Integration (30 minutes)
**Goal**: Connect all modules into complete VLA pipeline

**Tasks**:
- Wire voice → vision → planning → safety → execution
- Add error handling and recovery
- Implement demo mode
- Polish UI

**Test**: Full demo - "Pick up the red cube" → Robot executes complete task

---

## Estimated Time

**Total**: 2-3 hours (each phase independently testable)

## Demo Scenario

When your capstone is complete, you'll demonstrate:

```
1. [Voice] Say: "Describe what you see"
   → Robot: "I see a red cube on the left and a blue cylinder ahead"

2. [Vision] Camera feed displays with object labels overlaid

3. [Command] Say: "Pick up the red cube"
   → Robot plans: [turn_left, move_forward(0.3), grasp]

4. [Safety] System validates: ✓ No collisions, ✓ Speed OK, ✓ In bounds

5. [Execution] Robot executes actions smoothly in Gazebo

6. [Success] Gripper closes around red cube, lifts it
```

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────┐
│                  User Interface (React)                  │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │  Voice   │  │  Vision  │  │  Control │             │
│  │   UI     │  │  Overlay │  │  Panel   │             │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘             │
└───────┼─────────────┼─────────────┼───────────────────┘
        │             │             │
        │     REST API + WebSocket  │
        │             │             │
┌───────┼─────────────┼─────────────┼───────────────────┐
│       │     FastAPI Backend       │                    │
│       │             │             │                    │
│  ┌────▼─────┐  ┌───▼──────┐  ┌──▼────────┐          │
│  │ Whisper  │  │ GPT-4    │  │  Safety   │          │
│  │ Client   │  │ Vision   │  │ Validator │          │
│  └──────────┘  └──────────┘  └───────────┘          │
│       │             │             │                    │
│       └─────────────┴─────────────┤                    │
│                                   │                    │
│                          ┌────────▼────────┐          │
│                          │  GPT-4 Planner  │          │
│                          └────────┬────────┘          │
│                                   │                    │
│                          ┌────────▼────────┐          │
│                          │  ROS 2 Bridge   │          │
│                          └────────┬────────┘          │
└──────────────────────────────────┼────────────────────┘
                                   │
                                   │ ROS 2 Topics
                                   │
┌──────────────────────────────────┼────────────────────┐
│                Gazebo Simulation  │                    │
│         ┌─────────────────────────▼─────────┐         │
│         │  Humanoid Robot + Sensors         │         │
│         │  - Camera (640x480 RGB)           │         │
│         │  - Odometry                       │         │
│         │  - Gripper                        │         │
│         └───────────────────────────────────┘         │
└─────────────────────────────────────────────────────┘
```

## Success Criteria

Your capstone is complete when:

- ✅ All 6 phases independently functional
- ✅ Voice commands transcribed accurately (>90%)
- ✅ Vision detects objects correctly
- ✅ LLM generates valid action plans
- ✅ Safety validator blocks 100% of unsafe commands
- ✅ Robot executes actions smoothly in Gazebo
- ✅ Full pipeline works end-to-end
- ✅ Demo completable in under 90 seconds

## Getting Started

Ready to build? Start with **Phase 1: Voice Input Module**!

_Phase guides coming soon._
