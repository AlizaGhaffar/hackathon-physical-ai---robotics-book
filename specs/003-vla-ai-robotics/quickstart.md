# Quickstart Guide: Chapter 3 Development

**Feature**: Vision-Language-Action: AI Meets Robotics
**Audience**: Developers implementing Chapter 3
**Est. Time**: 2-3 weeks (13 days per constitution)

## Prerequisites

- [x] Chapters 1 & 2 complete (ROS 2 + Gazebo foundation)
- [x] OpenAI API key with access to Whisper, GPT-4, GPT-4 Vision
- [x] Qdrant Cloud account (free tier)
- [x] Neon Postgres database (from Chapters 1-2)
- [x] Docker installed (for code sandbox)
- [x] Gazebo Classic 11 running smoothly

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                      Browser (React + Docusaurus)           │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │ Voice UI │  │ Vision   │  │ Code Gen │  │ Capstone │  │
│  │(Whisper) │  │(GPT-4V)  │  │(GPT-4)   │  │ Progress │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
│         │            │              │              │        │
│         └────────────┴──────────────┴──────────────┘        │
│                         │                                    │
│                    WebSocket + REST                          │
└────────────────────────┼────────────────────────────────────┘
                         │
┌────────────────────────┼────────────────────────────────────┐
│               FastAPI Backend (Python)                       │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐  │
│  │ Whisper  │  │ GPT-4    │  │ Safety   │  │ ROS 2    │  │
│  │ Client   │  │ Client   │  │ Validator│  │ Bridge   │  │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘  │
│         │            │              │              │        │
│         └────────────┴──────────────┴──────────────┘        │
│                         │                      │             │
│                   Neon Postgres          rclpy (ROS 2)       │
└─────────────────────────────────────────────┼───────────────┘
                                               │
┌──────────────────────────────────────────────┼───────────────┐
│                    ROS 2 Network             │               │
│                                              │               │
│  ┌──────────────────────────────────────────▼─────────┐    │
│  │           Gazebo Simulation (Robot + Sensors)      │    │
│  └──────────────────────────────────────────┬─────────┘    │
│                                              │               │
│                         Camera Feed, State Updates          │
└─────────────────────────────────────────────────────────────┘
```

## Development Workflow (13-Day Plan)

### Week 1: Core AI Integration (Days 1-7)

**Day 1-2: Voice Control Foundation**
- [ ] Implement browser audio capture (`MediaRecorder` API)
- [ ] Create FastAPI `/voice/transcribe` endpoint
- [ ] Integrate OpenAI Whisper API client
- [ ] Build voice UI component (microphone button, waveform visualization)
- [ ] Test end-to-end: audio → Whisper → display transcription

**Day 3-4: Vision Integration**
- [ ] Capture Gazebo camera feed (ROS 2 Image topic subscriber)
- [ ] Create FastAPI `/vision/analyze` endpoint
- [ ] Integrate GPT-4 Vision API client
- [ ] Implement 30-second caching (MD5 hash-based)
- [ ] Build vision UI (image display, object labels overlay)
- [ ] Test: Gazebo scene → GPT-4V → object list

**Day 5-6: LLM Command Processing**
- [ ] Create FastAPI `/robot/command` endpoint
- [ ] Implement GPT-4 prompt engineering (system prompt with safety rules)
- [ ] Build safety validation layer (collision check, speed limits, workspace bounds)
- [ ] Create ROS 2 action publisher (`geometry_msgs/Twist`, `control_msgs/GripperCommand`)
- [ ] Test: "Move forward 2m" → validated action → Gazebo execution

**Day 7: Integration & Testing**
- [ ] Connect voice → LLM → robot (full VLA pipeline)
- [ ] Build WebSocket for real-time robot state streaming
- [ ] Test voice commands end-to-end: speak → transcribe → plan → execute
- [ ] Fix integration bugs
- [ ] Record demo video (backup for live demo)

### Week 2: Features & Capstone (Days 8-13)

**Day 8-9: Code Generation**
- [ ] Create FastAPI `/code/generate` endpoint
- [ ] Implement code validation (syntax check, ROS 2 imports)
- [ ] Integrate Monaco Editor in frontend
- [ ] Build code execution sandbox (Docker container with 30s timeout)
- [ ] Test: "Create publisher node" → working Python code → execute in Gazebo

**Day 10-11: Capstone Project**
- [ ] Design 6-phase capstone structure (see spec.md User Story 4)
- [ ] Create capstone progress tracker UI
- [ ] Write step-by-step capstone guide (markdown content)
- [ ] Build capstone APIs (`/capstone/project`, `/capstone/project/phases/{n}/complete`)
- [ ] Test: Walk through all 6 phases, verify each can be completed independently

**Day 12: Playground & Polish**
- [ ] Build LLM-robot playground UI (prompt editor, parameter sliders)
- [ ] Implement playground APIs (`/playground/experiments`, `/playground/experiments/{id}/run`)
- [ ] Add Framer Motion animations (voice pulsing, code streaming, robot smooth movement)
- [ ] Polish error messages (3-tier graceful degradation)
- [ ] Mobile responsive testing

**Day 13: Documentation & Deployment**
- [ ] Write Chapter 3 markdown content (intro, concepts, examples, exercises)
- [ ] Populate Qdrant vector store with Chapter 3 embeddings
- [ ] Test RAG chatbot with 20 Chapter 3 questions
- [ ] Run full quality gate checklist (constitution.md)
- [ ] Rehearse 90-second demo (time it, smooth transitions)
- [ ] Deploy to production (GitHub Pages / Vercel)

## Key Implementation Notes

### Voice Control (Whisper API)

**Frontend Audio Capture**:
```typescript
const mediaRecorder = new MediaRecorder(stream, { mimeType: 'audio/webm' });
mediaRecorder.ondataavailable = async (event) => {
  const formData = new FormData();
  formData.append('audio_file', event.data, 'voice_command.webm');

  const response = await fetch('/api/v1/voice/transcribe', {
    method: 'POST',
    body: formData
  });

  const { transcribed_text } = await response.json();
  setTranscription(transcribed_text);
};
```

**Backend Whisper Integration**:
```python
import openai

@app.post("/api/v1/voice/transcribe")
async def transcribe_voice(audio_file: UploadFile):
    audio_bytes = await audio_file.read()

    response = openai.Audio.transcribe(
        model="whisper-1",
        file=audio_bytes,
        response_format="json"
    )

    return {
        "transcribed_text": response["text"],
        "confidence_score": 0.95,  # Whisper doesn't return confidence
        "language_detected": response.get("language", "en")
    }
```

### Safety Validation Layer

**Critical: Always validate LLM outputs before execution**

```python
class SafetyValidator:
    def validate_action(self, action: RobotAction, robot_state: RobotState) -> ValidationResult:
        """
        Safety checks (reject if any fail):
        1. Collision: obstacle <0.5m in movement direction
        2. Speed: speed >1.0 m/s navigation, >0.3 m/s manipulation
        3. Workspace: position outside [-5,5]m x [-5,5]m bounds
        4. Battery: <15% for high-power actions
        5. Gripper: grasp when holding, release when empty
        """
        if action.action_type == "move":
            if robot_state.nearest_obstacle_distance < 0.5:
                return ValidationResult(
                    valid=False,
                    reason="Collision risk: obstacle 0.3m ahead (need >0.5m clearance)"
                )
            if action.parameters["speed"] > 1.0:
                return ValidationResult(
                    valid=False,
                    reason="Speed limit exceeded: 0.8 m/s requested (max 1.0 m/s)"
                )

        return ValidationResult(valid=True, reason="All safety checks passed")
```

### GPT-4 Vision Prompt Engineering

```python
def build_vision_prompt(image_b64: str, query: str = "list all objects") -> dict:
    return {
        "model": "gpt-4-vision-preview",
        "messages": [
            {
                "role": "system",
                "content": """You are analyzing a robot camera feed from Gazebo simulation.

                OUTPUT FORMAT (JSON):
                {
                  "objects": [
                    {"label": "red_cube", "position": "30cm left", "confidence": 0.9},
                    {"label": "blue_cylinder", "position": "ahead 50cm", "confidence": 0.85"}
                  ],
                  "spatial_relations": [
                    {"obj1": "red_cube", "relation": "left_of", "obj2": "blue_cylinder"}
                  ]
                }
                """
            },
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": query},
                    {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{image_b64}"}}
                ]
            }
        ],
        "max_tokens": 500
    }
```

### Code Execution Sandbox

**Run user code in isolated Docker container**:

```python
import docker
import asyncio

async def execute_user_code(code: str, timeout: int = 30) -> ExecutionResult:
    client = docker.from_env()

    container = client.containers.run(
        image="ros:humble-ros-base",
        command=["python3", "-c", code],
        detach=True,
        network_mode="ros2_network",  # Access to ROS 2 topics
        mem_limit="512m",
        cpu_quota=100000,  # 1 CPU core
        remove=True
    )

    try:
        result = await asyncio.wait_for(
            container.wait(timeout=timeout),
            timeout=timeout
        )

        stdout = container.logs(stdout=True, stderr=False).decode()
        stderr = container.logs(stdout=False, stderr=True).decode()

        return ExecutionResult(
            success=(result["StatusCode"] == 0),
            stdout=stdout,
            stderr=stderr,
            timeout_reached=False
        )
    except asyncio.TimeoutError:
        container.kill()
        return ExecutionResult(
            success=False,
            stdout="",
            stderr="Execution timeout (30s limit)",
            timeout_reached=True
        )
```

### Capstone Phase Structure

**6 Modular Phases** (each 20-30 minutes):

| Phase | Module | Test | Time |
|-------|--------|------|------|
| 1 | Voice Input | Echo transcription | 20min |
| 2 | Vision Analysis | Scene description | 25min |
| 3 | Task Planner | Multi-step sequence | 30min |
| 4 | Safety Validator | Reject unsafe commands | 20min |
| 5 | ROS 2 Executor | Robot moves in Gazebo | 30min |
| 6 | Integration | Voice→Vision→Plan→Execute | 30min |

**Learner Experience**:
- Complete Phase 1 → immediate success (voice echo works!)
- Complete Phase 2 → see vision working (robot "sees" objects)
- Phase 3-5 → build up complexity gradually
- Phase 6 → "magic moment" when everything connects

## Testing Strategy

### Unit Tests (pytest)
```python
def test_whisper_transcription():
    audio_file = load_test_audio("move_forward.webm")
    result = transcribe_voice(audio_file)
    assert "forward" in result.transcribed_text.lower()

def test_safety_validator_rejects_collision():
    action = RobotAction(action_type="move", parameters={"distance": 2.0})
    robot_state = RobotState(nearest_obstacle_distance=0.3)  # Too close!
    result = validator.validate_action(action, robot_state)
    assert result.valid == False
    assert "collision" in result.reason.lower()
```

### Integration Tests
```python
async def test_voice_to_robot_pipeline():
    # 1. Upload audio
    response = await client.post("/voice/transcribe", files={"audio_file": audio})
    transcription = response.json()["transcribed_text"]

    # 2. Send command
    response = await client.post("/robot/command", json={"command": transcription})
    actions = response.json()["actions"]
    assert len(actions) > 0
    assert actions[0]["safety_validated"] == True

    # 3. Wait for execution
    await asyncio.sleep(5)
    status = await client.get(f"/robot/actions/{actions[0]['action_id']}/status")
    assert status.json()["status"] == "completed"
```

### End-to-End Test (Capstone)
```python
def test_capstone_full_walkthrough():
    # Start project
    project = create_capstone_project()

    # Complete all 6 phases
    for phase in range(1, 7):
        complete_phase(project.id, phase, test_passed=True)

    # Verify completion
    final_project = get_capstone_project(project.id)
    assert final_project.progress_percentage == 100.0
    assert final_project.demo_ready == True
```

## Common Issues & Solutions

### Issue: Whisper API Slow (>5 seconds)

**Solution**:
- Check audio file size (<1MB recommended)
- Use `response_format="json"` not "verbose_json"
- Consider parallel processing (start transcription, continue with other tasks)

### Issue: GPT-4 Vision Hallucinating Objects

**Solution**:
- Cross-validate with simple color detection (HSV thresholding)
- Add to system prompt: "Only list objects you are confident about (>80%)"
- Use temperature=0 for more deterministic responses

### Issue: Gazebo Crashes During Demo

**Solution**:
- Use lightweight worlds (max 20 objects)
- Disable Gazebo GUI shadows (performance)
- Have backup Three.js animated robot ready
- Pre-record video as ultimate fallback

### Issue: Code Generation Produces Invalid ROS 2 Code

**Solution**:
- Use temperature=0 for code generation (deterministic)
- Include few-shot examples in system prompt
- Validate with `ast.parse()` before returning to user
- Provide "Debug with AI" button if execution fails

## Quality Gates (Before Deployment)

- [ ] All 6 user stories (P1) functional
- [ ] RAG chatbot answers 85%+ of Chapter 3 questions correctly
- [ ] Capstone project completable in 2-3 hours
- [ ] 90-second demo rehearsed (smooth, no delays)
- [ ] All code examples executable (copy-paste-run works)
- [ ] Safety validator blocks 100% of dangerous commands (test set)
- [ ] Visual consistency with Chapters 1-2 (same UI components)
- [ ] Mobile responsive (tablets 768px+)
- [ ] No console errors in browser DevTools
- [ ] Documentation complete (Chapter 3 content + capstone guide)

## Resources

- OpenAI API Docs: https://platform.openai.com/docs/api-reference
- ROS 2 Humble Docs: https://docs.ros.org/en/humble/
- Gazebo Classic Tutorials: http://gazebosim.org/tutorials
- Monaco Editor React: https://github.com/suren-atoyan/monaco-react
- Framer Motion: https://www.framer.com/motion/
- Qdrant Hybrid Search: https://qdrant.tech/documentation/guides/hybrid-search/

## Next Steps

After completing this quickstart:
1. Review `specs/003-vla-ai-robotics/plan.md` for detailed architecture
2. Follow 13-day timeline in `constitution.md` (Chapter 3 workflow)
3. Run `/sp.tasks` to generate implementation tasks
4. Begin Day 1: Voice Control Foundation

**Good luck building the climax chapter!** 🚀
