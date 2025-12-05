# Implementation Tasks: Vision-Language-Action: AI Meets Robotics (Chapter 3)

**Feature**: Vision-Language-Action: AI Meets Robotics
**Branch**: `003-vla-ai-robotics`
**Created**: 2025-12-03
**Total Tasks**: 67 tasks across 9 phases
**Estimated Duration**: 13 days (per constitution 13-day workflow)

---

## Task Organization

Tasks are organized by user story to enable independent implementation and testing. Each P1 user story can be developed in parallel after foundational tasks complete.

**User Story Priorities** (from spec.md):
- **P1 (Critical for Demo)**: US1 (Voice Control), US2 (Vision Manipulation), US4 (Capstone)
- **P2 (Important)**: US3 (Code Generation), US5 (Playground)
- **P3 (Nice-to-Have)**: US6 (Multi-Modal RAG)

---

## Implementation Strategy

**MVP Scope**: User Story 1 (Voice-Controlled Robot Navigation)
- Delivers end-to-end VLA pipeline: Voice → LLM → Safety → Robot
- Independently testable: Speak command → robot moves in Gazebo
- ~2-3 days of work after foundational tasks

**Incremental Delivery**:
1. **Sprint 1** (Days 1-3): Setup + Foundational + US1
2. **Sprint 2** (Days 4-6): US2 (Vision) + US4 Phase 1-3 (Capstone foundations)
3. **Sprint 3** (Days 7-10): US3 (Code Gen) + US4 Phase 4-6 (Capstone completion)
4. **Sprint 4** (Days 11-13): US5 (Playground) + Polish + Demo prep

---

## Phase 1: Project Setup & Infrastructure

**Goal**: Initialize project structure, configure dependencies, set up databases

**Duration**: 0.5 days (4 hours)

**Tasks**:

- [ ] T001 Create backend/ directory structure per plan.md (models, services, api/routes, utils)
- [ ] T002 Create frontend/ directory structure per plan.md (components/chapter3, hooks, services)
- [ ] T003 Create docs/chapter-3/ directory structure with subdirectories (voice-control, vision, task-planning, examples, exercises)
- [ ] T004 [P] Create backend/requirements.txt with pinned versions (FastAPI 0.109+, OpenAI SDK 1.3+, rclpy, SQLAlchemy, psycopg2-binary, redis)
- [ ] T005 [P] Create frontend/package.json with dependencies (Monaco Editor 0.45+, Framer Motion 11.0+, axios)
- [ ] T006 [P] Create docker/code_sandbox.Dockerfile for user code execution (ROS 2 Humble base image)
- [ ] T007 [P] Create docker/docker-compose.yml for full stack (backend, postgres, redis, gazebo)
- [ ] T008 Configure Neon Postgres connection in backend/.env.example (DATABASE_URL, connection pool settings)
- [ ] T009 Configure Qdrant Cloud connection in backend/.env.example (QDRANT_URL, QDRANT_API_KEY, collection names)
- [ ] T010 [P] Configure OpenAI API keys in backend/.env.example (OPENAI_API_KEY, model names, rate limits)
- [ ] T011 Create backend/src/database.py with SQLAlchemy engine and session management
- [ ] T012 [P] Create backend/src/config.py with Pydantic settings validation

---

## Phase 2: Foundational Components (Blocking Prerequisites)

**Goal**: Build shared infrastructure needed by all user stories

**Duration**: 1 day

**Dependencies**: Phase 1 complete

**Tasks**:

### Database Models

- [ ] T013 [P] Create backend/src/models/voice_command.py with VoiceCommand model per data-model.md (audio_data, transcribed_text, confidence_score, status enum)
- [ ] T014 [P] Create backend/src/models/visual_scene.py with VisualScene model (camera_image, scene_description, detected_objects JSONB, cache_key)
- [ ] T015 [P] Create backend/src/models/llm_query.py with LLMQuery model (query_type enum, prompt_text, system_context, response_text, tokens_used)
- [ ] T016 [P] Create backend/src/models/robot_action.py with RobotAction model (action_type enum, parameters JSONB, safety_validated, execution_order, status enum)
- [ ] T017 [P] Create backend/src/models/task_plan.py with TaskPlan model (goal_description, action_sequence JSONB, current_step, replanning_count, status enum)
- [ ] T018 [P] Create backend/src/models/generated_code.py with GeneratedCode model (natural_language_input, generated_code, validation_errors JSONB, execution_tested)
- [ ] T019 [P] Create backend/src/models/capstone_project.py with CapstoneProject model (project_title, task_plan_id FK, completion_steps JSONB, current_phase, progress_percentage, demo_ready)
- [ ] T020 [P] Create backend/src/models/playground_experiment.py with PlaygroundExperiment model (experiment_name, system_prompt, parameter_settings JSONB, scenario_type enum, run_count)

### Database Migrations

- [ ] T021 Create Alembic migration for all Chapter 3 tables with indexes per data-model.md

### OpenAI API Clients

- [ ] T022 [P] Create backend/src/services/whisper_client.py with transcribe_audio() function (handles WebM upload, returns TranscriptionResponse)
- [ ] T023 [P] Create backend/src/services/gpt4_client.py with query_llm() function (system prompt, temperature control, token counting, streaming support)
- [ ] T024 [P] Create backend/src/services/gpt4_vision_client.py with analyze_image() function (base64 image input, structured JSON output for detected objects)

### ROS 2 Integration

- [ ] T025 Create backend/src/services/ros2_bridge.py with rclpy node initialization (geometry_msgs/Twist publisher, robot state subscriber, action client)
- [ ] T026 Implement publish_robot_action() in ros2_bridge.py (converts RobotAction model to ROS 2 messages, publishes to /cmd_vel or gripper topics)
- [ ] T027 Implement get_robot_state() in ros2_bridge.py (subscribes to /odom, /joint_states, /scan, returns RobotState dataclass)

### Shared Services

- [ ] T028 [P] Create backend/src/services/safety_validator.py with validate_action() implementing 5 safety rules per research.md (collision check, speed limit, workspace bounds, battery, gripper state)
- [ ] T029 [P] Create backend/src/utils/cache.py with Redis caching helpers (get_cached_transcription, cache_scene_analysis with 30s TTL)
- [ ] T030 [P] Create backend/src/utils/error_handlers.py with 3-tier error handling (retry, cache fallback, manual fallback) and user-friendly error messages

### Frontend Shared Components

- [ ] T031 [P] Create frontend/src/hooks/useRobotState.ts with WebSocket connection to backend for real-time robot state updates
- [ ] T032 [P] Create frontend/src/services/api.ts with typed Axios client from OpenAPI spec (all 30+ endpoints typed)
- [ ] T033 [P] Create frontend/src/components/chapter3/SafetyIndicator.tsx with real-time safety status display (green/yellow/red based on robot state)

---

## Phase 3: User Story 1 - Voice-Controlled Robot Navigation [US1] [P1]

**Story Goal**: Learners speak natural language commands to control robot in Gazebo via Whisper → GPT-4 → ROS 2 pipeline

**Independent Test**: Launch Gazebo, speak "Move forward 2 meters", verify robot moves forward 2m

**Duration**: 2 days

**Dependencies**: Phase 2 complete

**Acceptance Criteria** (from spec.md):
1. Voice command "Move forward 3 meters" → Whisper transcribes → LLM generates command → robot moves 3m
2. Multi-step command "Turn right 90 degrees and move to red cube" → LLM plans sequence → robot executes both actions
3. Unclear command "Go over there" → LLM requests clarification
4. Unsafe command "Drive off platform" → safety validator rejects with explanation

### Backend Tasks

- [ ] T034 [US1] Create backend/src/api/routes/voice.py with POST /voice/transcribe endpoint per contracts/api-specification.yaml
- [ ] T035 [US1] Implement handle_voice_transcription() in voice.py (accepts multipart/form-data audio, calls whisper_client, saves VoiceCommand to DB, returns transcription)
- [ ] T036 [US1] Add audio file validation in voice.py (check size <5MB, duration 1-60s, format WebM)
- [ ] T037 [US1] Implement transcription caching in voice.py (check Redis cache by audio MD5 hash, cache for 5 minutes)
- [ ] T038 [US1] Create backend/src/api/routes/robot.py with POST /robot/command endpoint
- [ ] T039 [US1] Implement handle_robot_command() in robot.py (accepts natural language command, creates LLMQuery, calls gpt4_client with safety system prompt from research.md)
- [ ] T040 [US1] Implement command_to_actions() in robot.py (parses GPT-4 JSON response into RobotAction objects with action_type and parameters JSONB)
- [ ] T041 [US1] Integrate safety_validator in robot.py (validate each action before execution, reject if safety_validated=False, return user-friendly explanation)
- [ ] T042 [US1] Implement execute_validated_actions() in robot.py (calls ros2_bridge.publish_robot_action for each validated action, updates action status in DB)
- [ ] T043 [US1] Create GET /robot/actions/{action_id}/status endpoint in robot.py for action progress polling

### Frontend Tasks

- [ ] T044 [P] [US1] Create frontend/src/hooks/useVoiceRecorder.ts with MediaRecorder API wrapper (start/stop recording, WebM output, error handling for unsupported browsers)
- [ ] T045 [US1] Create frontend/src/components/chapter3/VoiceCommandUI.tsx with microphone button (pulsing animation during recording, waveform visualization, transcription display)
- [ ] T046 [US1] Implement voice command submission in VoiceCommandUI.tsx (upload audio to /voice/transcribe, display transcription, send to /robot/command on user confirmation)
- [ ] T047 [US1] Add Framer Motion animations to VoiceCommandUI.tsx (pulsing mic icon, fade-in transcription, smooth state transitions)
- [ ] T048 [US1] Create frontend/src/components/chapter3/RobotControlPanel.tsx with manual control fallback (text input for commands, action status display, safety indicator integration)
- [ ] T049 [US1] Implement WebSocket subscription in RobotControlPanel.tsx (real-time robot position updates, action execution progress, error messages)

### Integration & Testing

- [ ] T050 [US1] Create backend/tests/integration/test_voice_to_robot_pipeline.py with end-to-end test (upload audio → transcribe → command → validate → execute → verify robot moved)
- [ ] T051 [US1] Test US1 acceptance criterion 1: "Move forward 3 meters" full pipeline
- [ ] T052 [US1] Test US1 acceptance criterion 2: Multi-step command "Turn right 90 degrees and move to red cube"
- [ ] T053 [US1] Test US1 acceptance criterion 3: Unclear command "Go over there" → clarification requested
- [ ] T054 [US1] Test US1 acceptance criterion 4: Unsafe command "Drive off platform" → rejected by safety validator

---

## Phase 4: User Story 2 - Vision-Based Object Manipulation [US2] [P1]

**Story Goal**: Learners use GPT-4 Vision to identify objects in Gazebo and command robot to interact via natural language

**Independent Test**: Load Gazebo world with colored objects, ask "What objects do you see?", receive GPT-4 Vision description, command "Pick up blue cylinder", verify robot grasps it

**Duration**: 2 days

**Dependencies**: Phase 2 complete (can run parallel with US1 after foundational tasks)

**Acceptance Criteria**:
1. "Describe what you see" → GPT-4 Vision analyzes Gazebo camera → lists 5 objects with colors/positions
2. "Grasp the red cube" → vision identifies cube → LLM plans grasp → robot picks up cube
3. "Pick up the leftmost green block" → system disambiguates with spatial reasoning → correct grasp
4. Obstructed object → system reports obstruction and suggests alternatives

### Backend Tasks

- [ ] T055 [P] [US2] Create backend/src/api/routes/vision.py with POST /vision/analyze endpoint per contracts/api-specification.yaml
- [ ] T056 [US2] Implement handle_vision_analysis() in vision.py (accepts multipart image, calls gpt4_vision_client with scene understanding system prompt, saves VisualScene to DB)
- [ ] T057 [US2] Implement vision result caching in vision.py (MD5 hash of image as cache_key, return cached result if <30 seconds old, reduces API costs by 70%)
- [ ] T058 [US2] Add image validation in vision.py (check size <500KB, format JPEG/PNG, resolution 640x480 recommended)
- [ ] T059 [US2] Extend handle_robot_command() in robot.py to accept visual_scene_id parameter (link LLMQuery to VisualScene for spatial context)
- [ ] T060 [US2] Update GPT-4 system prompt in robot.py to include detected objects from VisualScene (enables spatial reasoning like "leftmost green block")
- [ ] T061 [US2] Implement grasp action type in ros2_bridge.py (publish control_msgs/GripperCommand, pre-grasp positioning, approach direction from vision data)

### Frontend Tasks

- [ ] T062 [P] [US2] Create frontend/src/components/chapter3/VisionOverlay.tsx with Gazebo camera feed display (overlays object labels from GPT-4 Vision, bounding boxes if coordinates available)
- [ ] T063 [US2] Implement vision query button in VisionOverlay.tsx (capture current Gazebo frame, send to /vision/analyze, display scene_description)
- [ ] T064 [US2] Add Framer Motion animations to VisionOverlay.tsx (fade-in object labels, highlight selected object on hover, smooth camera feed updates)
- [ ] T065 [US2] Integrate VisionOverlay with VoiceCommandUI (when voice command mentions object, highlight in vision overlay, e.g., "red cube" → red cube highlighted)

### Gazebo Integration

- [ ] T066 [US2] Create Gazebo test world in ros2_workspace/worlds/manipulation_test.world with 5 colored objects (red cube, blue cylinder, green blocks x3, table surface)
- [ ] T067 [US2] Add camera plugin to Gazebo robot model (publishes to /camera/image_raw topic, 640x480 resolution, 30 FPS)
- [ ] T068 [US2] Add gripper plugin to Gazebo robot model (MimicJoint for parallel gripper, effort control, collision detection)

### Integration & Testing

- [ ] T069 [US2] Create backend/tests/integration/test_vision_pipeline.py with tests for GPT-4 Vision accuracy (load test images, verify object detection >80% accuracy)
- [ ] T070 [US2] Test US2 acceptance criterion 1: "Describe what you see" → lists 5 objects correctly
- [ ] T071 [US2] Test US2 acceptance criterion 2: "Grasp the red cube" → successful pick-up
- [ ] T072 [US2] Test US2 acceptance criterion 3: "Leftmost green block" → spatial disambiguation works
- [ ] T073 [US2] Test US2 acceptance criterion 4: Obstructed object → reports obstruction

---

## Phase 5: User Story 3 - AI-Assisted Code Generation [US3] [P2]

**Story Goal**: Learners describe robot behavior in natural language, receive executable ROS 2 Python code, run in interactive Monaco Editor

**Independent Test**: Type "Create a ROS 2 publisher that sends Twist messages at 10 Hz", receive Python code, click "Run Code", verify robot moves in square pattern

**Duration**: 1.5 days

**Dependencies**: Phase 2 complete (can run parallel with US1/US2)

**Acceptance Criteria**:
1. "Publisher that sends Twist messages at 10 Hz" → complete ROS 2 node with proper imports, timer callback
2. Generated code contains errors → user clicks "Debug with AI" → receives explanation + corrected code
3. "Add safety stop if obstacle detected" → LLM inserts LaserScan subscriber logic
4. Generated code runs in interactive editor → executes in live Gazebo instance

### Backend Tasks

- [ ] T074 [P] [US3] Create backend/src/api/routes/code.py with POST /code/generate endpoint per contracts/api-specification.yaml
- [ ] T075 [US3] Implement handle_code_generation() in code.py (accepts natural_language_input, calls gpt4_client with temperature=0 for deterministic code, returns GeneratedCode model)
- [ ] T076 [US3] Implement code validation in code.py (syntax check with ast.parse(), ROS 2 import validation, common anti-pattern detection, populate validation_errors JSONB)
- [ ] T077 [US3] Create POST /code/debug endpoint in code.py (accepts code + error_message, calls GPT-4 for debugging assistance, returns corrected code + explanation)
- [ ] T078 [US3] Create POST /code/execute endpoint in code.py (accepts Python code, spawns Docker container with ROS 2 Humble base, 30s timeout, returns stdout/stderr)
- [ ] T079 [US3] Implement Docker sandbox execution in backend/src/services/code_executor.py (isolate with 512MB RAM limit, 1 CPU core, connect to ros2_network for topic access, kill on timeout)

### Frontend Tasks

- [ ] T080 [P] [US3] Create frontend/src/components/chapter3/CodeEditor.tsx with Monaco Editor integration (Python syntax highlighting, ROS 2 autocompletion, dark theme matching Docusaurus)
- [ ] T081 [US3] Implement code generation UI in CodeEditor.tsx (natural language input box, "Generate Code" button, display generated code in Monaco Editor)
- [ ] T082 [US3] Add "Run Code" button to CodeEditor.tsx (sends code to /code/execute, displays output in terminal pane below editor, streams execution progress)
- [ ] T083 [US3] Add "Debug with AI" button to CodeEditor.tsx (triggered on execution errors, sends code + error to /code/debug, displays AI explanation, offers to replace with corrected code)
- [ ] T084 [US3] Implement code streaming in CodeEditor.tsx (GPT-4 streaming response, typing effect as code appears character-by-character, Framer Motion smooth scrolling)
- [ ] T085 [US3] Add code execution status indicators (loading spinner during execution, success/error icons, execution time display)

### Integration & Testing

- [ ] T086 [US3] Create backend/tests/integration/test_code_generation.py with validation tests (5 common ROS 2 patterns, verify 90%+ generate valid Python)
- [ ] T087 [US3] Test US3 acceptance criterion 1: "Publisher at 10 Hz" → compiles and runs
- [ ] T088 [US3] Test US3 acceptance criterion 2: Introduce syntax error → debug with AI → fixed
- [ ] T089 [US3] Test US3 acceptance criterion 3: "Add safety stop" → LaserScan subscriber added correctly
- [ ] T090 [US3] Test US3 acceptance criterion 4: Code runs in editor → Gazebo robot responds

---

## Phase 6: User Story 4 (Part 1) - Capstone Project Foundation [US4] [P1]

**Story Goal**: Set up capstone project infrastructure with 6 modular phases, enable phase 1-3 completion

**Independent Test**: Create new capstone project, complete Phase 1 (Voice Input Module), verify test passes, progress updates to 16.67%

**Duration**: 1 day

**Dependencies**: US1, US2, US3 complete (uses components from all 3 stories)

**Acceptance Criteria** (Phases 1-3 only for Part 1):
- Phase 1: Voice input module echoes transcription
- Phase 2: Vision module describes Gazebo scene
- Phase 3: Task planner generates multi-step action sequences

### Backend Tasks

- [ ] T091 [US4] Create backend/src/api/routes/capstone.py with GET /capstone/project and POST /capstone/project endpoints per contracts/api-specification.yaml
- [ ] T092 [US4] Implement create_capstone_project() in capstone.py (one project per user, initializes 6 phases in completion_steps JSONB, default project_title="Autonomous Humanoid Assistant")
- [ ] T093 [US4] Implement get_capstone_project() in capstone.py (returns current project with phases[], current_phase, progress_percentage calculated as completed_phases/6 * 100)
- [ ] T094 [US4] Create POST /capstone/project/phases/{phase_number}/complete endpoint in capstone.py (marks phase complete if test_passed=True, increments current_phase, recalculates progress_percentage)
- [ ] T095 [US4] Implement capstone phase validation in capstone.py (Phase N+1 cannot be completed before Phase N, test_passed must be True for completion)

### Frontend Tasks

- [ ] T096 [P] [US4] Create frontend/src/components/chapter3/CapstoneProgress.tsx with 6-phase visual progress tracker (phase cards, completion checkmarks, current phase highlighted, progress bar)
- [ ] T097 [US4] Implement phase detail view in CapstoneProgress.tsx (click phase → shows phase instructions, test criteria, "Mark Complete" button)
- [ ] T098 [US4] Add phase navigation in CapstoneProgress.tsx (Previous/Next buttons, disabled if previous phases incomplete, mobile responsive accordion view)

### Content Tasks

- [ ] T099 [P] [US4] Create docs/chapter-3/exercises/capstone-project.md with overview (what is autonomous humanoid assistant, why capstone matters, 2-3 hour time estimate)
- [ ] T100 [US4] Write Phase 1 guide in capstone-project.md (Voice Input Module: integrate Whisper, echo transcription, test: speak → see text)
- [ ] T101 [US4] Write Phase 2 guide in capstone-project.md (Vision Module: integrate GPT-4 Vision, describe scene, test: ask "what do you see?" → get description)
- [ ] T102 [US4] Write Phase 3 guide in capstone-project.md (Task Planner: integrate GPT-4 for multi-step planning, test: "clean up workspace" → get action sequence)

### Integration & Testing

- [ ] T103 [US4] Create backend/tests/e2e/test_capstone_walkthrough.py with automated test for Phases 1-3 (create project, complete each phase, verify progression)
- [ ] T104 [US4] Test capstone Phase 1 completion: Voice module works, test passes, progress = 16.67%
- [ ] T105 [US4] Test capstone Phase 2 completion: Vision module works, progress = 33.33%
- [ ] T106 [US4] Test capstone Phase 3 completion: Task planner works, progress = 50.0%

---

## Phase 7: User Story 4 (Part 2) - Capstone Project Completion [US4] [P1]

**Story Goal**: Complete capstone Phases 4-6, achieve full autonomous humanoid assistant integration

**Independent Test**: Complete all 6 phases, progress = 100%, demo_ready = True, run end-to-end test: "Bring me the water bottle from the table"

**Duration**: 1.5 days

**Dependencies**: Phase 6 complete (US4 Part 1)

**Acceptance Criteria**:
- Phase 4: Safety validator rejects unsafe commands
- Phase 5: ROS 2 executor publishes commands to Gazebo
- Phase 6: Full integration works (voice → vision → plan → validate → execute)
- End-to-end capstone test: Multi-step task completes successfully (identify object → grasp → deliver)

### Content Tasks

- [ ] T107 [P] [US4] Write Phase 4 guide in capstone-project.md (Safety Validator: integrate 5 safety rules, test: unsafe command → rejected with explanation)
- [ ] T108 [P] [US4] Write Phase 5 guide in capstone-project.md (ROS 2 Executor: publish Twist/GripperCommand messages, test: command → robot moves in Gazebo)
- [ ] T109 [P] [US4] Write Phase 6 guide in capstone-project.md (Full Integration: connect all modules, test: "Bring water bottle" → complete task autonomously)
- [ ] T110 [US4] Add troubleshooting section to capstone-project.md (top 10 common issues: Gazebo crash, Whisper slow, GPT-4 invalid command, gripper fails, etc.)

### ROS 2 Integration Tasks

- [ ] T111 [US4] Create ros2_workspace/src/vla_control/launch/capstone.launch.py with full system launch (Gazebo world, robot model, all ROS 2 nodes)
- [ ] T112 [US4] Create ros2_workspace/src/vla_control/config/safety_params.yaml with configurable safety thresholds (min_obstacle_distance, max_speed, workspace_bounds)
- [ ] T113 [P] [US4] Create ros2_workspace/src/vla_control/scripts/voice_listener.py starter code (subscribes to transcription topic, forwards to LLM)
- [ ] T114 [P] [US4] Create ros2_workspace/src/vla_control/scripts/vision_processor.py starter code (processes camera images, forwards to GPT-4 Vision)
- [ ] T115 [P] [US4] Create ros2_workspace/src/vla_control/scripts/action_executor.py starter code (executes validated robot actions)

### Integration & Testing

- [ ] T116 [US4] Extend backend/tests/e2e/test_capstone_walkthrough.py for Phases 4-6
- [ ] T117 [US4] Test capstone Phase 4: Safety validator rejects collision risk, progress = 66.67%
- [ ] T118 [US4] Test capstone Phase 5: ROS 2 commands execute in Gazebo, progress = 83.33%
- [ ] T119 [US4] Test capstone Phase 6: Full pipeline integration, progress = 100%, demo_ready = True
- [ ] T120 [US4] Test end-to-end capstone: "Bring me the water bottle from the table" → robot autonomously identifies, grasps, delivers (SUCCESS = capstone validated)

---

## Phase 8: User Story 5 - Interactive LLM-Robot Playground [US5] [P2]

**Story Goal**: Learners experiment with prompt engineering in safe sandbox, tune safety parameters, export configurations

**Independent Test**: Create experiment "Cautious Navigation", adjust safety slider to 0.9, issue test command, observe conservative behavior, export as ROS 2 launch file

**Duration**: 1 day

**Dependencies**: Phase 2 complete (can run parallel with US1-US4, but typically implemented after core features)

**Acceptance Criteria**:
1. Playground UI with scenario selection (navigation, manipulation, exploration)
2. Safety conservativeness slider → affects LLM behavior
3. Custom system prompt → alters robot "personality"
4. "Export to Code" → generates reproducible ROS 2 launch file

### Backend Tasks

- [ ] T121 [P] [US5] Create backend/src/api/routes/playground.py with GET /playground/experiments and POST /playground/experiments endpoints
- [ ] T122 [US5] Implement create_experiment() in playground.py (saves PlaygroundExperiment model with experiment_name, system_prompt, parameter_settings JSONB, scenario_type)
- [ ] T123 [US5] Create POST /playground/experiments/{experiment_id}/run endpoint (runs test command with experiment parameters, returns LLM reasoning + generated actions without executing)
- [ ] T124 [US5] Create GET /playground/experiments/{experiment_id}/export endpoint (generates Python ROS 2 launch file with embedded system prompt and parameters)
- [ ] T125 [US5] Implement parameter override logic in playground.py (applies llm_temperature, safety_conservativeness, max_speed_override to LLM query)

### Frontend Tasks

- [ ] T126 [P] [US5] Create frontend/src/components/chapter3/PlaygroundUI.tsx with experiment creation form (experiment name input, scenario dropdown, system prompt textarea)
- [ ] T127 [US5] Add parameter sliders to PlaygroundUI.tsx (LLM temperature 0.0-2.0, safety conservativeness 0.0-1.0, max speed override 0.0-1.0, verbose reasoning toggle)
- [ ] T128 [US5] Implement test command panel in PlaygroundUI.tsx (command input, "Run Experiment" button, displays LLM reasoning + planned actions side-by-side)
- [ ] T129 [US5] Add experiment comparison view in PlaygroundUI.tsx (select 2 experiments, show behavioral differences for same command)
- [ ] T130 [US5] Implement export functionality in PlaygroundUI.tsx ("Export to Code" button, downloads .launch.py file, shows instructions for local execution)

### Content Tasks

- [ ] T131 [P] [US5] Create docs/chapter-3/exercises/exercise-03-prompt-engineering.md with playground tutorial (create cautious vs aggressive experiments, compare behaviors)

### Integration & Testing

- [ ] T132 [US5] Test US5 acceptance criterion 1: Create experiment with scenario_type="navigation"
- [ ] T133 [US5] Test US5 acceptance criterion 2: Adjust safety slider → LLM rejects riskier commands
- [ ] T134 [US5] Test US5 acceptance criterion 3: Custom system prompt "You are a very polite robot" → responses change tone
- [ ] T135 [US5] Test US5 acceptance criterion 4: Export generates valid launch file, runs locally

---

## Phase 9: Polish, Documentation & Demo Preparation

**Goal**: Final quality checks, documentation, demo video, deployment

**Duration**: 1.5 days

**Dependencies**: All P1 user stories complete (US1, US2, US4)

**Tasks**:

### Multi-Modal RAG Enhancement (US6 - P3, optional if time permits)

- [ ] T136 [P] [US6] Enhance backend/src/services/qdrant_client.py to support image embeddings (CLIP model for diagrams/screenshots, hybrid search with 70% dense + 30% sparse)
- [ ] T137 [P] [US6] Update RAG chatbot frontend (from Ch1-2) to accept image uploads (user uploads screenshot of Gazebo error, chatbot analyzes and suggests fix)

### Chapter 3 Content Creation

- [ ] T138 [P] Create docs/chapter-3/intro.md (chapter overview, what you'll learn, why VLA matters for robotics)
- [ ] T139 [P] Create docs/chapter-3/what-is-vla.md (Vision-Language-Action pipeline explanation, diagram, real-world examples)
- [ ] T140 [P] Create docs/chapter-3/voice-control/whisper-integration.md (how Whisper works, integration guide, code examples)
- [ ] T141 [P] Create docs/chapter-3/vision/gpt4-vision.md (GPT-4 Vision capabilities, scene understanding examples)
- [ ] T142 [P] Create docs/chapter-3/task-planning/llm-based-planning.md (LLM task decomposition, multi-step execution)
- [ ] T143 [P] Create docs/chapter-3/troubleshooting.md (common issues: API rate limits, Gazebo crashes, voice recognition failures, solutions from research.md)
- [ ] T144 [P] Create 5 code example files in docs/chapter-3/examples/ (example-01 to example-05 per spec.md, each with working Python code + explanation)

### Qdrant Vector Store Population

- [ ] T145 Populate Qdrant Cloud with Chapter 3 content embeddings (50 sections + 20 code examples + 15 diagrams = 85 vectors, use OpenAI text-embedding-3-large)
- [ ] T146 Test RAG chatbot with 20 Chapter 3 questions (verify >85% accuracy, adjust retrieval parameters if needed)

### Visual Consistency & UX Polish

- [ ] T147 [P] Add Framer Motion animations to all Chapter 3 components (fade-ins, smooth transitions, 60fps performance validated)
- [ ] T148 Validate visual consistency with Chapters 1-2 (same colors, fonts, spacing, UI component styles)
- [ ] T149 Test mobile responsiveness (tablets 768px+, all Chapter 3 pages render correctly, touch interactions work)
- [ ] T150 Test browser compatibility (Chrome, Edge, Firefox latest 2 versions, Safari macOS)

### Demo Preparation

- [ ] T151 Write 90-second demo script (0-15s voice command, 15-30s vision analysis, 30-45s task planning, 45-60s execution, 60-75s code generation, 75-90s capstone highlight)
- [ ] T152 Rehearse live demo 5 times (time each segment, identify stutter points, smooth transitions)
- [ ] T153 Record pre-recorded fallback demo video (1080p60fps, OBS Studio, clear audio narration, text overlays, professional quality)
- [ ] T154 Create demo backup scenarios (if Whisper fails → use text input, if Gazebo crashes → switch to Three.js animation, if API rate limit → show cached responses)

### Testing & Quality Gates

- [ ] T155 Run full quality gate checklist from plan.md (11 items, all must pass before deployment)
- [ ] T156 Create test set of 20 unsafe commands (collision courses, speed violations, out-of-bounds), verify safety validator blocks 100%
- [ ] T157 Test all 5 code examples (copy from docs, paste in editor, run, verify robot behavior matches description)
- [ ] T158 Validate capstone completable in 2-3 hours (recruit 3 test learners, time them, identify friction points, simplify if >3 hours)

### Deployment

- [ ] T159 Create deployment checklist (environment variables set, database migrations run, Qdrant collections created, API keys configured)
- [ ] T160 Deploy backend to production (Vercel/Railway, connect to Neon Postgres, verify health endpoint)
- [ ] T161 Deploy frontend to GitHub Pages/Vercel (build Docusaurus, configure base URL, test all routes)
- [ ] T162 Add Chapter 3 card to homepage (frontend/src/pages/index.tsx, color=#FF6B6B or appropriate theme color, links to /chapter-3/intro)
- [ ] T163 Test production deployment (all P1 features functional, API rate limits appropriate, no console errors)

### Documentation Updates

- [ ] T164 [P] Update README.md with Chapter 3 setup instructions (OpenAI API keys, Qdrant setup, ROS 2 workspace optional for capstone)
- [ ] T165 [P] Create DEMO-SCRIPT.md with detailed 90-second presentation guide (what to say, which buttons to click, timing cues)
- [ ] T166 [P] Update main constitution.md if any principles were adjusted during implementation (document changes with rationale)

---

## Dependency Graph

**Sequential Dependencies** (must complete in order):

```
Phase 1 (Setup)
  ↓
Phase 2 (Foundational)
  ↓
Phase 3-5-8 (US1, US3, US5 - Can run in parallel)
  ↓
Phase 4 (US2 - Depends on Phase 2, can run parallel with Phase 3)
  ↓
Phase 6-7 (US4 Part 1-2 - Depends on US1, US2, US3)
  ↓
Phase 9 (Polish - Depends on all P1 stories)
```

**Critical Path** (blocking for demo):
Phase 1 → Phase 2 → Phase 3 (US1) → Phase 4 (US2) → Phase 6 (US4 Part 1) → Phase 7 (US4 Part 2) → Phase 9 (Demo Prep)

**Estimated Timeline**: 13 days
- Day 1: Setup (Phase 1) + Foundational start (Phase 2)
- Day 2: Foundational complete + US1 start (Phase 3)
- Day 3: US1 complete (Phase 3)
- Day 4-5: US2 (Phase 4) parallel with US3 start (Phase 5)
- Day 6: US3 complete (Phase 5) + US4 Part 1 start (Phase 6)
- Day 7: US4 Part 1 complete (Phase 6)
- Day 8-9: US4 Part 2 (Phase 7)
- Day 10: US5 (Phase 8) + Polish start (Phase 9)
- Day 11-12: Polish continue (content creation, testing, demo prep)
- Day 13: Final deployment, demo rehearsal, quality gates

---

## Parallel Execution Opportunities

**High Parallelizability** (marked with [P], different files/modules):

- Phase 1 Setup: T004-T007, T010 (backend deps, frontend deps, Docker files, config templates)
- Phase 2 Models: T013-T020 (8 SQLAlchemy models, all independent)
- Phase 2 Clients: T022-T024, T028-T030, T031-T033 (API clients, utils, frontend hooks)
- Phase 9 Content: T138-T144 (Chapter 3 markdown files, all independent)

**Example Parallel Task Groups**:

**Group A** (Backend API routes after Phase 2):
- T034-T043 (US1 voice + robot routes)
- T055-T061 (US2 vision routes)
- T074-T079 (US3 code generation routes)
Can assign 3 developers, one per story

**Group B** (Frontend components after Phase 2):
- T044-T049 (US1 voice UI)
- T062-T065 (US2 vision overlay)
- T080-T085 (US3 code editor)
Can assign 3 developers, one per story

**Group C** (Content creation in Phase 9):
- T138-T142 (conceptual docs)
- T143-T144 (troubleshooting + examples)
- T151-T154 (demo preparation)
Can assign 2 technical writers + 1 demo specialist

---

## MVP Scope (Minimum Viable Product)

**MVP = User Story 1 Complete**

Delivers core value proposition: **Voice → LLM → Robot** pipeline

**Included in MVP**:
- Phase 1: Setup (T001-T012)
- Phase 2: Foundational (T013-T033)
- Phase 3: US1 Voice Control (T034-T054)

**MVP Delivers**:
- Learner speaks "Move forward 2 meters"
- Whisper transcribes audio
- GPT-4 generates robot command
- Safety validator checks constraints
- Robot moves forward 2m in Gazebo
- **Total Demo Time**: 15-20 seconds (perfect for quick validation)

**MVP Estimated Time**: 3-4 days (after full setup)

**Why This MVP**:
- Demonstrates complete VLA pipeline
- Proves AI-robotics integration works
- Independently testable
- Provides foundation for US2-US6

---

## Task Summary

**Total Tasks**: 166 tasks
**P0 (MVP - US1)**: 45 tasks
**P1 (Critical for Demo - US1, US2, US4)**: 120 tasks
**P2 (Important - US3, US5)**: 34 tasks
**P3 (Nice-to-Have - US6)**: 2 tasks (multi-modal RAG enhancement)

**Task Breakdown by Phase**:
- Phase 1 (Setup): 12 tasks
- Phase 2 (Foundational): 21 tasks
- Phase 3 (US1): 21 tasks
- Phase 4 (US2): 19 tasks
- Phase 5 (US3): 17 tasks
- Phase 6 (US4 Part 1): 16 tasks
- Phase 7 (US4 Part 2): 14 tasks
- Phase 8 (US5): 15 tasks
- Phase 9 (Polish): 31 tasks

**Parallelizable Tasks**: 68 tasks (41% can run in parallel after dependencies met)

---

## Next Steps

1. **Begin Implementation**: Start with Phase 1 (Setup) - T001 to T012
2. **Follow 13-Day Timeline**: Allocate tasks per day according to dependency graph
3. **Test Incrementally**: Complete US1 → test end-to-end before moving to US2
4. **Track Progress**: Update task checkboxes as completed, monitor against quality gates
5. **Demo Early**: Record rough demo after US1 complete to identify issues early

**Ready to implement Chapter 3!** 🚀
