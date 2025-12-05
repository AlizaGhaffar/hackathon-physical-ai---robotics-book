# Data Model: Chapter 3 VLA System

**Feature**: Vision-Language-Action: AI Meets Robotics
**Date**: 2025-12-03
**Status**: Phase 1 Complete

## Entity Relationship Overview

```
User (from Ch1-2)
  ↓ 1:N
VoiceCommand → LLMQuery → RobotAction → TaskPlan
  ↓ 1:1        ↓ 1:N         ↓ 1:1
Transcription  GeneratedCode  ExecutionResult

VisualScene → LLMQuery
  ↓ N:M
DetectedObject

CapstoneProject → TaskPlan
  ↓ 1:N          ↓ 1:N
ProjectModule   RobotAction

PlaygroundExperiment
  ↓ 1:N
ExperimentRun → RobotAction
```

---

## Core Entities

### 1. VoiceCommand

**Purpose**: Represents user's spoken input captured via microphone and processed through Whisper API

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK, auto-generated | Unique identifier |
| `user_id` | UUID | FK → User, NOT NULL | Owner of command |
| `audio_data` | BYTEA | NOT NULL, <5MB | Raw audio (WebM format) |
| `audio_duration` | DECIMAL(5,2) | NOT NULL, >0, <60 | Duration in seconds |
| `transcribed_text` | VARCHAR(500) | NULLABLE (pending transcription) | Whisper output |
| `confidence_score` | DECIMAL(3,2) | NULLABLE, 0.0-1.0 | Whisper confidence |
| `language_detected` | VARCHAR(10) | NULLABLE, ISO 639-1 code | Auto-detected language |
| `created_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Recording timestamp |
| `transcribed_at` | TIMESTAMP | NULLABLE | When Whisper completed |
| `status` | ENUM | recording, processing, completed, failed | Processing state |

**Relationships**:
- `user_id` → `users.id` (many-to-one)
- `id` ← `llm_queries.voice_command_id` (one-to-many)

**State Transitions**:
```
recording → processing → completed
         ↓            ↓
         ↓           failed
```

**Validation Rules**:
- Audio duration must be between 1-60 seconds (enforce in app, not DB)
- If status=completed, transcribed_text and confidence_score must NOT be NULL
- Audio data stored temporarily (delete after 24 hours for privacy)

---

### 2. VisualScene

**Purpose**: Represents GPT-4 Vision's understanding of robot's camera feed from Gazebo

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique identifier |
| `user_id` | UUID | FK → User, NOT NULL | Owner (for cache/quota tracking) |
| `camera_image` | BYTEA | NOT NULL, <500KB | JPEG image (base64) |
| `image_resolution` | VARCHAR(20) | NOT NULL | E.g., "640x480" |
| `scene_description` | TEXT | NULLABLE | GPT-4V natural language output |
| `detected_objects` | JSONB | NULLABLE | Array of detected objects (see DetectedObject) |
| `spatial_relations` | JSONB | NULLABLE | E.g., [{"obj1": "red_cube", "relation": "left_of", "obj2": "blue_cylinder"}] |
| `created_at` | TIMESTAMP | NOT NULL, DEFAULT NOW() | Capture timestamp |
| `analyzed_at` | TIMESTAMP | NULLABLE | When GPT-4V completed |
| `cache_key` | VARCHAR(64) | NULLABLE, MD5 hash | For caching identical scenes |
| `status` | ENUM | captured, analyzing, completed, failed | Processing state |

**Relationships**:
- `user_id` → `users.id`
- `id` ← `detected_objects.scene_id` (one-to-many)
- `id` ← `llm_queries.visual_scene_id` (one-to-many)

**JSONB Structure for detected_objects**:
```json
[
  {
    "object_id": "obj_1",
    "label": "red_cube",
    "confidence": 0.92,
    "position_description": "30cm to robot's left",
    "attributes": {"color": "red", "shape": "cube", "size": "small"}
  },
  {
    "object_id": "obj_2",
    "label": "blue_cylinder",
    "confidence": 0.88,
    "position_description": "directly ahead, 50cm away",
    "attributes": {"color": "blue", "shape": "cylinder", "size": "medium"}
  }
]
```

**Caching Strategy**:
- Compute MD5 hash of image_data → cache_key
- If cache_key exists and created_at <30 seconds ago, return cached result
- Reduces API costs by 70% (simulation scenes change slowly)

---

### 3. LLMQuery

**Purpose**: Represents natural language prompt sent to GPT-4 for robot control, code generation, or planning

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique identifier |
| `user_id` | UUID | FK → User, NOT NULL | Query owner |
| `query_type` | ENUM | command, code_gen, debug, planning | Purpose of query |
| `prompt_text` | TEXT | NOT NULL | User's natural language input |
| `system_context` | TEXT | NOT NULL | System prompt with robot state |
| `voice_command_id` | UUID | FK → VoiceCommand, NULLABLE | If triggered by voice |
| `visual_scene_id` | UUID | FK → VisualScene, NULLABLE | If using vision context |
| `model_used` | VARCHAR(50) | NOT NULL | E.g., "gpt-4", "gpt-4-1106-preview" |
| `temperature` | DECIMAL(2,1) | NOT NULL, 0.0-2.0 | Model temperature setting |
| `max_tokens` | INTEGER | NOT NULL, 100-4000 | Token limit for response |
| `response_text` | TEXT | NULLABLE | LLM output |
| `tokens_used` | INTEGER | NULLABLE | Total tokens (prompt + completion) |
| `execution_time_ms` | INTEGER | NULLABLE | API latency |
| `created_at` | TIMESTAMP | NOT NULL | Query timestamp |
| `completed_at` | TIMESTAMP | NULLABLE | Response received timestamp |
| `status` | ENUM | queued, processing, completed, failed | Query state |
| `error_message` | TEXT | NULLABLE | If status=failed |

**Relationships**:
- `user_id` → `users.id`
- `voice_command_id` → `voice_commands.id` (optional)
- `visual_scene_id` → `visual_scenes.id` (optional)
- `id` ← `robot_actions.llm_query_id` (one-to-many)
- `id` ← `generated_code.llm_query_id` (one-to-one)

**Example prompt_text for Different query_types**:

**command**:
```
"Move forward 2 meters and pick up the red cube"
```

**code_gen**:
```
"Create a ROS 2 publisher node that sends Twist messages at 10 Hz to make the robot drive in a square"
```

**debug**:
```
"This code gives AttributeError: 'NoneType' object has no attribute 'publish'. How do I fix it?"
```

**planning**:
```
"Plan a sequence of actions to clean up the workspace: identify objects, pick up each, place in container"
```

---

### 4. RobotAction

**Purpose**: Represents atomic or composite action to be executed by robot in Gazebo

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique identifier |
| `llm_query_id` | UUID | FK → LLMQuery, NOT NULL | Source LLM decision |
| `task_plan_id` | UUID | FK → TaskPlan, NULLABLE | Parent plan (if part of sequence) |
| `action_type` | ENUM | move, rotate, grasp, release, wait | Type of action |
| `parameters` | JSONB | NOT NULL | Action-specific params |
| `safety_validated` | BOOLEAN | NOT NULL, DEFAULT FALSE | Passed safety checks |
| `validation_notes` | TEXT | NULLABLE | Safety validator output |
| `execution_order` | INTEGER | NULLABLE | Sequence position in plan |
| `created_at` | TIMESTAMP | NOT NULL | Action created |
| `started_at` | TIMESTAMP | NULLABLE | Execution started |
| `completed_at` | TIMESTAMP | NULLABLE | Execution finished |
| `status` | ENUM | pending, validated, executing, completed, failed | Execution state |
| `error_message` | TEXT | NULLABLE | Failure reason |

**Relationships**:
- `llm_query_id` → `llm_queries.id`
- `task_plan_id` → `task_plans.id` (optional, for multi-step tasks)
- `id` ← `execution_results.action_id` (one-to-one)

**JSONB parameters by action_type**:

**move**:
```json
{
  "distance": 2.0,        // meters
  "direction": "forward", // forward, backward, left, right
  "speed": 0.5,          // m/s
  "duration": 4.0        // seconds (calculated)
}
```

**rotate**:
```json
{
  "angle": 90.0,         // degrees (positive = counterclockwise)
  "speed": 0.3,          // rad/s
  "duration": 5.2        // seconds (calculated)
}
```

**grasp**:
```json
{
  "target_object": "red_cube",
  "approach_direction": "top_down",
  "gripper_force": 0.7,  // 0.0-1.0 (normalized)
  "pre_grasp_distance": 0.1  // meters (hover before grasp)
}
```

**release**:
```json
{
  "gripper_open_amount": 1.0  // 0.0-1.0 (fully open)
}
```

**Safety Validation Rules** (safety_validated = TRUE only if all pass):
1. **Collision Check**: No obstacles within 0.5m in movement direction
2. **Speed Limit**: speed ≤ 1.0 m/s for navigation, ≤0.3 m/s for manipulation
3. **Workspace Bounds**: Resulting position within [-5, 5]m x [-5, 5]m
4. **Battery Check**: Remaining battery ≥ 15% for high-power actions
5. **Gripper State**: If action=grasp, gripper must be empty; if release, must be holding

---

### 5. TaskPlan

**Purpose**: Represents LLM-generated multi-step plan for complex behaviors (capstone project)

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique identifier |
| `user_id` | UUID | FK → User, NOT NULL | Plan owner |
| `llm_query_id` | UUID | FK → LLMQuery, NOT NULL | Planning query that generated this |
| `goal_description` | TEXT | NOT NULL | High-level task (e.g., "Clean up workspace") |
| `action_sequence` | JSONB | NOT NULL | Ordered list of action IDs |
| `current_step` | INTEGER | NOT NULL, DEFAULT 0 | Index of current action |
| `completion_percentage` | DECIMAL(5,2) | NOT NULL, DEFAULT 0.0 | 0.0-100.0% |
| `replanning_count` | INTEGER | NOT NULL, DEFAULT 0 | Times plan was regenerated |
| `created_at` | TIMESTAMP | NOT NULL | Plan generation time |
| `started_at` | TIMESTAMP | NULLABLE | Execution started |
| `completed_at` | TIMESTAMP | NULLABLE | All actions done |
| `status` | ENUM | draft, executing, paused, completed, failed | Plan state |

**Relationships**:
- `user_id` → `users.id`
- `llm_query_id` → `llm_queries.id`
- `id` ← `robot_actions.task_plan_id` (one-to-many)
- `id` ← `capstone_projects.task_plan_id` (one-to-one)

**action_sequence JSONB structure**:
```json
{
  "actions": [
    {"action_id": "uuid-1", "description": "Scan workspace for objects"},
    {"action_id": "uuid-2", "description": "Move to red cube"},
    {"action_id": "uuid-3", "description": "Grasp red cube"},
    {"action_id": "uuid-4", "description": "Move to container"},
    {"action_id": "uuid-5", "description": "Release cube into container"}
  ],
  "dependencies": [
    {"action": "uuid-3", "requires": ["uuid-1", "uuid-2"]}
  ],
  "failure_recovery": {
    "uuid-3": {
      "on_failure": "replan_from_uuid-2",
      "retry_count": 2
    }
  }
}
```

**State Machine**:
```
draft → executing → completed
        ↓         ↓
       paused    failed
        ↓
       executing (resume or replan)
```

---

### 6. GeneratedCode

**Purpose**: Represents AI-generated ROS 2 code from natural language description

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique identifier |
| `llm_query_id` | UUID | FK → LLMQuery, UNIQUE, NOT NULL | Generation request |
| `natural_language_input` | TEXT | NOT NULL | User's code description |
| `generated_code` | TEXT | NOT NULL | Python code output |
| `language` | VARCHAR(20) | NOT NULL, DEFAULT 'python' | Programming language |
| `code_type` | ENUM | node, launch_file, service, action | ROS 2 code type |
| `validation_errors` | JSONB | NULLABLE | Syntax/lint errors found |
| `execution_tested` | BOOLEAN | NOT NULL, DEFAULT FALSE | Ran in sandbox |
| `execution_result` | TEXT | NULLABLE | Output/errors from test run |
| `created_at` | TIMESTAMP | NOT NULL | Code generation time |
| `validated_at` | TIMESTAMP | NULLABLE | Validation completed |

**Relationships**:
- `llm_query_id` → `llm_queries.id` (one-to-one)

**validation_errors JSONB structure**:
```json
{
  "syntax_errors": [
    {"line": 15, "error": "IndentationError: unexpected indent"}
  ],
  "import_errors": [
    {"line": 3, "error": "ImportError: No module named 'geometry_msgs'"}
  ],
  "lint_warnings": [
    {"line": 22, "warning": "W0612: Unused variable 'result'"}
  ]
}
```

---

### 7. CapstoneProject

**Purpose**: Represents learner's final project integrating all Chapter 3 concepts

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique identifier |
| `user_id` | UUID | FK → User, UNIQUE, NOT NULL | Project owner (1 capstone per user) |
| `project_title` | VARCHAR(200) | NOT NULL, DEFAULT 'Autonomous Humanoid Assistant' | Project name |
| `task_plan_id` | UUID | FK → TaskPlan, NULLABLE | Main execution plan |
| `completion_steps` | JSONB | NOT NULL | Phase completion tracking |
| `current_phase` | INTEGER | NOT NULL, DEFAULT 1 | Current phase (1-6) |
| `progress_percentage` | DECIMAL(5,2) | NOT NULL, DEFAULT 0.0 | Overall completion |
| `test_results` | JSONB | NULLABLE | Unit/integration test outcomes |
| `demo_video_url` | VARCHAR(500) | NULLABLE | Recorded demo (optional) |
| `demo_ready` | BOOLEAN | NOT NULL, DEFAULT FALSE | Ready for presentation |
| `created_at` | TIMESTAMP | NOT NULL | Project start time |
| `last_activity_at` | TIMESTAMP | NOT NULL | Last progress update |
| `completed_at` | TIMESTAMP | NULLABLE | All phases done |
| `status` | ENUM | in_progress, paused, completed, abandoned | Project state |

**Relationships**:
- `user_id` → `users.id` (one-to-one, each user has max 1 capstone)
- `task_plan_id` → `task_plans.id` (optional)

**completion_steps JSONB structure**:
```json
{
  "phases": [
    {
      "phase_number": 1,
      "phase_name": "Voice Input Module",
      "status": "completed",
      "completed_at": "2025-12-03T14:30:00Z",
      "test_passed": true
    },
    {
      "phase_number": 2,
      "phase_name": "Vision Module",
      "status": "in_progress",
      "started_at": "2025-12-03T15:00:00Z",
      "test_passed": false
    },
    {
      "phase_number": 3,
      "phase_name": "Task Planner",
      "status": "pending"
    }
  ]
}
```

**Progress Calculation**:
```
progress_percentage = (completed_phases / total_phases) * 100
completed_phases = count(phases where status='completed' AND test_passed=true)
```

---

### 8. PlaygroundExperiment

**Purpose**: Represents user's prompt engineering experiment in LLM-robot playground

**Fields**:
| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique identifier |
| `user_id` | UUID | FK → User, NOT NULL | Experiment owner |
| `experiment_name` | VARCHAR(200) | NOT NULL | User-defined name |
| `system_prompt` | TEXT | NOT NULL | Custom LLM system prompt |
| `parameter_settings` | JSONB | NOT NULL | Temperature, safety, etc. |
| `scenario_type` | ENUM | navigation, manipulation, exploration | Preset scenario |
| `created_at` | TIMESTAMP | NOT NULL | Experiment created |
| `last_run_at` | TIMESTAMP | NULLABLE | Most recent test |
| `run_count` | INTEGER | NOT NULL, DEFAULT 0 | Times executed |
| `exported_config` | TEXT | NULLABLE | ROS 2 launch file (if exported) |

**Relationships**:
- `user_id` → `users.id`
- `id` ← `experiment_runs.experiment_id` (one-to-many)

**parameter_settings JSONB**:
```json
{
  "llm_temperature": 0.3,
  "safety_conservativeness": 0.8,  // 0.0 (aggressive) - 1.0 (cautious)
  "max_speed_override": 0.7,       // Max allowed speed
  "verbose_reasoning": true,        // Show LLM thinking
  "allow_replanning": true         // Auto-replan on failure
}
```

---

## Supporting Entities (Lightweight)

### 9. ExecutionResult

**Purpose**: Stores outcome of robot action execution

| Field | Type | Description |
|-------|------|-------------|
| `action_id` | UUID | FK → RobotAction, PK |
| `success` | BOOLEAN | Action completed successfully |
| `actual_duration_ms` | INTEGER | Time to execute |
| `final_robot_pose` | JSONB | `{x, y, z, roll, pitch, yaw}` |
| `sensor_data` | JSONB | Camera/LiDAR readings during action |
| `error_details` | TEXT | Failure reason (if success=false) |

### 10. DetectedObject

**Purpose**: Individual object identified in VisualScene

| Field | Type | Description |
|-------|------|-------------|
| `id` | UUID | PK |
| `scene_id` | UUID | FK → VisualScene |
| `object_id` | VARCHAR(50) | E.g., "obj_1" |
| `label` | VARCHAR(100) | E.g., "red_cube" |
| `confidence` | DECIMAL(3,2) | 0.0-1.0 |
| `position_description` | TEXT | "30cm to left" |
| `attributes` | JSONB | `{color, shape, size}` |

---

## Database Indexes

**Performance-Critical Indexes**:

```sql
-- Voice Commands
CREATE INDEX idx_voice_commands_user_created ON voice_commands(user_id, created_at DESC);
CREATE INDEX idx_voice_commands_status ON voice_commands(status) WHERE status IN ('processing', 'recording');

-- Visual Scenes (cache lookups)
CREATE INDEX idx_visual_scenes_cache_key ON visual_scenes(cache_key, created_at DESC);

-- LLM Queries (user history, analytics)
CREATE INDEX idx_llm_queries_user_type ON llm_queries(user_id, query_type, created_at DESC);
CREATE INDEX idx_llm_queries_status ON llm_queries(status) WHERE status IN ('queued', 'processing');

-- Robot Actions (task plan execution)
CREATE INDEX idx_robot_actions_task_plan ON robot_actions(task_plan_id, execution_order);
CREATE INDEX idx_robot_actions_status ON robot_actions(status) WHERE status NOT IN ('completed', 'failed');

-- Capstone Projects
CREATE INDEX idx_capstone_projects_user ON capstone_projects(user_id);
CREATE UNIQUE INDEX idx_capstone_projects_user_unique ON capstone_projects(user_id) WHERE status != 'abandoned';
```

---

## Data Retention Policies

| Entity | Retention Period | Rationale |
|--------|------------------|-----------|
| VoiceCommand.audio_data | 24 hours | Privacy (delete after transcription) |
| VisualScene.camera_image | 7 days | Cache efficiency, demo replay |
| LLMQuery (all) | Indefinite | Learning analytics, user history |
| RobotAction (completed) | 30 days | Debugging, demo replay, then archive |
| GeneratedCode | Indefinite | User's code library |
| CapstoneProject | Indefinite | Portfolio evidence |
| PlaygroundExperiment | Indefinite | User's saved experiments |

---

## Phase 1 Data Model Completion Checklist

- [x] Core entities defined (8 primary entities)
- [x] Relationships mapped (ERD overview provided)
- [x] Field constraints specified (types, NULLability, defaults)
- [x] JSONB structures documented (for flexible data)
- [x] State machines defined (status transitions)
- [x] Validation rules documented (business logic)
- [x] Indexes specified (performance optimization)
- [x] Data retention policies defined (privacy, storage)

**Status**: Ready for API Contracts ✅
