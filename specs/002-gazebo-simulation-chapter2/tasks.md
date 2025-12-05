# Tasks: Gazebo Simulation Chapter 2

**Input**: Design documents from `/specs/002-gazebo-simulation-chapter2/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/

**Tests**: Not requested in spec - manual testing only per constitution

**Organization**: Tasks grouped by user story to enable independent implementation and testing of each story

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend (Docusaurus)**: `docs/chapter-2/`, `src/components/`, `sidebars.ts`
- **Backend (FastAPI)**: `backend/src/`, `backend/src/scripts/`
- **Config**: `docs/chapters-config.json`, `.env`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create Chapter 2 directory structure at `docs/chapter-2/` with subdirectories: intro, urdf-sdf, physics, sensors, examples, exercises
- [ ] T002 [P] Create chapter metadata config file at `docs/chapters-config.json` with Chapter 1 and Chapter 2 entries per data-model.md schema
- [ ] T003 [P] Add Chart.js dependency to `package.json`: `"chart.js": "^4.4.0"`, `"react-chartjs-2": "^5.2.0"`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T004 Update `sidebars.ts` to add Chapter 2 category with nested subcategories for urdf-sdf, physics, sensors, examples, exercises
- [ ] T005 [P] Create Qdrant collection `chapter_2_embeddings` by adding `create_chapter_2_collection()` function to `backend/src/vector_store.py`
- [ ] T006 [P] Verify Chapter 1 components (PersonalizeButton, TranslateButton, RAGChatbot) accept `chapterId` prop - if not implemented, document interface requirements in `src/components/README.md`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Access and Navigate to Chapter 2 (Priority: P1) 🎯 MVP

**Goal**: Enable users to navigate to Chapter 2 and see all bonus features functional

**Independent Test**: Navigate from homepage → Chapter 2, verify page loads with title "Gazebo Simulation: Creating Digital Twins", confirm PersonalizeButton, TranslateButton, ChatBot are present (even if not fully functional yet)

### Implementation for User Story 1

- [ ] T007 [P] [US1] Create Chapter 2 landing page at `docs/chapter-2/intro.md` with title, learning objectives, and chapter overview
- [ ] T008 [P] [US1] Add PersonalizeButton component to `docs/chapter-2/intro.md` with prop `chapterId="chapter-2"`
- [ ] T009 [P] [US1] Add TranslateButton component to `docs/chapter-2/intro.md` with prop `chapterId="chapter-2"`
- [ ] T010 [P] [US1] Add RAGChatbot component to `docs/chapter-2/intro.md` with prop `chapterId="chapter-2"`
- [ ] T011 [US1] Test navigation from homepage to Chapter 2 - verify URL `/docs/chapter-2/intro` loads correctly
- [ ] T012 [US1] Test direct Chapter 2 URL access - verify deep linking works
- [ ] T013 [US1] Test authenticated user navigation - verify session persists when switching from Chapter 1 to Chapter 2

**Checkpoint**: At this point, User Story 1 is complete - Chapter 2 is accessible with all bonus feature components present

---

## Phase 4: User Story 2 - Learn Gazebo Basics and URDF/SDF (Priority: P2)

**Goal**: Provide core educational content about Gazebo simulation and robot description formats

**Independent Test**: Read through all sections, verify content completeness, check code examples render with syntax highlighting, ask RAG chatbot Chapter 2 questions

### Content Creation for User Story 2

- [ ] T014 [P] [US2] Write `docs/chapter-2/what-is-gazebo.md` covering: Gazebo overview, why simulation matters, ROS 2 integration, installation guide for Ubuntu/Windows
- [ ] T015 [P] [US2] Write `docs/chapter-2/urdf-sdf/urdf-basics.md` covering: URDF structure, XML tags (robot, link, joint), code examples with syntax highlighting
- [ ] T016 [P] [US2] Write `docs/chapter-2/urdf-sdf/sdf-basics.md` covering: SDF structure, world definitions, model definitions, plugin system
- [ ] T017 [P] [US2] Write `docs/chapter-2/urdf-sdf/urdf-vs-sdf.md` covering: comparison table, when to use each, conversion between formats
- [ ] T018 [P] [US2] Create URDF code example at `docs/chapter-2/examples/simple-robot.md` with downloadable URDF file and RobotViewer component (`chapterId="chapter-2"`)
- [ ] T019 [P] [US2] Add PersonalizeButton and TranslateButton to all URDF/SDF section pages
- [ ] T020 [US2] Create embedding script at `backend/src/scripts/embed_chapter2.py` to chunk and embed all Chapter 2 markdown files
- [ ] T021 [US2] Run embedding script to populate `chapter_2_embeddings` Qdrant collection (execute: `python backend/src/scripts/embed_chapter2.py`)
- [ ] T022 [US2] Test RAG chatbot with 5 sample Chapter 2 questions: "What's the difference between URDF and SDF?", "How do I install Gazebo?", "What's a <link> tag in URDF?", "What are SDF plugins?", "How do I describe a robot joint?"
- [ ] T023 [US2] Verify personalization works - test with software background user, check for code-focused analogies in Gazebo content
- [ ] T024 [US2] Verify translation works - test Urdu translation on URDF section, confirm technical terms (URDF, SDF, Gazebo) preserved

**Checkpoint**: At this point, User Story 2 is complete - Gazebo basics and URDF/SDF content is available with all bonus features functional

---

## Phase 5: User Story 3 - Explore Physics and Sensor Simulation (Priority: P3)

**Goal**: Teach advanced simulation concepts (physics engines, sensor modeling)

**Independent Test**: Read physics and sensor sections, interact with PhysicsParameterSlider and SensorVisualization components, ask RAG chatbot about physics parameters and sensor configuration

### Content Creation for User Story 3

- [ ] T025 [P] [US3] Write `docs/chapter-2/physics/physics-engines.md` covering: ODE vs Bullet vs Simbody, when to use each, performance tradeoffs
- [ ] T026 [P] [US3] Write `docs/chapter-2/physics/gravity-friction.md` covering: gravity configuration, friction coefficients, surface properties, bouncing/restitution
- [ ] T027 [P] [US3] Write `docs/chapter-2/physics/collisions.md` covering: collision detection, contact forces, collision geometry
- [ ] T028 [P] [US3] Write `docs/chapter-2/sensors/cameras.md` covering: camera sensor types, resolution, field of view, camera plugins for ROS 2
- [ ] T029 [P] [US3] Write `docs/chapter-2/sensors/lidar.md` covering: ray-based sensors, scan parameters (samples, range, angles), LiDAR plugins
- [ ] T030 [P] [US3] Write `docs/chapter-2/sensors/imu.md` covering: inertial measurement units, acceleration/gyroscope data, noise models
- [ ] T031 [P] [US3] Write `docs/chapter-2/sensors/sensor-plugins.md` covering: Gazebo sensor plugin architecture, connecting sensors to ROS 2 topics
- [ ] T032 [P] [US3] Add PersonalizeButton and TranslateButton to all physics and sensor section pages

### Interactive Components for User Story 3

- [ ] T033 [P] [US3] Create `PhysicsParameterSlider` component in `src/components/PhysicsParameterSlider.tsx` with props: parameter (gravity/friction/mass), minValue, maxValue, defaultValue, unit
- [ ] T034 [P] [US3] Create SVG force visualization in PhysicsParameterSlider - show force arrows on robot diagram that update with slider value
- [ ] T035 [P] [US3] Embed PhysicsParameterSlider in `docs/chapter-2/physics/gravity-friction.md` with gravity slider (0-20 m/s²)
- [ ] T036 [P] [US3] Create sample sensor data files: `static/data/lidar-sample.json`, `static/data/camera-sample.json`, `static/data/imu-sample.json`
- [ ] T037 [P] [US3] Create `SensorVisualization` component in `src/components/SensorVisualization.tsx` using Chart.js for LiDAR (polar chart), IMU (line chart)
- [ ] T038 [P] [US3] Embed SensorVisualization in `docs/chapter-2/sensors/lidar.md` with LiDAR point cloud polar chart
- [ ] T039 [P] [US3] Embed SensorVisualization in `docs/chapter-2/sensors/imu.md` with IMU acceleration/gyro time-series chart

### RAG & Validation for User Story 3

- [ ] T040 [US3] Re-run embedding script to add physics and sensor content to Qdrant (execute: `python backend/src/scripts/embed_chapter2.py`)
- [ ] T041 [US3] Test RAG chatbot with 5 advanced Chapter 2 questions: "How do I add a LiDAR sensor?", "Which physics engine should I use?", "How do I configure friction?", "What's a camera plugin?", "How does IMU noise work?"
- [ ] T042 [US3] Verify personalization on sensor content - test with hardware background user, check for real-world sensor specs emphasis
- [ ] T043 [US3] Verify PhysicsParameterSlider updates force visualization in real-time
- [ ] T044 [US3] Verify SensorVisualization renders Chart.js charts correctly for LiDAR and IMU

**Checkpoint**: At this point, User Story 3 is complete - Physics and sensor simulation content is available with interactive visualizations

---

## Phase 6: User Story 4 - Track Learning Progress Across Chapters (Priority: P4)

**Goal**: Enable users to see multi-chapter progress and resume learning

**Independent Test**: Complete sections in Chapter 2, logout, login, verify progress persists, check dashboard shows Chapter 1 + Chapter 2 completion percentages

### Implementation for User Story 4

- [ ] T045 [P] [US4] Create Dashboard page at `src/pages/dashboard.tsx` with ProgressTracker component
- [ ] T046 [P] [US4] Create ProgressTracker component in `src/components/ProgressTracker.tsx` accepting props: userId, chapters (from chapters-config.json)
- [ ] T047 [P] [US4] Implement progress API integration in ProgressTracker - fetch from `GET /api/progress?user_id=<uuid>`
- [ ] T048 [P] [US4] Design progress UI in ProgressTracker: Chapter cards showing completion percentage, sections completed, last accessed timestamp
- [ ] T049 [P] [US4] Add navbar badge in `docusaurus.config.ts` showing global progress (e.g., "Progress: 1.5 / 2") linking to `/dashboard`
- [ ] T050 [P] [US4] Create ProgressContext in `src/contexts/ProgressContext.tsx` for React state management across components
- [ ] T051 [US4] Add section completion tracking to Chapter 2 pages - call `POST /api/progress/update` when user scrolls to bottom of section
- [ ] T052 [US4] Test progress tracking: Complete `intro.md` in Chapter 2, verify database record created with `chapter_id="chapter-2"`, `sections_completed=["intro"]`
- [ ] T053 [US4] Test progress persistence: Logout, login, navigate to `/dashboard`, verify Chapter 2 progress displays correctly
- [ ] T054 [US4] Test multi-chapter progress: Complete Chapter 1 (100%), partially complete Chapter 2 (50%), verify dashboard shows "Overall: 1.5 / 2 chapters"
- [ ] T055 [US4] Test "Resume Learning" prompt: Close browser mid-chapter, reopen, verify prompt offers to resume from Chapter 2

**Checkpoint**: All user stories complete - Chapter 2 fully functional with multi-chapter progress tracking

---

## Phase 7: Exercises & Examples (Content Completion)

**Purpose**: Add hands-on learning activities for Chapter 2

- [ ] T056 [P] Create hands-on exercise at `docs/chapter-2/exercises/build-your-robot.md` with step-by-step URDF creation instructions
- [ ] T057 [P] Create `docs/chapter-2/examples/robot-with-sensors.md` with URDF example including camera and LiDAR sensors
- [ ] T058 [P] Create `docs/chapter-2/examples/physics-demo.md` with SDF world file demonstrating gravity and collisions
- [ ] T059 Add CodeEditor component (Monaco XML mode) to exercise pages for in-browser URDF editing
- [ ] T060 Re-run embedding script to add exercises and examples to RAG (execute: `python backend/src/scripts/embed_chapter2.py`)

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final refinements, testing, validation

- [ ] T061 [P] Visual consistency check: Side-by-side comparison of Chapter 1 and Chapter 2 layouts, typography, spacing - document any inconsistencies
- [ ] T062 [P] RAG accuracy validation: Test 10 diverse Chapter 2 questions, calculate accuracy percentage, target ≥90% per spec SC-004
- [ ] T063 [P] Performance testing: Measure Chapter 2 page load time (target <3s), chatbot response time (target <3s), personalization (<2s), translation (<5s)
- [ ] T064 [P] Cross-browser testing: Test Chapter 2 navigation and bonus features in Chrome, Firefox, Safari
- [ ] T065 [P] Mobile responsive testing: Test Chapter 2 on screen widths 320px, 768px, 1920px - verify layouts adapt correctly
- [ ] T066 [P] Fix visual inconsistencies identified in T061 - update CSS or component styles to match Chapter 1
- [ ] T067 [P] Improve RAG accuracy if <90% in T062 - adjust chunking strategy, add more context, or refine prompts
- [ ] T068 [P] Optimize performance if targets not met in T063 - enable Docusaurus production build, optimize images, lazy-load components
- [ ] T069 Verify all Chapter 2 links are functional - no broken internal links, all external links valid
- [ ] T070 Verify all code examples render with correct syntax highlighting - test XML (URDF/SDF), Python, Bash
- [ ] T071 Final integration test: Complete user journey from signup → Chapter 1 → Chapter 2 → personalization → translation → chatbot → progress tracking

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - User stories can then proceed in priority order (P1 → P2 → P3 → P4)
  - US1 and US2 can proceed in parallel after Foundational (different content areas)
  - US3 depends on US2 (builds on Gazebo basics)
  - US4 can proceed in parallel with US2/US3 (different feature area)
- **Exercises (Phase 7)**: Depends on US2/US3 content completion
- **Polish (Phase 8)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - No dependencies on US1 (content is independent)
- **User Story 3 (P3)**: Depends on US2 content completion (physics/sensors build on Gazebo basics)
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - Independent of content stories (US2/US3)

### Within Each User Story

**User Story 1**:
- T007-T010 can run in parallel (different pages/components)
- T011-T013 must run sequentially (testing)

**User Story 2**:
- T014-T019 can run in parallel (different content files)
- T020 must complete before T021 (create script before running it)
- T021 must complete before T022 (populate embeddings before testing RAG)
- T022-T024 can run in parallel (different feature tests)

**User Story 3**:
- T025-T032 can run in parallel (different content files)
- T033-T039 can run in parallel (different components/visualizations)
- T040 must complete before T041 (populate embeddings before testing)
- T041-T044 can run in parallel (different tests)

**User Story 4**:
- T045-T050 can run in parallel (different components/pages)
- T051 must complete before T052-T055 (implement before testing)
- T052-T055 can run sequentially (progressive testing)

### Parallel Opportunities

- All Setup tasks (T001-T003) marked [P] can run in parallel
- All Foundational tasks (T004-T006) marked [P] can run in parallel (within Phase 2)
- After Foundational phase completes:
  - US1 (T007-T010) + US2 content (T014-T019) can run in parallel
  - US4 setup (T045-T050) can run in parallel with US1/US2 content
- Interactive components (T033-T039) can be built in parallel

---

## Parallel Example: User Story 2

```bash
# Launch all content creation tasks for User Story 2 together:
Task: "Write docs/chapter-2/what-is-gazebo.md"
Task: "Write docs/chapter-2/urdf-sdf/urdf-basics.md"
Task: "Write docs/chapter-2/urdf-sdf/sdf-basics.md"
Task: "Write docs/chapter-2/urdf-sdf/urdf-vs-sdf.md"
Task: "Create docs/chapter-2/examples/simple-robot.md"
Task: "Add PersonalizeButton and TranslateButton to all URDF/SDF pages"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T006) - CRITICAL, blocks all stories
3. Complete Phase 3: User Story 1 (T007-T013)
4. **STOP and VALIDATE**: Test Chapter 2 navigation, verify bonus feature components present
5. Deploy/demo if ready (Chapter 2 accessible with placeholders for content)

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 (navigation) → Test independently → Deploy (MVP!)
3. Add User Story 2 (Gazebo/URDF content + RAG) → Test independently → Deploy
4. Add User Story 3 (physics/sensors + visualizations) → Test independently → Deploy
5. Add User Story 4 (progress tracking) → Test independently → Deploy
6. Add Exercises (Phase 7) → Test → Deploy
7. Polish (Phase 8) → Final testing → Production release

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T006)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T007-T013) + User Story 4 (T045-T055)
   - **Developer B**: User Story 2 content (T014-T024)
   - **Developer C**: User Story 3 content (T025-T032) + Interactive components (T033-T039)
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story is independently completable and testable
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Tests are NOT included (not requested in spec, manual testing per constitution)
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence

---

## Task Summary

**Total Tasks**: 71

**By Phase**:
- Setup: 3 tasks
- Foundational: 3 tasks (BLOCKING)
- User Story 1 (P1): 7 tasks - Navigation & Access
- User Story 2 (P2): 11 tasks - Gazebo & URDF Content + RAG
- User Story 3 (P3): 20 tasks - Physics & Sensors + Interactive Components
- User Story 4 (P4): 11 tasks - Progress Tracking
- Exercises: 5 tasks
- Polish: 11 tasks

**Parallel Opportunities**:
- Setup phase: 2 parallel tasks
- Foundational phase: 2 parallel tasks
- US1: 4 parallel tasks
- US2: 6 parallel content tasks, 3 parallel test tasks
- US3: 8 parallel content tasks, 7 parallel component tasks, 4 parallel test tasks
- US4: 6 parallel setup tasks
- Exercises: 3 parallel tasks
- Polish: 5 parallel tasks

**MVP Scope** (User Story 1 only):
- Total: 9 tasks (T001-T006 + T007-T013)
- Delivers: Chapter 2 accessible with navigation, bonus feature components present (placeholders OK)

**Full Feature** (All User Stories):
- Total: 71 tasks
- Delivers: Complete Chapter 2 with content, RAG, interactive visualizations, progress tracking

**Critical Path**: Setup → Foundational → US2 content → US2 RAG → US3 content → US3 RAG → Polish
**Estimated Effort**: 5-7 days (1 developer), 3-4 days (3 developers in parallel)
