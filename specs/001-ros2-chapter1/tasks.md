# Tasks: ROS 2 Chapter 1 - The Robotic Nervous System

**Input**: Feature specification from `specs/001-ros2-chapter1/spec.md` and user-provided task list
**Prerequisites**: spec.md (required)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5, US6)
- Include exact file paths in descriptions

## Implementation Strategy

**MVP Scope**: User Story 1 (P1) - Core learning content
**Incremental Delivery**: Add bonus features (US2-US5) after MVP is solid
**Demo-Ready**: All tasks aligned with 90-second demo requirements

---

## Phase 1: Project Setup

**Purpose**: Initialize Docusaurus project and configure for single-chapter deployment

- [ ] T001 Initialize Docusaurus project with TypeScript template in root directory
- [ ] T002 [P] Configure docusaurus.config.js for single chapter (disable blog, limit docs to chapter-1)
- [ ] T003 [P] Set up GitHub Pages deployment configuration in docusaurus.config.js (baseUrl, organizationName, projectName)
- [ ] T004 [P] Create .gitignore for node_modules, build artifacts, and environment variables
- [ ] T005 [P] Install core dependencies: @docusaurus/core, @docusaurus/preset-classic, react, react-dom
- [ ] T006 [P] Create project structure: docs/, src/components/, src/css/, static/
- [ ] T007 [P] Configure package.json scripts: start, build, deploy
- [ ] T008 [P] Create .env.example file with placeholders for API keys (OPENAI_API_KEY, NEON_DATABASE_URL, QDRANT_API_KEY, BETTER_AUTH_SECRET)
- [ ] T009 [P] Set up ESLint and Prettier for code quality
- [ ] T010 [P] Initialize Git repository and create initial commit

---

## Phase 2: Foundational Infrastructure

**Purpose**: Set up backend services and database schema before user stories

- [ ] T011 Create FastAPI backend project structure: backend/src/, backend/requirements.txt
- [ ] T012 [P] Install Python dependencies in backend/requirements.txt: fastapi, uvicorn, python-dotenv, openai, qdrant-client, psycopg2-binary, better-auth
- [ ] T013 [P] Configure FastAPI app in backend/src/main.py with CORS middleware
- [ ] T014 [P] Set up Neon Serverless Postgres connection in backend/src/database.py
- [ ] T015 [P] Create database schema for users table (id, email, password_hash, name, software_level, hardware_level, learning_goals, created_at)
- [ ] T016 [P] Create database schema for progress_records table (id, user_id, chapter_id, sections_completed, quiz_score, completion_date)
- [ ] T017 [P] Set up Qdrant Cloud client in backend/src/vector_store.py
- [ ] T018 [P] Configure OpenAI client in backend/src/ai_client.py (for embeddings and GPT-4)
- [ ] T019 [P] Create environment variable loading in backend/src/config.py
- [ ] T020 [P] Test database connection and create tables if not exist

---

## Phase 3: User Story 1 - Core Learning Content (P1)

**Goal**: New learner can discover and complete ROS 2 fundamentals
**Independent Test**: User with Python knowledge but no ROS 2 experience reads chapter, completes exercise, and can explain what ROS 2 is

### Content Creation

- [ ] T021 [US1] Write Chapter 1 introduction section (500 words) in docs/chapter-1/intro.md
- [ ] T022 [P] [US1] Create "What is ROS 2?" section explaining middleware, distributed systems, and robotics context in docs/chapter-1/what-is-ros2.md
- [ ] T023 [P] [US1] Write "Why ROS 2?" section covering use cases, industry adoption, and benefits in docs/chapter-1/why-ros2.md
- [ ] T024 [P] [US1] Create "Core Concepts" section explaining Nodes in docs/chapter-1/core-concepts/nodes.md
- [ ] T025 [P] [US1] Write Topics & Publishers/Subscribers explanation in docs/chapter-1/core-concepts/topics.md
- [ ] T026 [P] [US1] Create Services explanation in docs/chapter-1/core-concepts/services.md
- [ ] T027 [P] [US1] Write Installation Guide for ROS 2 Humble in docs/chapter-1/installation.md
- [ ] T028 [US1] Create URDF basics section with robot description fundamentals in docs/chapter-1/urdf-basics.md

### Interactive Code Examples

- [ ] T029 [P] [US1] Create Example 1: Minimal ROS 2 Node (Hello World) in docs/chapter-1/examples/example-01-hello-node.md
- [ ] T030 [P] [US1] Create Example 2: Publisher Node (publishing string messages) in docs/chapter-1/examples/example-02-publisher.md
- [ ] T031 [P] [US1] Create Example 3: Subscriber Node (receiving messages) in docs/chapter-1/examples/example-03-subscriber.md
- [ ] T032 [P] [US1] Create Example 4: Service Server (responding to requests) in docs/chapter-1/examples/example-04-service-server.md
- [ ] T033 [P] [US1] Create Example 5: Service Client (making requests) in docs/chapter-1/examples/example-05-service-client.md
- [ ] T034 [US1] Add syntax highlighting configuration for Python code blocks in docusaurus.config.js (Prism theme)

### Interactive Elements

- [ ] T035 [P] [US1] Create clickable ROS 2 architecture diagram component in src/components/ArchitectureDiagram.tsx
- [ ] T036 [P] [US1] Implement diagram interactivity (click nodes to show tooltips with explanations)
- [ ] T037 [P] [US1] Build live Python code editor component in src/components/CodeEditor.tsx using @monaco-editor/react
- [ ] T038 [P] [US1] Add "Run Code" button to CodeEditor (executes in sandboxed environment or shows expected output)
- [ ] T039 [P] [US1] Implement 3D robot model viewer component in src/components/RobotViewer.tsx using @react-three/fiber
- [ ] T040 [US1] Load sample URDF robot model into RobotViewer for URDF basics section

### Exercises & Quiz

- [ ] T041 [P] [US1] Design hands-on exercise: "Build Your First ROS 2 Node" in docs/chapter-1/exercises/exercise-01.md
- [ ] T042 [P] [US1] Create exercise instructions with step-by-step guide and expected output
- [ ] T043 [P] [US1] Create 10-question multiple-choice quiz in src/components/Quiz.tsx
- [ ] T044 [US1] Implement quiz logic with instant feedback and explanations for each answer
- [ ] T045 [P] [US1] Add progress indicator showing sections completed
- [ ] T046 [US1] Create completion badge component in src/components/CompletionBadge.tsx (shown when quiz passed)

---

## Phase 4: User Story 2 - Content Personalization (P2)

**Goal**: Logged-in user can personalize content based on their background profile
**Independent Test**: Create account with specific background, click "Personalize Content", verify content adapts

### Personalization Engine

- [ ] T047 [P] [US2] Create PersonalizationEngine service in backend/src/services/personalization_engine.py
- [ ] T048 [P] [US2] Implement content adaptation logic (software beginner → detailed code explanations, hardware advanced → skip hardware basics)
- [ ] T049 [P] [US2] Create API endpoint POST /api/personalize in backend/src/routes/personalization.py (accepts user profile, returns adapted content)
- [ ] T050 [US2] Implement content filtering rules based on user's software_level and hardware_level fields

### Frontend Integration

- [ ] T051 [P] [US2] Build PersonalizeButton component in src/components/PersonalizeButton.tsx
- [ ] T052 [P] [US2] Add PersonalizeButton to Chapter 1 page header (visible only when logged in)
- [ ] T053 [P] [US2] Implement toggle logic (on → fetch personalized content, off → show default content)
- [ ] T054 [P] [US2] Create PersonalizationIndicator component showing active personalization state
- [ ] T055 [US2] Test content adaptation: beginner software user sees detailed Python explanations, advanced user sees concise explanations

---

## Phase 5: User Story 3 - Urdu Translation (P2)

**Goal**: User can translate entire chapter to Urdu while preserving code and technical terms
**Independent Test**: Click "Translate to Urdu", verify prose is Urdu, code/terms remain English

### Translation Service

- [ ] T056 [P] [US3] Create TranslationService in backend/src/services/translation_service.py
- [ ] T057 [P] [US3] Implement OpenAI GPT-4 translation with context preservation (system prompt: "Translate to Urdu, preserve ROS 2, Node, Topic, Service, rclpy in English")
- [ ] T058 [P] [US3] Create API endpoint POST /api/translate in backend/src/routes/translation.py (accepts text, returns Urdu)
- [ ] T059 [US3] Add caching layer for translated content (store translations in PostgreSQL to avoid re-translating)

### Frontend Integration

- [ ] T060 [P] [US3] Build TranslateButton component in src/components/TranslateButton.tsx
- [ ] T061 [P] [US3] Add TranslateButton to Chapter 1 page header with language toggle (English ↔ Urdu)
- [ ] T062 [P] [US3] Implement translation state management using React Context (LanguageContext)
- [ ] T063 [P] [US3] Create translation loader that fetches Urdu content from backend API
- [ ] T064 [P] [US3] Preserve code blocks during translation (detect ```python blocks, skip translation)
- [ ] T065 [P] [US3] Store language preference in localStorage for persistence across sessions
- [ ] T066 [US3] Test translation: verify headings, paragraphs in Urdu, code examples in English, technical terms preserved

---

## Phase 6: User Story 4 - RAG Chatbot (P2)

**Goal**: User can ask questions about Chapter 1 and receive AI-generated answers based on chapter content
**Independent Test**: Ask 10 questions about chapter, verify accurate, contextually relevant answers

### Vector Database Setup

- [ ] T067 [P] [US4] Create content chunking script in backend/src/scripts/chunk_content.py (splits Chapter 1 into semantic chunks)
- [ ] T068 [P] [US4] Generate OpenAI embeddings for each chunk using text-embedding-ada-002 model
- [ ] T069 [P] [US4] Upload embeddings to Qdrant Cloud collection "chapter-1-embeddings"
- [ ] T070 [US4] Create metadata for each chunk (section title, subsection, chunk index)

### RAG Backend

- [ ] T071 [P] [US4] Create RAGService in backend/src/services/rag_service.py
- [ ] T072 [P] [US4] Implement semantic search (user question → embedding → Qdrant query → top 5 relevant chunks)
- [ ] T073 [P] [US4] Implement OpenAI Agents/ChatKit SDK integration for conversational responses
- [ ] T074 [P] [US4] Create API endpoint POST /api/chatbot/ask in backend/src/routes/chatbot.py (accepts question, returns answer with citations)
- [ ] T075 [P] [US4] Implement scope limiting (reject questions outside Chapter 1 content with polite message)
- [ ] T076 [US4] Add chat history storage in PostgreSQL (session_id, user_question, bot_response, timestamp)

### Chatbot Frontend

- [ ] T077 [P] [US4] Build RAGChatbot component in src/components/RAGChatbot.tsx
- [ ] T078 [P] [US4] Create floating chatbot button (bottom-right corner, opens chat modal)
- [ ] T079 [P] [US4] Implement chat interface (message list, input field, send button)
- [ ] T080 [P] [US4] Add "Ask about selected text" feature (right-click menu or button when text selected)
- [ ] T081 [P] [US4] Display loading state during API call (<3 second response time target)
- [ ] T082 [P] [US4] Show chat history within session (scroll to view previous Q&A)
- [ ] T083 [US4] Test chatbot accuracy: ask "What is a ROS 2 Node?", "How do I create a publisher?", verify correct answers citing chapter sections

---

## Phase 7: User Story 5 - Authentication & Profile (P3)

**Goal**: User can sign up, log in, access profile dashboard, and track progress
**Independent Test**: Sign up, log out, log back in, verify profile persists

### Authentication Backend

- [ ] T084 [P] [US5] Integrate better-auth.com SDK in backend/src/auth/better_auth.py
- [ ] T085 [P] [US5] Create API endpoint POST /api/auth/signup in backend/src/routes/auth.py (accepts email, password, background questions)
- [ ] T086 [P] [US5] Create API endpoint POST /api/auth/login (validates credentials, returns JWT token)
- [ ] T087 [P] [US5] Create API endpoint POST /api/auth/logout (invalidates session)
- [ ] T088 [P] [US5] Create API endpoint GET /api/user/profile (returns user profile data)
- [ ] T089 [P] [US5] Create API endpoint PUT /api/user/profile (updates background questions)
- [ ] T090 [US5] Implement session persistence with JWT tokens (7-day expiry)

### Authentication Frontend

- [ ] T091 [P] [US5] Build SignUpForm component in src/components/auth/SignUpForm.tsx
- [ ] T092 [P] [US5] Add background questionnaire to signup: software experience (dropdown), hardware experience (dropdown), learning goals (textarea)
- [ ] T093 [P] [US5] Build SignInForm component in src/components/auth/SignInForm.tsx
- [ ] T094 [P] [US5] Create AuthContext for global auth state (logged in user, token storage)
- [ ] T095 [P] [US5] Implement auth state persistence in localStorage (auto-login on return visit)
- [ ] T096 [P] [US5] Build ProfileDashboard component in src/components/ProfileDashboard.tsx
- [ ] T097 [P] [US5] Show user background questions in dashboard with edit capability
- [ ] T098 [P] [US5] Create ProgressTracker component in src/components/ProgressTracker.tsx (shows sections read, quiz score)
- [ ] T099 [US5] Test auth flow: signup → logout → login → verify profile data persists

---

## Phase 8: User Story 6 - Reusable Components (P3)

**Goal**: Developer can reuse chatbot, personalization, translation components for future chapters
**Independent Test**: Developer integrates chatbot into hypothetical Chapter 2 without modifying core code

### Component Modularization

- [ ] T100 [P] [US6] Refactor RAGChatbot to accept chapterId prop (make chapter-agnostic)
- [ ] T101 [P] [US6] Refactor PersonalizationEngine to accept chapter content as parameter
- [ ] T102 [P] [US6] Refactor TranslationService to accept any content input (not hardcoded to Chapter 1)
- [ ] T103 [P] [US6] Extract QuizWidget as reusable component in src/components/QuizWidget.tsx (accepts questions array prop)
- [ ] T104 [P] [US6] Create ComponentLibrary documentation in docs/developer/reusable-components.md
- [ ] T105 [P] [US6] Document API for each reusable component (props, usage examples, integration guide)
- [ ] T106 [US6] Create example integration showing how to add chatbot to a new chapter

### Claude Code Subagents & Skills

- [ ] T107 [P] [US6] Create Claude Code Subagent for chapter generation in .claude/agents/chapter-generator.md
- [ ] T108 [P] [US6] Define Subagent prompts: "Generate Chapter [N] structure based on Chapter 1 template"
- [ ] T109 [P] [US6] Create Skill for quiz generation in .claude/skills/quiz-skill.md
- [ ] T110 [US6] Document Subagents & Skills in docs/developer/claude-code-integration.md

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: UI/UX refinement, testing, deployment preparation

### Testing

- [ ] T111 [P] Test all interactive features on Chrome, Firefox, Safari
- [ ] T112 [P] Verify Urdu translation produces readable output (manual review by Urdu speaker)
- [ ] T113 [P] Test responsive design on mobile devices (iPhone, Android, tablets)
- [ ] T114 [P] Verify all code examples are runnable (test in local ROS 2 environment if possible)
- [ ] T115 [P] Test chatbot with 10 sample questions, verify accuracy >90%
- [ ] T116 [P] Test personalization with different user profiles (beginner/advanced combinations)
- [ ] T117 Test end-to-end user journey: signup → personalize → translate → chatbot → quiz

### Performance & Polish

- [ ] T118 [P] Optimize page load time (lazy load components, code splitting)
- [ ] T119 [P] Add loading states to async operations (translation, chatbot, personalization)
- [ ] T120 [P] Implement error handling with user-friendly messages (API failures, network issues)
- [ ] T121 [P] Polish UI (consistent spacing, typography, color scheme aligned with Docusaurus theme)
- [ ] T122 [P] Add animations for smooth transitions (button clicks, content swaps)
- [ ] T123 Remove console errors and warnings from browser DevTools

### Deployment

- [ ] T124 [P] Deploy backend to cloud provider (Railway, Render, or Fly.io)
- [ ] T125 [P] Deploy Docusaurus frontend to GitHub Pages (run npm run deploy)
- [ ] T126 [P] Verify deployment: test all features on live site
- [ ] T127 [P] Create demo script for 90-second video (outline key features to showcase)
- [ ] T128 [P] Record demo video (max 90 seconds, highlight all bonus features)
- [ ] T129 [P] Write README.md with setup instructions, feature list, and demo link
- [ ] T130 Submit project to hackathon form with GitHub repo link, deployed site link, demo video link

---

## Dependencies & Execution Order

### Story Completion Order

1. **Phase 1-2 (Setup & Foundation)** → MUST complete before any user story
2. **User Story 5 (Auth)** → SHOULD complete before US2 (personalization requires login)
3. **User Story 1 (Core Content)** → Can proceed in parallel with US5
4. **User Story 4 (Chatbot)** → Depends on US1 (needs chapter content to vectorize)
5. **User Story 2, 3 (Personalization, Translation)** → Can proceed in parallel after US1 and US5
6. **User Story 6 (Reusability)** → Best done after US1-5 complete (refactor existing components)

### Parallel Execution Examples

**Setup Phase (Day 1)**:
```bash
# Run in parallel:
T001, T002, T003, T004, T005, T006, T007, T008, T009 → Frontend setup
T011, T012, T013, T014, T015, T016, T017, T018, T019 → Backend setup
```

**Content Creation (Day 2)**:
```bash
# Run in parallel:
T021, T022, T023, T024, T025, T026, T027, T028 → Write all sections
T029, T030, T031, T032, T033 → Create all code examples
```

**Bonus Features (Day 3)**:
```bash
# Run in parallel (different features):
T047-T055 → Personalization (US2)
T056-T066 → Translation (US3)
T084-T099 → Authentication (US5)
```

---

## Task Summary

**Total Tasks**: 130
**Tasks per User Story**:
- Setup & Foundation: 20 tasks
- US1 (Core Content): 26 tasks
- US2 (Personalization): 9 tasks
- US3 (Translation): 11 tasks
- US4 (RAG Chatbot): 17 tasks
- US5 (Authentication): 16 tasks
- US6 (Reusability): 11 tasks
- Polish & Deploy: 20 tasks

**Parallel Opportunities**: 89 tasks marked [P] can run in parallel

**MVP Scope (Minimum Demo)**: Complete Phase 1-3 (Setup + US1) = 46 tasks
**Full Demo (All Bonus Features)**: Complete all 130 tasks

**Suggested 5-Day Breakdown**:
- **Day 1**: T001-T020 (Setup & Foundation)
- **Day 2**: T021-T046 (US1 Core Content)
- **Day 3**: T047-T099 (US2, US3, US4, US5 Bonus Features)
- **Day 4**: T100-T117 (US6 Reusability + Testing)
- **Day 5**: T118-T130 (Polish & Deployment)
