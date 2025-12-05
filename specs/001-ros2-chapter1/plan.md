# Implementation Plan: ROS 2 Chapter 1 - The Robotic Nervous System

**Branch**: `001-ros2-chapter1` | **Date**: 2025-12-03 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-ros2-chapter1/spec.md`

## Summary

Build a complete, self-contained educational chapter on ROS 2 Fundamentals using Docusaurus, integrated with four bonus features (authentication, personalization, Urdu translation, RAG chatbot) to maximize hackathon scoring (300 total points). The chapter includes theory, interactive diagrams, Python code examples, exercises, and a quiz. All components are designed as reusable modules for future chapters. Target deployment: GitHub Pages/Vercel within 5-day timeline.

**Technical Approach**: Hybrid web application with Docusaurus static frontend extended with React components for interactivity, FastAPI backend for AI services (translation, chatbot, personalization), Neon Postgres for user/progress data, Qdrant Cloud for vector embeddings, and better-auth.com for authentication.

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.x + JavaScript (ES2022)
- Backend: Python 3.11+

**Primary Dependencies**:
- Frontend: Docusaurus 3.x, React 18, @monaco-editor/react (code editor), @react-three/fiber (3D viewer), better-auth client SDK
- Backend: FastAPI 0.104+, OpenAI Python SDK 1.x, Qdrant Client 1.7+, psycopg2-binary 2.9+ (Postgres), better-auth Python SDK

**Storage**:
- Primary DB: Neon Serverless Postgres (users, progress_records, chat_messages, translations cache)
- Vector DB: Qdrant Cloud Free Tier (chapter content embeddings for RAG)
- Static Assets: GitHub/Vercel CDN (Docusaurus build output)
- Client Storage: localStorage (language preference, auth tokens)

**Testing**:
- Frontend: Manual testing (Chrome, Firefox, Safari), Responsive testing (mobile/tablet)
- Backend: Manual API testing with curl/Postman, RAG accuracy testing (10 sample questions)
- E2E: Manual user journey testing (signup → personalize → translate → chatbot → quiz)

**Target Platform**:
- Deployment: GitHub Pages or Vercel (static site hosting)
- Backend: Cloud platform (Railway, Render, Fly.io - supports FastAPI)
- Browsers: Chrome 100+, Firefox 100+, Safari 15+
- Mobile: iOS Safari, Android Chrome (responsive design)

**Project Type**: Web application (frontend + backend)

**Performance Goals**:
- Page load: <3 seconds (first contentful paint)
- Chatbot response: <3 seconds (p95)
- Translation: <5 seconds (full chapter)
- Personalization: <2 seconds (content swap)
- Concurrent users: 100 (demo/hackathon scale, not production)

**Constraints**:
- Timeline: 5 days (120 hours) from start to deployment
- Demo: 90 seconds max (judges only watch first 90s)
- Scope: Chapter 1 only (no multi-chapter navigation)
- Cost: Free tier services only (Neon, Qdrant, Vercel/GitHub Pages)
- Mobile: Must be responsive (judges may view on phones)

**Scale/Scope**:
- Content: ~10 sections, 5 code examples, 10 quiz questions, 1 exercise
- Users: ~50-100 during hackathon evaluation period
- Database: <1000 user records, <10,000 chat messages, <500 translation cache entries
- Vector embeddings: ~200-300 chunks (Chapter 1 content)
- Lines of code: ~5,000-7,000 (frontend + backend combined)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Single Chapter Focus ✅ PASS
- **Rule**: All features (core + bonus) MUST be implemented for Chapter 1
- **Compliance**: Plan scopes all development to Chapter 1 only. No multi-chapter features planned.
- **Rule**: Chapter 1 MUST function independently
- **Compliance**: No navigation to other chapters. All content, examples, quiz self-contained.
- **Rule**: No placeholder content or "coming soon" features
- **Compliance**: All planned sections will be complete. Exercises and quiz fully functional.

### II. Bonus Features Integration ✅ PASS
- **Rule**: Auth (better-auth.com) MUST be implemented before content development
- **Compliance**: Tasks.md Phase 7 implements auth. Will be prioritized in Day 1-2.
- **Rule**: Personalization button MUST be present at Chapter 1 start
- **Compliance**: Planned in Phase 4 (tasks.md T051-T055)
- **Rule**: Urdu translation button MUST be present
- **Compliance**: Planned in Phase 5 (tasks.md T060-T066)
- **Rule**: All bonus features MUST be demonstrable in 90-second demo
- **Compliance**: Demo script (T127) will showcase: auth signup, personalize click, translate toggle, chatbot Q&A

### III. Modular Design ✅ PASS
- **Rule**: RAG chatbot MUST be chapter-agnostic (accepts any chapter content)
- **Compliance**: RAGChatbot component (T077) will accept `chapterId` prop. Backend RAGService (T071) parameterized.
- **Rule**: Personalization engine MUST be configurable per chapter
- **Compliance**: PersonalizationEngine (T047) accepts user profile + chapter content as parameters.
- **Rule**: Translation system MUST accept any chapter content
- **Compliance**: TranslationService (T056) accepts text input, not hardcoded to Chapter 1.
- **Rule**: UI components MUST be reusable React components
- **Compliance**: Components extracted: PersonalizeButton, TranslateButton, RAGChatbot, QuizWidget (Phase 8)
- **Rule**: Database schemas MUST accommodate multi-chapter metadata
- **Compliance**: `chapter_id` field in progress_records table, chat_messages, vector embeddings

### IV. Self-Contained Chapter ✅ PASS
- **Rule**: No navigation links to unimplemented chapters
- **Compliance**: Docusaurus config (T002) limits docs to chapter-1 only. No sidebar links to other chapters.
- **Rule**: Chapter 1 learning outcomes MUST be achievable with provided content alone
- **Compliance**: All ROS 2 concepts (Nodes, Topics, Services) explained with code examples. Exercise is runnable.
- **Rule**: All code examples MUST be complete and runnable
- **Compliance**: 5 Python examples (T029-T033) with expected output. Syntax highlighted (T034).
- **Rule**: RAG chatbot MUST answer questions using only Chapter 1 content
- **Compliance**: Scope limiting implemented (T075) - rejects off-topic questions

### V. Demo-Ready Excellence ✅ PASS
- **Rule**: UI MUST be polished (no placeholder text, broken styles, console errors)
- **Compliance**: Polish phase (T118-T123) includes error removal, UI consistency, animations
- **Rule**: Feature transitions MUST be smooth (no loading spinners exceeding 2 seconds)
- **Compliance**: Performance requirements: personalization <2s, chatbot <3s, translation <5s with loading states
- **Rule**: Personalization and translation MUST show visible, immediate changes
- **Compliance**: Content swap on button click (T053), Urdu text replacement (T061-T064)
- **Rule**: RAG chatbot responses MUST be fast (<3 seconds) and accurate
- **Compliance**: Target <3s (T082), accuracy testing with 10 questions (T083, T115)
- **Rule**: Demo script MUST showcase all bonus features within 90 seconds
- **Compliance**: Demo script task (T127) explicitly requires 90s showcase of all bonus features

### VI. Maximum Points Optimization ✅ PASS
- **Rule**: Bonus features take priority over content polish
- **Compliance**: 5-day plan allocates Day 3 entirely to bonus features (personalization, translation, chatbot, auth)
- **Rule**: Claude Code Subagents/Skills MUST be created and documented (50 points)
- **Compliance**: Phase 8 (T107-T110) creates Subagent for chapter generation and Skill for quiz generation
- **Rule**: Better-auth.com MUST be implemented with user profiling (50 points)
- **Compliance**: Phase 7 (T084-T099) integrates better-auth with background questionnaire
- **Rule**: Content personalization MUST be functional (50 points)
- **Compliance**: Phase 4 (T047-T055) implements personalization engine with content adaptation
- **Rule**: Urdu translation MUST be functional (50 points)
- **Compliance**: Phase 5 (T056-T066) implements OpenAI-powered translation with term preservation
- **Rule**: Demo video MUST explicitly highlight all implemented bonus features
- **Compliance**: Demo recording task (T128) with explicit requirement to showcase bonus features

**GATE RESULT**: ✅ ALL PRINCIPLES PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/001-ros2-chapter1/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (technology decisions, best practices)
├── data-model.md        # Phase 1 output (database schema, entities)
├── quickstart.md        # Phase 1 output (local development setup)
├── contracts/           # Phase 1 output (API endpoint specifications)
│   ├── auth.yaml        # Authentication endpoints
│   ├── chatbot.yaml     # RAG chatbot endpoints
│   ├── personalization.yaml  # Content personalization endpoints
│   └── translation.yaml # Translation endpoints
├── spec.md              # Feature specification (already created)
└── tasks.md             # Task list (already created)
```

### Source Code (repository root)

```text
# Option 2: Web application (frontend + backend)

book/                    # Repository root
├── docs/                # Docusaurus content (markdown files)
│   └── chapter-1/       # Chapter 1 content
│       ├── intro.md
│       ├── what-is-ros2.md
│       ├── why-ros2.md
│       ├── core-concepts/
│       │   ├── nodes.md
│       │   ├── topics.md
│       │   └── services.md
│       ├── installation.md
│       ├── urdf-basics.md
│       ├── examples/
│       │   ├── example-01-hello-node.md
│       │   ├── example-02-publisher.md
│       │   ├── example-03-subscriber.md
│       │   ├── example-04-service-server.md
│       │   └── example-05-service-client.md
│       └── exercises/
│           └── exercise-01.md
│
├── src/                 # Docusaurus frontend source
│   ├── components/      # React components
│   │   ├── auth/
│   │   │   ├── SignUpForm.tsx
│   │   │   └── SignInForm.tsx
│   │   ├── ArchitectureDiagram.tsx
│   │   ├── CodeEditor.tsx
│   │   ├── RobotViewer.tsx
│   │   ├── Quiz.tsx
│   │   ├── CompletionBadge.tsx
│   │   ├── PersonalizeButton.tsx
│   │   ├── TranslateButton.tsx
│   │   ├── RAGChatbot.tsx
│   │   ├── QuizWidget.tsx
│   │   ├── ProfileDashboard.tsx
│   │   └── ProgressTracker.tsx
│   ├── css/
│   │   └── custom.css
│   ├── contexts/
│   │   ├── AuthContext.tsx
│   │   └── LanguageContext.tsx
│   └── pages/
│       └── index.tsx
│
├── static/              # Static assets
│   ├── img/
│   └── models/          # 3D robot models (URDF)
│
├── backend/             # FastAPI backend
│   ├── src/
│   │   ├── main.py      # FastAPI app entry point
│   │   ├── config.py    # Environment variable loading
│   │   ├── database.py  # Neon Postgres connection
│   │   ├── vector_store.py  # Qdrant client
│   │   ├── ai_client.py # OpenAI client
│   │   ├── auth/
│   │   │   └── better_auth.py  # better-auth integration
│   │   ├── models/
│   │   │   ├── user.py
│   │   │   ├── progress.py
│   │   │   └── chat.py
│   │   ├── services/
│   │   │   ├── personalization_engine.py
│   │   │   ├── translation_service.py
│   │   │   └── rag_service.py
│   │   ├── routes/
│   │   │   ├── auth.py
│   │   │   ├── chatbot.py
│   │   │   ├── personalization.py
│   │   │   └── translation.py
│   │   └── scripts/
│   │       └── chunk_content.py  # Vector embedding generation
│   ├── requirements.txt
│   └── .env.example
│
├── .claude/             # Claude Code integration (bonus points)
│   ├── agents/
│   │   └── chapter-generator.md
│   └── skills/
│       └── quiz-skill.md
│
├── docusaurus.config.js # Docusaurus configuration
├── package.json
├── .gitignore
└── README.md
```

**Structure Decision**: Web application structure (Option 2) selected because the project requires both a static documentation frontend (Docusaurus) and a dynamic backend for AI services (FastAPI). Frontend serves the educational content and UI, while backend handles authentication, RAG chatbot queries, translation requests, and personalization logic. This separation enables independent scaling and deployment (frontend to GitHub Pages/Vercel, backend to Railway/Render).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected** - All constitution checks passed. No complexity justification required.

---

## Phase 0: Research & Technology Decisions

**Goal**: Resolve all technical unknowns and establish best practices for key technologies

**Output**: `research.md` with decisions, rationale, and alternatives considered

**Research Tasks**: (See next section for research.md generation)

---

## Phase 1: Data Model & API Contracts

**Goal**: Define database schema, API endpoints, and development quickstart

**Prerequisites**: research.md complete

**Outputs**:
1. `data-model.md` - Entity definitions and relationships
2. `contracts/*.yaml` - OpenAPI specifications for all endpoints
3. `quickstart.md` - Local development setup guide

**Data Entities** (to be detailed in data-model.md):
- User (email, password_hash, name, software_level, hardware_level, learning_goals)
- ProgressRecord (user_id, chapter_id, sections_completed, quiz_score, completion_date)
- ChatMessage (session_id, user_question, bot_response, timestamp, context_used)
- TranslationCache (original_text_hash, translated_text, language, created_at)
- VectorEmbedding (chunk_id, chapter_id, text_content, embedding_vector, metadata)

**API Endpoints** (to be detailed in contracts/):
- POST /api/auth/signup
- POST /api/auth/login
- POST /api/auth/logout
- GET /api/user/profile
- PUT /api/user/profile
- POST /api/personalize
- POST /api/translate
- POST /api/chatbot/ask
- GET /api/progress/:userId/:chapterId
- PUT /api/progress/:userId/:chapterId

---

## Implementation Phases

### Phase 2: Foundation (Tasks T001-T020)
- Initialize Docusaurus project with TypeScript
- Set up FastAPI backend structure
- Configure databases (Neon Postgres, Qdrant Cloud)
- Set up OpenAI client and better-auth integration

### Phase 3: Core Content (Tasks T021-T046)
- Write Chapter 1 markdown content (intro, concepts, examples)
- Build interactive components (diagrams, code editor, 3D viewer)
- Create quiz and exercises
- Implement progress tracking

### Phase 4: Bonus Features - Personalization (Tasks T047-T055)
- PersonalizationEngine service
- Content adaptation logic
- PersonalizeButton component

### Phase 5: Bonus Features - Translation (Tasks T056-T066)
- TranslationService with OpenAI GPT-4
- TranslateButton component
- Language preference persistence

### Phase 6: Bonus Features - RAG Chatbot (Tasks T067-T083)
- Vector embedding generation and upload to Qdrant
- RAGService with semantic search
- RAGChatbot UI component

### Phase 7: Bonus Features - Authentication (Tasks T084-T099)
- better-auth.com integration
- SignUp/SignIn forms with background questionnaire
- Profile dashboard

### Phase 8: Reusability (Tasks T100-T110)
- Refactor components to be chapter-agnostic
- Create Claude Code Subagents and Skills
- Document reusable component APIs

### Phase 9: Polish & Deployment (Tasks T111-T130)
- Cross-browser and responsive testing
- Performance optimization
- Deploy to GitHub Pages/Vercel
- Record 90-second demo video

---

## Risk Assessment

**High Risk**:
1. **Qdrant vector embedding setup** - First time using Qdrant Cloud
   - Mitigation: Follow official Qdrant Python client docs, test with small dataset first
2. **better-auth.com integration** - Relatively new service
   - Mitigation: Review better-auth docs thoroughly, allocate extra time for auth debugging
3. **90-second demo constraint** - Must showcase 4 bonus features + core content
   - Mitigation: Create demo script early (Day 4), practice multiple times

**Medium Risk**:
1. **OpenAI API rate limits** - Heavy usage for translation + chatbot + embeddings
   - Mitigation: Implement caching (translation cache table), use gpt-3.5-turbo where possible
2. **Urdu translation quality** - GPT-4 may not preserve technical terms correctly
   - Mitigation: Explicit system prompt with term preservation rules, manual review of key sections
3. **Mobile responsive design** - Interactive components (code editor, 3D viewer) may break on mobile
   - Mitigation: Test early and often on real devices, simplify mobile UX if needed

**Low Risk**:
1. **Docusaurus deployment** - Well-documented process
2. **FastAPI development** - Team familiar with Python/FastAPI
3. **React component development** - Standard React patterns

---

## Next Steps

1. **Execute Phase 0**: Generate `research.md` (technology decisions)
2. **Execute Phase 1**: Generate `data-model.md` and `contracts/*.yaml`
3. **Update agent context**: Run `.specify/scripts/powershell/update-agent-context.ps1`
4. **Begin implementation**: Start with tasks T001-T010 (project setup)

**Command completion**: Plan generation complete. Ready for `/sp.implement` or manual task execution.
