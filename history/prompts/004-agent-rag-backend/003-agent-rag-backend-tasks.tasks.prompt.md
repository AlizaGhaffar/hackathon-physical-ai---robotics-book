---
id: 003
title: Agent-Based RAG Backend Task Breakdown
stage: tasks
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-agent-rag-backend
branch: 004-agent-rag-backend
user: AlizaGhaffar
command: /sp.tasks
labels: ["rag", "tasks", "implementation", "user-stories", "fastapi", "openai-agents"]
links:
  spec: specs/004-agent-rag-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-agent-rag-backend/tasks.md
tests:
  - Validated all 56 tasks follow checklist format (checkbox + ID + optional [P] + optional [Story] + file paths)
  - Confirmed task organization by user story (US1, US2, US3)
  - Verified independent testability of each user story
  - Validated dependency graph (Phase 1 → Phase 2 → US1/US2/US3 in parallel → Polish)
---

## Prompt

```text
RAG Agent Backend

*File:* backend/app.py

### Tasks
- Initialize FastAPI, Qdrant client, Cohere, and OpenAI Agent.
- Implement retrieve_chunks(query) for semantic search.
- Create OpenAI Agent with a custom retrieval tool.
- Ensure the tool embeds the query and fetches top-k Qdrant results.
- Add /ask endpoint:
  - accept query
  - pass to agent
  - return grounded answer
- Add /health endpoint.
- Add verbose logging: query → retrieved chunks → final output.
- Ensure agent never answers without retrieval context.
```

## Response snapshot

Generated comprehensive task breakdown for Agent-Based RAG Backend (56 tasks) organized by user story to enable independent implementation and testing.

### Task Organization by User Story

**Phase 1: Setup (5 tasks)**:
- T001-T005: Project structure, dependencies, environment config, logging infrastructure

**Phase 2: Foundational (9 tasks)**:
- T006-T014: Pydantic models, Cohere embedding service, Qdrant retrieval service, error handling, FastAPI app, validators, formatters
- **Blocking**: Must complete before any user story work begins

**Phase 3: User Story 1 - Single-Turn Q&A (10 tasks) 🎯 MVP**:
- T015-T024: OpenAI agent initialization, retrieval tool, /ask endpoint, response tracking, confidence calculation, deflection logic, logging, /health endpoint
- **Goal**: User asks question → gets grounded answer with citations
- **Independent Test**: POST /ask with "What are the three main components of a robot?" → verify answer + citations
- **Success Criteria**: 0% hallucination, <3s response time, source citations, deflects out-of-scope

**Phase 4: User Story 2 - Multi-Turn Conversations (10 tasks)**:
- T025-T034: ConversationHistory entity, in-memory session storage, session retrieval/expiry, conversation context in agent calls, session_id handling
- **Goal**: Back-and-forth conversation with context maintenance
- **Independent Test**: "What is RL?" → "How is it used in robotics?" → verify context understanding
- **Success Criteria**: 5+ turn conversations, context-aware follow-ups, still deflects out-of-scope

**Phase 5: User Story 3 - Chapter-Scoped Queries (9 tasks)**:
- T035-T043: chapter_filter validation, section_filter validation, XOR logic, metadata filtering in Qdrant, error handling for invalid filters
- **Goal**: Questions limited to specific chapter/section
- **Independent Test**: POST with chapter_filter=3 → verify all sources from Chapter 3 only
- **Success Criteria**: 100% metadata filtering accuracy, clear errors for invalid filters

**Phase 6: Polish & Cross-Cutting (13 tasks)**:
- T044-T056: Request IDs, retry logic (Cohere, Qdrant, OpenAI), CORS, README, OpenAPI docs, edge cases, metrics, startup validation
- **Goal**: Production-ready backend with comprehensive error handling

### Task Format Compliance

**All 56 tasks follow strict checklist format**:
- ✅ Checkbox: `- [ ]` (markdown checkbox)
- ✅ Task ID: T001-T056 (sequential execution order)
- ✅ [P] marker: 21 tasks marked parallelizable (different files, no dependencies)
- ✅ [Story] label: 29 tasks labeled [US1], [US2], [US3] (user story mapping)
- ✅ File paths: Every task includes exact file path (backend/agent-rag/...)
- ✅ Descriptions: Clear actions with implementation details

**Examples**:
- `- [ ] T001 Create project directory structure: backend/agent-rag/ with subdirectories (models/, services/, api/, middleware/, utils/)`
- `- [ ] T003 [P] Create .env.example file with template environment variables...`
- `- [ ] T015 [P] [US1] Implement OpenAI Assistants agent initialization in backend/agent-rag/services/agent.py...`

### Dependency Graph

```
Phase 1 (Setup) → Phase 2 (Foundation - BLOCKING)
                        ↓
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
    Phase 3 (US1)   Phase 4 (US2)   Phase 5 (US3)
     [MVP - 10]      [10 tasks]       [9 tasks]
        ↓               ↓               ↓
        └───────────────┼───────────────┘
                        ↓
                  Phase 6 (Polish - 13)
```

**Critical Path**: Phase 1 → Phase 2 → Phase 3 (US1) = 24 tasks for MVP
**Parallelizable**: US2 and US3 can run in parallel after US1 core agent working (T018)

### Entity-to-User-Story Mapping

**User Story 1 (Single-Turn Q&A)**:
- Entities: Query (AskRequest), Chunk (read-only from Qdrant), RetrievalResult, AgentResponse, Citation, HealthStatus
- Endpoints: POST /ask, GET /health
- Services: embedding.py, vector.py, agent.py

**User Story 2 (Multi-Turn)**:
- Entities: ConversationHistory, Message (nested)
- Extends: POST /ask (add session_id handling)
- Services: session.py (new)

**User Story 3 (Chapter-Scoped)**:
- Entities: No new entities (uses Query filters)
- Extends: POST /ask (pass filters to vector retrieval)
- Services: vector.py (metadata filtering already in T009)

### Manual Testing Scenarios Provided

**Per User Story**:
- US1: 3 curl commands (simple question, out-of-scope, health check)
- US2: 3 curl commands (start conversation, follow-up, third turn)
- US3: 4 curl commands (chapter filter, section filter, invalid filter, both filters error)

### Implementation Strategy

**MVP Definition**: User Story 1 only (24 tasks: Phase 1-3)
- Delivers core value: single-turn grounded Q&A with citations
- Estimated: 8-12 hours for experienced Python/FastAPI developer

**Incremental Delivery**:
1. Sprint 1 (MVP): Phase 1-3 (T001-T024) → Single-turn Q&A
2. Sprint 2: Phase 4 (T025-T034) → Multi-turn conversations (+6-8 hours)
3. Sprint 3: Phase 5 (T035-T043) → Chapter/section filtering (+4-6 hours)
4. Sprint 4: Phase 6 (T044-T056) → Production polish (+6-8 hours)
5. **Total**: 24-34 hours full feature

**Parallel Execution**: After Phase 2 complete:
- Developer A: US1 (T015-T024)
- Developer B: US2 (T025-T034)
- Developer C: US3 (T035-T043)
- All can work independently

### Validation Results

**Format Validation**:
- ✅ Total tasks: 56 (matches count)
- ✅ Parallel tasks: 21 marked [P]
- ✅ US1 tasks: 10 labeled [US1]
- ✅ US2 tasks: 10 labeled [US2]
- ✅ US3 tasks: 9 labeled [US3]
- ✅ All checkboxes present: `- [ ]` format
- ✅ Sequential IDs: T001-T056
- ✅ File paths included: 100% of tasks have explicit paths

**Completeness Validation (User Story Coverage)**:
- ✅ US1 (Single-Turn): All FR-001 to FR-015 from spec.md covered
- ✅ US2 (Multi-Turn): FR-007 (conversation history), Session management covered
- ✅ US3 (Chapter-Scoped): FR-010 (metadata filtering) covered
- ✅ All endpoints: POST /ask, GET /health implemented
- ✅ All entities: 7 entities from data-model.md mapped to tasks
- ✅ All services: embedding.py, vector.py, agent.py, session.py implemented

**Independent Testability**:
- ✅ US1: Can test without US2 or US3 (single-turn Q&A standalone)
- ✅ US2: Can test without US3 (multi-turn conversations standalone, depends on US1)
- ✅ US3: Can test without US2 (chapter filtering standalone, depends on US1)
- ✅ Each story has manual test scenarios with curl commands

### User Feedback Integration

User provided high-level task list in prompt:
- ✅ "Initialize FastAPI, Qdrant client, Cohere, OpenAI Agent" → T001-T014 (setup + foundation)
- ✅ "Implement retrieve_chunks(query)" → T009 (vector.py), T016-T017 (retrieval tool)
- ✅ "Create OpenAI Agent with custom retrieval tool" → T015-T018 (agent.py)
- ✅ "Ensure tool embeds query and fetches top-k Qdrant results" → T008 (embedding), T009 (top-K=5), T017 (tool logic)
- ✅ "Add /ask endpoint" → T019 (ask.py endpoint)
- ✅ "Add /health endpoint" → T024 (health.py endpoint)
- ✅ "Add verbose logging" → T005 (logging infra), T023 (verbose ask logging), T034 (session logging)
- ✅ "Ensure agent never answers without retrieval context" → T018 (grounding system prompt), T022 (deflection logic)

All user-requested tasks expanded into concrete, file-specific implementation tasks.

## Outcome

- ✅ Impact: Complete implementation roadmap with 56 tasks organized by user story. MVP clearly defined (24 tasks). Parallel execution opportunities identified (21 tasks). Each user story independently testable.
- 🧪 Tests: Manual testing scenarios provided for each user story (curl commands). No automated test tasks generated (not requested in spec).
- 📁 Files: tasks.md (4,500+ lines with tasks, dependency graph, testing scenarios, implementation strategy)
- 🔁 Next prompts: Begin implementation with Phase 1 (T001-T005 setup), then Phase 2 (T006-T014 foundation), then Phase 3 MVP (T015-T024 US1)
- 🧠 Reflection: Task breakdown successfully maps all entities, endpoints, and requirements from planning artifacts to concrete implementation tasks. User story organization enables MVP delivery (US1 only) followed by incremental feature additions (US2, US3). Dependency graph clarifies critical path and parallelization opportunities.

## Evaluation notes (flywheel)

- Failure modes observed: None - all tasks generated successfully with proper format
- Graders run and results (PASS/FAIL):
  - Format Compliance: PASS (56/56 tasks have checkbox + ID + file paths, 21 marked [P], 29 labeled [US1/US2/US3])
  - User Story Coverage: PASS (all 3 user stories from spec.md mapped to task phases)
  - Entity Mapping: PASS (all 7 entities from data-model.md covered in tasks)
  - Endpoint Mapping: PASS (both endpoints from openapi.yaml implemented: POST /ask T019, GET /health T024)
  - Independent Testability: PASS (each user story has independent test criteria + manual curl commands)
  - MVP Definition: PASS (24 tasks for US1 clearly identified as MVP scope)
- Prompt variant (if applicable): N/A (standard /sp.tasks flow with user-provided high-level task list)
- Next experiment (smallest change to try): For future /sp.tasks commands, consider adding "Estimated Complexity" column to tasks (S/M/L/XL) to help developers estimate effort per task, not just per user story
