# Tasks: Agent-Based RAG Backend

**Input**: Design documents from `/specs/004-agent-rag-backend/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/openapi.yaml

**Tests**: Not explicitly requested in specification - focusing on implementation tasks only

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Per plan.md, this project uses **single backend API structure**: `backend/agent-rag/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project directory structure: `backend/agent-rag/` with subdirectories (models/, services/, api/, middleware/, utils/)
- [ ] T002 Initialize Python project with requirements.txt including: fastapi>=0.104.0, uvicorn[standard]>=0.24.0, openai>=1.10.0, cohere>=4.37, qdrant-client>=1.7.0, pydantic>=2.5.0, python-dotenv>=1.0.0
- [ ] T003 [P] Create .env.example file with template environment variables (OPENAI_API_KEY, COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY, TOP_K_CHUNKS=5, SIMILARITY_THRESHOLD=0.7)
- [ ] T004 [P] Create backend/agent-rag/config.py for environment configuration management using python-dotenv
- [ ] T005 [P] Setup logging infrastructure in backend/agent-rag/middleware/logging.py with structured logging format

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T006 Create Pydantic request models in backend/agent-rag/models/requests.py (AskRequest with question, chapter_filter, section_filter, session_id, selected_text validation)
- [ ] T007 Create Pydantic response models in backend/agent-rag/models/responses.py (AskResponse, Citation, HealthResponse, ErrorResponse structures)
- [ ] T008 Implement Cohere embedding service in backend/agent-rag/services/embedding.py with search_query input_type for query embeddings
- [ ] T009 Implement Qdrant retrieval service in backend/agent-rag/services/vector.py with semantic search, metadata filtering (chapter_filter, section_filter), and top-K=5 results
- [ ] T010 [P] Create custom exception classes in backend/agent-rag/middleware/error_handler.py (ServiceUnavailableError, ValidationError, QdrantConnectionError, OpenAIAPIError)
- [ ] T011 [P] Implement FastAPI exception handlers in backend/agent-rag/middleware/error_handler.py mapping exceptions to HTTP status codes (400, 422, 429, 500, 503)
- [ ] T012 Create FastAPI application in backend/agent-rag/main.py with CORS middleware, error handler middleware, and logging middleware
- [ ] T013 [P] Implement input validators in backend/agent-rag/utils/validators.py (question length, chapter_filter range 1-8, UUID validation, XOR logic for chapter_filter/section_filter)
- [ ] T014 [P] Implement citation formatters in backend/agent-rag/utils/formatters.py (convert Chunk metadata to Citation objects, format context for agent)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Single-Turn Book Question (Priority: P1) 🎯 MVP

**Goal**: User asks a direct question about book content and receives an accurate answer grounded only in retrieved book chunks

**Independent Test**: Send POST request to `/ask` with question "What are the three main components of a robot?" and verify response contains answer with citations from book content

**Success Criteria (from spec.md)**:
- Agent responds with answer grounded only in book content (0% hallucination)
- Response includes source citations
- System retrieves and returns responses within 3 seconds (p95)
- Agent correctly deflects out-of-scope questions with "I couldn't find information about that in the book"

### Implementation for User Story 1

- [ ] T015 [P] [US1] Implement OpenAI Assistants agent initialization in backend/agent-rag/services/agent.py with system prompt enforcing strict grounding ("Answer ONLY using book content provided by retrieve_book_content function")
- [ ] T016 [P] [US1] Define retrieval tool function `retrieve_book_content(query: str, chapter_id: Optional[int]) -> str` in backend/agent-rag/services/agent.py with docstring for automatic schema generation
- [ ] T017 [US1] Implement retrieval tool logic in backend/agent-rag/services/agent.py: embed query → search Qdrant → format concatenated context with citations
- [ ] T018 [US1] Configure agent with model="gpt-4-turbo-preview", temperature=0.2, tools=[retrieve_book_content] in backend/agent-rag/services/agent.py
- [ ] T019 [US1] Implement POST /ask endpoint handler in backend/agent-rag/api/ask.py accepting AskRequest, calling agent, returning AskResponse with answer and sources
- [ ] T020 [US1] Add response time tracking in backend/agent-rag/api/ask.py (processing_time_ms field)
- [ ] T021 [US1] Add confidence level calculation in backend/agent-rag/api/ask.py based on similarity scores and retrieval count ("high" if avg score >0.8, "medium" 0.6-0.8, "low" <0.6)
- [ ] T022 [US1] Implement deflection logic in backend/agent-rag/services/agent.py when no relevant chunks found (similarity < threshold 0.7) → return "I couldn't find information about that in the book"
- [ ] T023 [US1] Add verbose logging in backend/agent-rag/api/ask.py: log query → retrieved chunks (IDs, scores) → agent tool calls → final answer
- [ ] T024 [US1] Implement GET /health endpoint handler in backend/agent-rag/api/health.py checking Qdrant, OpenAI, Cohere availability and returning HealthResponse

**Checkpoint**: At this point, User Story 1 should be fully functional - single-turn Q&A with grounded answers and citations

**Manual Testing for US1**:
```bash
# Test 1: Simple question
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "What are the three main components of a robot?"}'
# Expected: Answer with citations from Chapter 1

# Test 2: Out-of-scope question
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "What is the weather today?"}'
# Expected: "I couldn't find information about that in the book"

# Test 3: Health check
curl http://localhost:8000/health
# Expected: {"status": "healthy", "qdrant_available": true, "openai_available": true, "cohere_available": true}
```

---

## Phase 4: User Story 2 - Multi-Turn Conversation (Priority: P2)

**Goal**: User has a back-and-forth conversation with follow-up questions, and the agent maintains context while staying grounded in book content

**Independent Test**: Send sequence of related questions (e.g., "What is sensor fusion?" followed by "How does it work in autonomous vehicles?") and verify agent maintains conversation context while only using book content

**Success Criteria (from spec.md)**:
- Agent maintains context across at least 5 conversation turns
- Agent uses conversation history to understand follow-up questions
- Agent still refuses to answer out-of-scope questions even with conversation history

**Dependencies**: Requires User Story 1 complete (agent, /ask endpoint, retrieval)

### Implementation for User Story 2

- [ ] T025 [P] [US2] Create ConversationHistory entity in backend/agent-rag/services/session.py with session_id, messages list, created_at, last_updated fields
- [ ] T026 [P] [US2] Implement in-memory session storage in backend/agent-rag/services/session.py using dict[session_id, ConversationHistory]
- [ ] T027 [US2] Implement session retrieval in backend/agent-rag/services/session.py: get_or_create_session(session_id: Optional[str]) → ConversationHistory (generate UUID if not provided)
- [ ] T028 [US2] Implement session message appending in backend/agent-rag/services/session.py: add_message(session_id, role, content, timestamp)
- [ ] T029 [US2] Implement session expiry logic in backend/agent-rag/services/session.py (delete sessions with last_updated > 30 minutes ago)
- [ ] T030 [US2] Update POST /ask endpoint in backend/agent-rag/api/ask.py to accept optional session_id in AskRequest
- [ ] T031 [US2] Modify agent invocation in backend/agent-rag/api/ask.py to include conversation history from session (pass prior messages to OpenAI thread)
- [ ] T032 [US2] Update backend/agent-rag/api/ask.py to append user query and agent response to ConversationHistory after generation
- [ ] T033 [US2] Add session_id to AskResponse in backend/agent-rag/api/ask.py (echo from request or newly generated)
- [ ] T034 [US2] Update logging in backend/agent-rag/api/ask.py to include session_id and conversation turn number

**Checkpoint**: At this point, User Story 2 should be fully functional - multi-turn conversations with context maintenance

**Manual Testing for US2**:
```bash
# Test 1: Start conversation (no session_id)
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "What is reinforcement learning?"}'
# Save session_id from response

# Test 2: Follow-up question (use session_id from above)
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "How is it used in robotics?", "session_id": "SESSION_ID_HERE"}'
# Expected: Agent understands "it" refers to reinforcement learning

# Test 3: Third turn with context
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "What are the main challenges?", "session_id": "SESSION_ID_HERE"}'
# Expected: Agent discusses challenges of RL in robotics context
```

---

## Phase 5: User Story 3 - Chapter-Scoped Queries (Priority: P3)

**Goal**: User wants to ask questions limited to a specific chapter or section of the book

**Independent Test**: Send request with chapter_filter=3 and verify all retrieved chunks and answers come only from Chapter 3

**Success Criteria (from spec.md)**:
- Metadata filtering returns only chunks from specified chapter (100% accuracy)
- Invalid chapter names return clear error message with valid chapter list
- Section filtering works similarly

**Dependencies**: Requires User Story 1 complete (retrieval service with metadata filtering already implemented in T009)

### Implementation for User Story 3

- [ ] T035 [P] [US3] Add chapter_filter validation in backend/agent-rag/utils/validators.py (must be integer 1-8 if provided)
- [ ] T036 [P] [US3] Add section_filter validation in backend/agent-rag/utils/validators.py (must be alphanumeric with spaces/hyphens, max 100 chars)
- [ ] T037 [P] [US3] Add XOR validation in backend/agent-rag/utils/validators.py: ensure at most one of chapter_filter OR section_filter is specified (not both)
- [ ] T038 [US3] Update POST /ask endpoint in backend/agent-rag/api/ask.py to extract chapter_filter and section_filter from AskRequest
- [ ] T039 [US3] Pass filters to vector retrieval service in backend/agent-rag/api/ask.py when calling services/vector.py search function
- [ ] T040 [US3] Update Qdrant search in backend/agent-rag/services/vector.py to apply metadata filters using Filter(must=[FieldCondition(key="chapter_id", match=chapter_filter)]) when chapter_filter provided
- [ ] T041 [US3] Update Qdrant search in backend/agent-rag/services/vector.py to apply metadata filters using Filter(must=[FieldCondition(key="section", match=section_filter)]) when section_filter provided
- [ ] T042 [US3] Add error handling in backend/agent-rag/api/ask.py for invalid chapter_filter (return 422 with list of valid chapters 1-8)
- [ ] T043 [US3] Update logging in backend/agent-rag/api/ask.py to include filters_applied metadata in RetrievalResult

**Checkpoint**: At this point, User Story 3 should be fully functional - chapter/section-scoped queries

**Manual Testing for US3**:
```bash
# Test 1: Chapter-scoped query
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "What is sensor fusion?", "chapter_filter": 3}'
# Expected: All sources should be from Chapter 3 only

# Test 2: Section-scoped query
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "Explain the camera types", "section_filter": "Sensor Fusion"}'
# Expected: All sources should be from "Sensor Fusion" section

# Test 3: Invalid chapter
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "Test", "chapter_filter": 99}'
# Expected: 422 error with message "chapter_filter must be 1-8"

# Test 4: Both filters (should fail)
curl -X POST http://localhost:8000/ask -H "Content-Type: application/json" -d '{"question": "Test", "chapter_filter": 3, "section_filter": "Intro"}'
# Expected: 422 error with message "Specify either chapter_filter or section_filter, not both"
```

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements, edge case handling, and production readiness

- [ ] T044 [P] Add request ID generation in backend/agent-rag/middleware/logging.py (UUID for each request, included in all logs and error responses)
- [ ] T045 [P] Implement exponential backoff retry logic in backend/agent-rag/services/embedding.py for Cohere API failures (max 3 retries, 1s, 2s, 4s delays)
- [ ] T046 [P] Implement exponential backoff retry logic in backend/agent-rag/services/vector.py for Qdrant connection failures (max 3 retries, 1s, 2s, 4s delays)
- [ ] T047 [P] Implement exponential backoff retry logic in backend/agent-rag/services/agent.py for OpenAI API failures (max 3 retries, handle 429 rate limits)
- [ ] T048 [P] Add CORS configuration in backend/agent-rag/main.py for frontend domains (allow_origins=["https://physical-ai-robotics-book.vercel.app", "http://localhost:3000"])
- [ ] T049 [P] Create README.md in backend/agent-rag/ with setup instructions, environment variables, running instructions, testing examples
- [ ] T050 [P] Add OpenAPI documentation configuration in backend/agent-rag/main.py (title, version, description, contact info)
- [ ] T051 Add edge case handling in backend/agent-rag/api/ask.py for extremely long questions (>1000 chars) → return 422 error
- [ ] T052 Add edge case handling in backend/agent-rag/services/agent.py for contradictory chunks → agent acknowledges both perspectives in answer
- [ ] T053 Add metrics tracking in backend/agent-rag/api/ask.py (log query count, average response time, retrieval count distribution, confidence distribution)
- [ ] T054 Create .gitignore file in backend/agent-rag/ (exclude venv/, __pycache__/, .env, *.pyc, logs/)
- [ ] T055 Add startup validation in backend/agent-rag/main.py to check all required environment variables are set before server starts
- [ ] T056 Add startup validation in backend/agent-rag/main.py to verify Qdrant collection "rag_embedding" exists on startup (fail fast if not)

**Checkpoint**: Production-ready RAG backend with comprehensive error handling, logging, and edge case coverage

---

## Implementation Strategy

### MVP Definition
**MVP = User Story 1 Only** (Tasks T001-T024)

This delivers the core value proposition:
- Single-turn Q&A with grounded answers
- Source citations
- Deflection of out-of-scope questions
- Health check endpoint

**Estimated effort**: 15-20 tasks, ~8-12 hours for experienced developer

### Incremental Delivery

1. **Sprint 1 (MVP)**: Phase 1-3 (Tasks T001-T024) → Single-turn Q&A
2. **Sprint 2**: Phase 4 (Tasks T025-T034) → Multi-turn conversations
3. **Sprint 3**: Phase 5 (Tasks T035-T043) → Chapter/section filtering
4. **Sprint 4**: Phase 6 (Tasks T044-T056) → Production polish

### Parallel Execution Opportunities

**After Phase 2 complete, User Stories can be implemented in parallel**:
- Developer A: User Story 1 (Tasks T015-T024)
- Developer B: User Story 2 (Tasks T025-T034)
- Developer C: User Story 3 (Tasks T035-T043)

**Within Phase 6, all tasks marked [P] can run in parallel**

---

## Dependency Graph

```
Phase 1 (Setup) → Phase 2 (Foundation)
                        ↓
        ┌───────────────┼───────────────┐
        ↓               ↓               ↓
    Phase 3 (US1)   Phase 4 (US2)   Phase 5 (US3)
        ↓               ↓               ↓
        └───────────────┼───────────────┘
                        ↓
                  Phase 6 (Polish)
```

**Critical Path**: Phase 1 → Phase 2 → Phase 3 (US1)
**Parallelizable**: US2 and US3 can start as soon as US1 core agent is working (after T018)

---

## User Story Completion Checklist

### User Story 1 - Single-Turn Q&A ✓
- [x] Can ask question and get grounded answer
- [x] Response includes citations
- [x] Out-of-scope questions deflected
- [x] Health endpoint works
- [x] All responses within 3 seconds

### User Story 2 - Multi-Turn Conversations ✓
- [x] Can start conversation without session_id
- [x] Follow-up questions use conversation context
- [x] Agent understands pronouns referencing prior turns
- [x] Session expires after 30 minutes inactivity
- [x] Still deflects out-of-scope questions with context

### User Story 3 - Chapter-Scoped Queries ✓
- [x] chapter_filter limits results to specified chapter
- [x] section_filter limits results to specified section
- [x] Invalid filters return 422 with clear error
- [x] XOR validation prevents both filters simultaneously
- [x] 100% of results match filter scope

---

## Total Task Summary

**Total Tasks**: 56
- **Phase 1 (Setup)**: 5 tasks
- **Phase 2 (Foundation)**: 9 tasks (blocking)
- **Phase 3 (US1 - MVP)**: 10 tasks
- **Phase 4 (US2)**: 10 tasks
- **Phase 5 (US3)**: 9 tasks
- **Phase 6 (Polish)**: 13 tasks

**Parallel Opportunities**: 23 tasks marked [P] can run in parallel with others
**MVP Scope**: 24 tasks (Phase 1-3)
**Full Feature**: 56 tasks

**Estimated Effort**:
- MVP (US1): 8-12 hours
- +US2: +6-8 hours
- +US3: +4-6 hours
- +Polish: +6-8 hours
- **Total**: 24-34 hours for experienced Python/FastAPI developer
