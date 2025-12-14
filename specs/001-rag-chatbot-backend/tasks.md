# Tasks: RAG Chatbot Backend

**Input**: Design documents from `/specs/001-rag-chatbot-backend/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: Tests are NOT explicitly requested in the spec, so this task list focuses on implementation only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

This project uses **web application structure** per plan.md:
- Backend: `backend/src/`, `backend/tests/`
- Paths below follow this structure

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend directory structure per plan.md (backend/src/, backend/tests/, backend/migrations/)
- [X] T002 Create backend/requirements.txt with all dependencies (FastAPI 0.104+, qdrant-client 1.7+, openai 1.3+, SQLAlchemy 2.0+, asyncpg 0.29+, pydantic 2.5+, pytest 7.4+)
- [X] T003 [P] Create backend/Dockerfile for containerized deployment
- [X] T004 [P] Create backend/.dockerignore file
- [X] T005 [P] Create backend/README.md with setup instructions from quickstart.md
- [X] T006 Initialize Python 3.11+ virtual environment and verify dependencies install
- [X] T007 [P] Configure pytest.ini for test discovery and async support
- [X] T008 [P] Create .gitignore entries for Python (already exists, verify backend/ coverage)

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T009 Create backend/src/config.py for environment variable loading and validation (loads .env, fails fast if required vars missing)
- [ ] T010 Create backend/src/main.py as FastAPI application entry point with app initialization
- [ ] T011 [P] Create backend/src/models/__init__.py module structure
- [ ] T012 [P] Create backend/src/services/__init__.py module structure
- [ ] T013 [P] Create backend/src/api/__init__.py module structure
- [ ] T014 [P] Create backend/src/middleware/__init__.py module structure
- [ ] T015 [P] Create backend/src/utils/__init__.py module structure
- [ ] T016 Implement CORS middleware in backend/src/middleware/cors.py (configurable origins from .env)
- [ ] T017 [P] Implement global error handler middleware in backend/src/middleware/error_handler.py
- [ ] T018 [P] Implement rate limiter middleware in backend/src/middleware/rate_limiter.py (100 req/min per user from .env)
- [ ] T019 [P] Create logger utility in backend/src/utils/logger.py with structured logging
- [ ] T020 [P] Create request ID generation utility in backend/src/utils/validators.py
- [ ] T021 Initialize Alembic for database migrations in backend/ (alembic init migrations)
- [ ] T022 Configure alembic.ini and backend/migrations/env.py for async SQLAlchemy
- [ ] T023 Create base SQLAlchemy models in backend/src/models/db_models.py (Base, timestamp mixins)
- [ ] T024 Define Pydantic base response schema in backend/src/models/response_models.py (includes request_id field)
- [ ] T025 Implement health check endpoint GET /api/health in backend/src/api/health.py (checks OpenAI, Qdrant, Neon connectivity)
- [ ] T026 Create Qdrant client connection utility in backend/src/utils/qdrant_client.py (async client, connection pooling)
- [ ] T027 Create Neon async database session factory in backend/src/utils/database.py (asyncpg + SQLAlchemy async engine)
- [ ] T028 Create OpenAI client wrapper in backend/src/utils/openai_client.py (with retry logic, error handling)

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Basic Book Q&A (Priority: P1) 🎯 MVP

**Goal**: Enable users to ask questions about book content and receive accurate answers with citations within 3 seconds

**Independent Test**: Load sample book content into Qdrant, ask "What is ROS 2?", verify answer contains relevant content with citations [1][2] and <3s response time

### Implementation for User Story 1

#### Database Models (Parallel)

- [ ] T029 [P] [US1] Create ChatSession model in backend/src/models/db_models.py (session_id, user_id, started_at, last_active_at, is_authenticated)
- [ ] T030 [P] [US1] Create ChatMessage model in backend/src/models/db_models.py (message_id, session_id, role, content, query_metadata, response_metadata, timestamp)
- [ ] T031 [P] [US1] Generate Alembic migration for ChatSession and ChatMessage tables
- [ ] T032 [US1] Apply migrations to create chat tables (alembic upgrade head)

#### Request/Response Models (Parallel)

- [ ] T033 [P] [US1] Create QueryRequest schema in backend/src/models/request_models.py (query, session_id, selected_text optional, chapter_scope optional)
- [ ] T034 [P] [US1] Create QueryResponse schema in backend/src/models/response_models.py (answer, sources, confidence_score, request_id, generation_time_ms)
- [ ] T035 [P] [US1] Create Source schema in backend/src/models/response_models.py (chunk_id, chapter_id, section_id, heading, similarity_score, page optional)

#### Text Chunking Utility

- [ ] T036 [US1] Implement recursive text chunker in backend/src/utils/chunking.py (400-600 tokens, 50-100 overlap, semantic boundaries per research.md)

#### Services (Sequential dependencies)

- [ ] T037 [US1] Implement EmbeddingService in backend/src/services/embedding_service.py (embed_text method using OpenAI text-embedding-3-small, batch support)
- [ ] T038 [US1] Implement VectorService in backend/src/services/vector_service.py (search_chunks method, filters by chapter/section metadata, returns top-K results)
- [ ] T039 [US1] Implement QueryService in backend/src/services/query_service.py (orchestrates: embed query → vector search → format context for GPT)
- [ ] T040 [US1] Implement response generation in QueryService (send retrieved chunks to GPT-4-turbo, enforce citations in prompt, handle "I don't know" case)
- [ ] T041 [US1] Implement ChatService in backend/src/services/chat_service.py (create_session, save_message methods for database persistence)

#### API Endpoint

- [ ] T042 [US1] Implement POST /api/query endpoint in backend/src/api/query.py (validates QueryRequest, calls QueryService, returns QueryResponse)
- [ ] T043 [US1] Add request ID tracing to /api/query endpoint
- [ ] T044 [US1] Add error handling for /api/query (OpenAI errors → 503, validation errors → 400, rate limit → 429)
- [ ] T045 [US1] Wire /api/query endpoint into FastAPI app in backend/src/main.py

#### Manual Verification

- [ ] T046 [US1] Create Qdrant collection "book_embeddings" with payload indexes (chapter_id, section_id) - run setup script
- [ ] T047 [US1] Load sample book chunks into Qdrant (at least Chapter 1 content) using embedding service
- [ ] T048 [US1] Manual test: Ask "What is ROS 2?" via /api/query and verify answer quality, citations, <3s latency

**Checkpoint**: At this point, User Story 1 (MVP) should be fully functional - basic Q&A works with citations

---

## Phase 4: User Story 2 - Selected Text Q&A (Priority: P2)

**Goal**: Enable users to ask questions about specific selected text, with system prioritizing that text in retrieval

**Independent Test**: Select a paragraph about "ROS 2 nodes", ask "How does this work?", verify answer focuses on selected text

**Dependencies**: Requires US1 (Basic Q&A) to be complete

### Implementation for User Story 2

- [ ] T049 [US2] Add selected_text handling to QueryService.process_query in backend/src/services/query_service.py (validate selected text exists in DB)
- [ ] T050 [US2] Implement context expansion logic in QueryService for short selections (<100 chars) - retrieve surrounding chunks
- [ ] T051 [US2] Implement selected text prioritization in VectorService.search_chunks - boost chunks matching selected text
- [ ] T052 [US2] Update QueryRequest validation in backend/src/models/request_models.py (selected_text max 5000 chars)
- [ ] T053 [US2] Manual test: Select text "rclpy.init(args=args)", ask "Explain this code", verify focused answer

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Chapter-Scoped Queries (Priority: P2)

**Goal**: Enable users to scope queries to specific chapters, retrieving only relevant chapter content

**Independent Test**: Ask "What is Gazebo?" with chapter_scope="chapter-2", verify only Chapter 2 content retrieved

**Dependencies**: Requires US1 (Basic Q&A) to be complete; independent of US2

### Implementation for User Story 3

- [ ] T054 [US3] Add chapter_scope filtering to VectorService.search_chunks in backend/src/services/vector_service.py (filter Qdrant payloadby chapter_id)
- [ ] T055 [US3] Update QueryRequest validation for chapter_scope format (pattern: ^chapter-\d+$)
- [ ] T056 [US3] Add "suggest checking other chapters" logic in QueryService when chapter-scoped query returns no results above threshold
- [ ] T057 [US3] Manual test: Ask question with chapter_scope="chapter-1", verify only Chapter 1 chunks retrieved

**Checkpoint**: User Stories 1, 2, AND 3 all work independently

---

## Phase 6: User Story 4 - Chat History Persistence (Priority: P3)

**Goal**: Persist chat sessions and messages for authenticated users across logout/login cycles

**Independent Test**: Create session, ask 3 questions, logout, login, verify chat history preserved

**Dependencies**: Requires US1 (Basic Q&A); uses ChatSession and ChatMessage models already created in US1

### Implementation for User Story 4

- [ ] T058 [US4] Implement ChatService.get_session_history method in backend/src/services/chat_service.py (retrieve messages for session_id, ordered by timestamp)
- [ ] T059 [US4] Implement anonymous session cleanup logic in ChatService (delete sessions with user_id=NULL after 24h inactivity)
- [ ] T060 [US4] Add session_id generation to POST /api/query if not provided (create new session)
- [ ] T061 [US4] Implement session linking to user_id when user is authenticated (extract from JWT token)
- [ ] T062 [US4] Add GET /api/sessions/{session_id}/history endpoint in backend/src/api/query.py (returns chat history)
- [ ] T063 [US4] Manual test: Ask questions, verify saved in database, test anonymous vs authenticated session persistence

**Checkpoint**: User Stories 1-4 all work independently

---

## Phase 7: User Story 5 - Response Quality Feedback (Priority: P3)

**Goal**: Enable users to rate chatbot responses (thumbs up/down) with optional text feedback for quality monitoring

**Independent Test**: Receive answer, submit thumbs down with feedback text, verify stored in database

**Dependencies**: Requires US1 (Basic Q&A) to provide responses to rate; independent of US2-US4

### Implementation for User Story 5

#### Database Model

- [ ] T064 [US5] Create Feedback model in backend/src/models/db_models.py (feedback_id, message_id FK, user_id, rating, feedback_text, timestamp)
- [ ] T065 [US5] Generate Alembic migration for Feedback table
- [ ] T066 [US5] Apply migration (alembic upgrade head)

#### Request/Response Models

- [ ] T067 [P] [US5] Create FeedbackRequest schema in backend/src/models/request_models.py (message_id, rating enum, feedback_text optional)
- [ ] T068 [P] [US5] Create FeedbackResponse schema in backend/src/models/response_models.py (success, feedback_id, request_id)

#### Service & Endpoint

- [ ] T069 [US5] Implement FeedbackService in backend/src/services/feedback_service.py (submit_feedback method, validate message exists and is assistant role)
- [ ] T070 [US5] Implement POST /api/feedback endpoint in backend/src/api/feedback.py (validates FeedbackRequest, calls FeedbackService)
- [ ] T071 [US5] Wire /api/feedback endpoint into FastAPI app in backend/src/main.py
- [ ] T072 [US5] Manual test: Submit feedback (thumbs_up and thumbs_down), verify database storage

**Checkpoint**: All 5 user stories are now fully functional

---

## Phase 8: Admin Content Embedding

**Purpose**: Enable admins to embed book content into Qdrant vector database

**Note**: This is not a user story but a required operational capability for the system to work

- [ ] T073 Create ChunkInput schema in backend/src/models/request_models.py (chapter_id, section_id, chunk_text, heading, page, chunk_index)
- [ ] T074 Create EmbedRequest schema in backend/src/models/request_models.py (chunks list of ChunkInput, max 500 items)
- [ ] T075 Create EmbedResponse schema in backend/src/models/response_models.py (success, chunks_processed, chunks_embedded, chunks_failed, failed_chunks, request_id, processing_time_ms)
- [ ] T076 Implement batch embedding in EmbeddingService.embed_batch method (100-500 chunks per OpenAI API call per research.md)
- [ ] T077 Implement VectorService.upsert_chunks method (insert/update chunks in Qdrant with metadata)
- [ ] T078 Implement admin authentication middleware in backend/src/middleware/admin_auth.py (validates X-Admin-API-Key header)
- [ ] T079 Implement POST /api/embed endpoint in backend/src/api/embed.py (validates EmbedRequest, batch embeds, upserts to Qdrant)
- [ ] T080 Apply admin authentication middleware to /api/embed endpoint
- [ ] T081 Wire /api/embed endpoint into FastAPI app in backend/src/main.py
- [ ] T082 Create embedding script backend/scripts/embed_book_content.py for batch processing book chapters
- [ ] T083 Manual test: Run embed script with sample chapter, verify chunks in Qdrant collection

---

## Phase 9: Streaming Responses (Enhancement)

**Purpose**: Implement Server-Sent Events (SSE) streaming for progressive answer display (referenced in spec FR-034)

- [ ] T084 Implement POST /api/query/stream endpoint in backend/src/api/query.py (returns SSE stream instead of JSON)
- [ ] T085 Modify QueryService to support streaming responses (yield chunks as they arrive from GPT-4)
- [ ] T086 Add SSE event formatting (data: {"type": "chunk", "content": "..."})
- [ ] T087 Wire /api/query/stream endpoint into FastAPI app
- [ ] T088 Manual test: Call /api/query/stream and verify progressive response chunks

---

## Phase 10: Observability & Monitoring

**Purpose**: Add logging, usage tracking, and monitoring capabilities

- [ ] T089 Create UsageLog model in backend/src/models/db_models.py (log_id, request_id, endpoint, user_id, session_id, http_method, status_code, response_time_ms, error_message, timestamp)
- [ ] T090 Generate Alembic migration for UsageLog table
- [ ] T091 Apply migration (alembic upgrade head)
- [ ] T092 Implement usage logging middleware in backend/src/middleware/usage_logger.py (logs all API requests)
- [ ] T093 Add usage logging middleware to FastAPI app
- [ ] T094 Add structured logging to all services (embedding, vector, query, chat, feedback) for debugging
- [ ] T095 Configure log levels and output format via .env (LOG_LEVEL variable)

---

## Phase 11: Error Resilience (Circuit Breaker Pattern)

**Purpose**: Implement circuit breaker pattern for external services per research.md recommendations

- [ ] T096 Implement circuit breaker utility in backend/src/utils/circuit_breaker.py (tracks failures, opens after 5 consecutive failures, 60s cooldown)
- [ ] T097 Wrap OpenAI API calls with circuit breaker in backend/src/utils/openai_client.py
- [ ] T098 Wrap Qdrant calls with circuit breaker in backend/src/utils/qdrant_client.py
- [ ] T099 Add exponential backoff retry logic (3 retries: 2s, 4s, 8s) to OpenAI and Qdrant clients
- [ ] T100 Implement graceful degradation in QueryService (return cached responses or fallback message when circuit open)

---

## Phase 12: Security Hardening

**Purpose**: Implement security measures from spec FR-036 to FR-044

- [ ] T101 Implement input sanitization in backend/src/utils/validators.py (prevent SQL injection, XSS)
- [ ] T102 Add query length validation (max 1000 chars) to QueryRequest schema
- [ ] T103 Add selected_text length validation (max 5000 chars) to QueryRequest schema
- [ ] T104 Implement prompt injection detection in backend/src/utils/validators.py (detect malicious patterns)
- [ ] T105 Add output filtering for GPT responses (validate against RAG Triad: context relevance, groundedness, answer relevance)
- [ ] T106 Verify API keys never appear in logs or error messages (audit all error handlers)
- [ ] T107 Verify database credentials use SSL/TLS (check DATABASE_URL has sslmode=require)
- [ ] T108 Set secure file permissions on .env file (chmod 600)

---

## Phase 13: Polish & Cross-Cutting Concerns

**Purpose**: Final touches for production readiness

- [ ] T109 [P] Add API documentation to all endpoints using FastAPI docstrings (visible in /docs)
- [ ] T110 [P] Verify all endpoints return consistent error response format (ErrorResponse schema)
- [ ] T111 [P] Add request/response examples to OpenAPI schema for all endpoints
- [ ] T112 Implement /api/health status aggregation logic (healthy if all services up, degraded if 1 service down, unhealthy if 2+ down)
- [ ] T113 Configure CORS origins from .env (production domains)
- [ ] T114 Verify rate limiting works correctly (test 100 req/min limit)
- [ ] T115 Add API versioning to all endpoints (e.g., /api/v1/query)
- [ ] T116 Create production Dockerfile with multi-stage build (reduce image size)
- [ ] T117 Create docker-compose.yml for local development (backend + optional Postgres container)
- [ ] T118 Update backend/README.md with deployment instructions for Railway/Vercel
- [ ] T119 Verify all environment variables documented in .env.example
- [ ] T120 Run security audit (pip install safety && safety check on requirements.txt)

---

## Dependencies Between Phases

```
Phase 1 (Setup)
  ↓
Phase 2 (Foundational) ← MUST complete before any user story
  ↓
  ├→ Phase 3 (US1: Basic Q&A) 🎯 MVP ← HIGHEST PRIORITY
  │    ↓
  │    ├→ Phase 4 (US2: Selected Text) ← Depends on US1
  │    ├→ Phase 5 (US3: Chapter Scoping) ← Depends on US1
  │    ├→ Phase 6 (US4: Chat History) ← Depends on US1
  │    └→ Phase 7 (US5: Feedback) ← Depends on US1
  │
  ├→ Phase 8 (Admin Embedding) ← Required for US1 to have data
  ├→ Phase 9 (Streaming) ← Enhancement, can add anytime after US1
  ├→ Phase 10 (Observability) ← Can run in parallel with user stories
  ├→ Phase 11 (Error Resilience) ← Can add after US1 MVP
  ├→ Phase 12 (Security) ← Can run in parallel with user stories
  └→ Phase 13 (Polish) ← Final phase after all features
```

**Critical Path for MVP**:
1. Phase 1 (Setup) → Phase 2 (Foundational) → Phase 8 (Admin Embedding) → Phase 3 (US1) = **Minimum Viable Product**

**User Story Independence**:
- US2, US3, US4, US5 can all be implemented in parallel after US1 completes
- US2 and US3 are independent of each other (both only depend on US1)
- US4 reuses models from US1 (ChatSession, ChatMessage already created)
- US5 is completely independent (new Feedback table)

---

## Parallel Execution Opportunities

### Phase 1 (Setup) - Parallelizable
- T003 (Dockerfile), T004 (.dockerignore), T005 (README), T007 (pytest.ini), T008 (.gitignore) can all run in parallel

### Phase 2 (Foundational) - Parallelizable
- T011-T015 (create module __init__ files) - 5 tasks in parallel
- T016-T018 (middleware: CORS, error handler, rate limiter) - 3 tasks in parallel
- T019-T020 (utilities: logger, validators) - 2 tasks in parallel

### Phase 3 (US1) - Parallelizable
- T029-T030 (database models) - 2 tasks in parallel
- T033-T035 (Pydantic schemas) - 3 tasks in parallel
- After schemas done, services can be built sequentially

### Phase 4-7 (US2-US5) - **ALL PARALLELIZABLE**
- Once US1 completes, US2, US3, US4, US5 can all be implemented by different developers simultaneously

### Phase 10-12 (Observability, Resilience, Security) - Parallelizable
- Can run in parallel with user story development

---

## Testing Strategy (Manual Verification)

Since automated tests were not requested in the spec, each user story includes manual verification tasks:

- **US1 (T048)**: Ask "What is ROS 2?", verify answer quality, citations, <3s latency
- **US2 (T053)**: Select code snippet, ask question, verify focused answer
- **US3 (T057)**: Query with chapter scope, verify only that chapter's content used
- **US4 (T063)**: Test session persistence across logout/login
- **US5 (T072)**: Submit feedback, verify database storage
- **Admin (T083)**: Embed sample chapter, verify chunks in Qdrant

**Integration Testing**: Test complete flow end-to-end:
1. Embed Chapter 1 content (T083)
2. Ask question about Chapter 1 (T048)
3. Submit feedback on answer (T072)
4. Verify chat history persisted (T063)

---

## Implementation Strategy

### MVP Scope (Recommended First Delivery)

**Minimum Viable Product** = Phase 1 + Phase 2 + Phase 8 + Phase 3 (US1 only)

This gives you:
- ✅ Working FastAPI backend
- ✅ Qdrant vector database integration
- ✅ OpenAI embeddings and GPT-4 generation
- ✅ Basic question-answering with citations
- ✅ Admin endpoint to load book content
- ✅ Health check endpoint

**Estimated Tasks**: T001-T048 = **48 tasks**

### Incremental Delivery

After MVP (US1), deliver user stories in priority order:
1. **US2 (Selected Text)** - Phase 4 (T049-T053) = 5 tasks
2. **US3 (Chapter Scoping)** - Phase 5 (T054-T057) = 4 tasks
3. **US4 (Chat History)** - Phase 6 (T058-T063) = 6 tasks
4. **US5 (Feedback)** - Phase 7 (T064-T072) = 9 tasks

Then add enhancements and polish:
- Streaming responses (Phase 9)
- Observability (Phase 10)
- Error resilience (Phase 11)
- Security hardening (Phase 12)
- Final polish (Phase 13)

---

## Task Summary

**Total Tasks**: 120

**Breakdown by Phase**:
- Phase 1 (Setup): 8 tasks
- Phase 2 (Foundational): 20 tasks
- Phase 3 (US1 - MVP): 20 tasks
- Phase 4 (US2): 5 tasks
- Phase 5 (US3): 4 tasks
- Phase 6 (US4): 6 tasks
- Phase 7 (US5): 9 tasks
- Phase 8 (Admin Embedding): 11 tasks
- Phase 9 (Streaming): 5 tasks
- Phase 10 (Observability): 7 tasks
- Phase 11 (Error Resilience): 5 tasks
- Phase 12 (Security): 8 tasks
- Phase 13 (Polish): 12 tasks

**MVP Task Count**: 48 tasks (Phases 1, 2, 3, 8)
**Full Feature Set**: 72 tasks (MVP + US2-US5)
**Production Ready**: 120 tasks (all phases)

**Parallelization**: ~30% of tasks marked [P] for parallel execution

---

**Status**: ✅ Tasks ready for implementation. Proceed with `/sp.implement` or begin manual implementation starting with Phase 1.
