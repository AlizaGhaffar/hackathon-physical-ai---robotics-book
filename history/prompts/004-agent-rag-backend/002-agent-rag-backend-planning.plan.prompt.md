---
id: 002
title: Agent-Based RAG Backend Planning
stage: plan
date: 2025-12-13
surface: agent
model: claude-sonnet-4-5-20250929
feature: 004-agent-rag-backend
branch: 004-agent-rag-backend
user: AlizaGhaffar
command: /sp.plan
labels: ["rag", "architecture", "openai-agents", "qdrant", "fastapi", "planning"]
links:
  spec: specs/004-agent-rag-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/004-agent-rag-backend/plan.md
  - specs/004-agent-rag-backend/research.md
  - specs/004-agent-rag-backend/data-model.md
  - specs/004-agent-rag-backend/contracts/openapi.yaml
  - specs/004-agent-rag-backend/quickstart.md
  - CLAUDE.md (agent context updated)
tests:
  - Validated constitution compliance (RAG Backend Architecture sections)
  - Confirmed OpenAI Assistants API function calling pattern
  - Verified Qdrant metadata filtering approach
---

## Prompt

```
Agent + FastAPI Plan

### Plan Summary
Create a retrieval-augmented agent using OpenAI Agents SDK and expose it via FastAPI.

### Steps
1. Create `backend/app.py` for FastAPI server.
2. Initialize Qdrant + Cohere (for query embeddings).
3. Implement a retrieval utility to query Qdrant (top-k = 5).
4. Register a custom "retrieval tool" in the OpenAI Agent.
5. Inside the tool:
   - embed query
   - run Qdrant search
   - return concatenated context
6. Configure the agent:
   - system message: "Answer using only retrieved content"
   - low temperature for factual consistency
7. Create `/ask` POST endpoint:
   - accept user query
   - send query to the agent
   - return final grounded answer
8. Add `/health` GET endpoint.
9. Add logging for queries, retrieved chunks, response.
10. Test with 5–10 questions from the deployed book.

### Deliverables
- `app.py` FastAPI server
- Working RAG agent with retrieval tool
- Verified grounded answers
```

## Response snapshot

Created comprehensive implementation plan for Agent-Based RAG Backend (feature 004-agent-rag-backend) with complete Phase 0 research and Phase 1 design artifacts.

### Planning Artifacts Generated

**1. plan.md - Implementation Plan**:
- **Summary**: FastAPI backend integrating OpenAI Agents SDK with Qdrant for grounded Q&A
- **Technical Context**: Python 3.11+, FastAPI, OpenAI SDK, Cohere (embeddings), Qdrant
- **Constitution Check**: ✅ APPROVED (1 justified deviation: Neon Postgres deferred to post-MVP per spec assumption #3)
- **Project Structure**: `backend/agent-rag/` with services (embedding, vector, agent, session), API (ask, health), models (Pydantic), middleware (error handling, logging)
- **Complexity Tracking**: In-memory session management justified vs. Postgres (aligns with spec)

**2. research.md - Phase 0 Research**:
- **OpenAI Agents SDK**: Use Assistants API with function calling for retrieval tool (rejected LangChain, custom loops)
- **Cohere Embeddings**: `search_query` input_type for queries (asymmetric matching)
- **Qdrant Filtering**: Metadata `must` filters at query time (no separate collections)
- **FastAPI Errors**: Custom exception handlers mapping to HTTP status codes (503 for service unavailable, 422 for validation)
- **Hallucination Prevention**: Multi-layered approach (explicit system prompt + temp=0.2 + output format + tool validation)

**3. data-model.md - Entity Models**:
- **Query**: question, chapter_filter, section_filter, session_id, selected_text
- **Chunk**: id, text, embedding (1024-dim), chapter_id, section, page, source_url (read-only from feature 002)
- **RetrievalResult**: chunks, scores, query_embedding, filters_applied
- **AgentResponse**: answer, sources (Citation[]), confidence, session_id, retrieval_count, processing_time_ms
- **Citation**: chapter_id, section, page, chunk_id, source_url, relevance_score
- **ConversationHistory**: session_id, messages (Message[]), created_at, last_updated (in-memory, 30min expiry)
- **HealthStatus**: status, service availability flags, timestamp, uptime
- **Data Flows**: Single-turn query, multi-turn conversation, health check

**4. contracts/openapi.yaml - API Contract**:
- **POST /ask**: Submit question with optional filters/session_id → AskResponse (answer + sources)
- **GET /health**: Health check → HealthResponse (service availability + status)
- **Request Models**: AskRequest (question, chapter_filter, section_filter, session_id, selected_text)
- **Response Models**: AskResponse, Citation, HealthResponse, ErrorResponse, ValidationErrorResponse
- **HTTP Status Codes**: 200 (success), 400 (bad request), 422 (validation), 429 (rate limit), 500 (internal), 503 (service unavailable)
- **Examples**: grounded_answer, deflection, chapter_scoped, multi_turn

**5. quickstart.md - Developer Guide**:
- **Prerequisites**: Python 3.11+, OpenAI/Cohere/Qdrant API keys, pre-populated `rag_embedding` collection
- **Setup Steps**: venv creation, pip install, .env configuration, Qdrant verification
- **Running**: `uvicorn main:app --reload --host 0.0.0.0 --port 8000`
- **Testing**: Health check, simple question, chapter-scoped, multi-turn conversation
- **Troubleshooting**: Qdrant connection, missing collection, OpenAI rate limits, validation errors
- **Production Checklist**: Environment vars, logging, CORS, rate limiting, error sanitization

**6. Agent Context Updated (CLAUDE.md)**:
- Added Python 3.11+ to Active Technologies
- Updated project knowledge for feature 004-agent-rag-backend

### Constitution Compliance

**All RAG-specific sections validated**:
- ✅ RAG Backend Architecture (Section XIII): Clean service separation, FastAPI, stateless design
- ✅ Embedding Pipeline Excellence (Section XIV): Cohere embed-english-v3.0 consistency
- ✅ Vector Storage Strategy (Section XV): Qdrant namespacing, metadata filtering
- ✅ Context Retrieval Precision (Section XVI): Top-K=5, threshold=0.7, chapter scoping
- ✅ Response Generation Quality (Section XVII): OpenAI Agents SDK, grounding prompt, citations
- ✅ Technology Stack: FastAPI, OpenAI Agents SDK, Qdrant (all per hackathon requirements)
- ✅ Performance: <3s p95 response time, 50 concurrent users
- ✅ Security: API keys in env vars, input validation, no secrets in logs

**Justified Deviation**:
- Neon Postgres deferred to post-MVP (spec assumption #3: "in-memory session management for a single session")

### Architecture Decisions

**Key decisions documented in research.md**:

1. **Agent Pattern**: OpenAI Assistants API with function calling
   - Tool: `retrieve_book_content(query, chapter_id)`
   - Enables controlled grounding (agent must call tool)
   - Rejected: LangChain (over-abstraction), direct GPT-4 (no tool calling), custom loop (reinventing wheel)

2. **Query Embeddings**: Cohere `search_query` input_type
   - Asymmetric matching (query vs. document embeddings)
   - 5-10% accuracy improvement vs. symmetric
   - Rejected: OpenAI embeddings (would require re-embedding feature 002 corpus)

3. **Metadata Filtering**: Qdrant `must` filters at query time
   - Single collection with metadata filtering (efficient)
   - Rejected: Separate collections per chapter (maintenance burden), post-search filtering (inefficient)

4. **Error Handling**: FastAPI exception handlers
   - Custom exceptions → HTTP status codes (503 service unavailable, 422 validation)
   - Rejected: Generic 500s (loses context), 200 with error body (violates HTTP semantics)

5. **Hallucination Prevention**: Multi-layered
   - Explicit "ONLY use provided content" system prompt
   - Temperature=0.2 (low but not robotic)
   - Structured output (answer + sources)
   - Tool response validation (non-empty check)

### Project Structure

```
backend/agent-rag/
├── main.py                   # FastAPI entrypoint
├── config.py                 # Environment configuration
├── models/
│   ├── requests.py          # Pydantic request models
│   └── responses.py         # Pydantic response models
├── services/
│   ├── embedding.py         # Cohere embedding generation
│   ├── vector.py            # Qdrant retrieval
│   ├── agent.py             # OpenAI Agents SDK integration
│   └── session.py           # In-memory session management
├── api/
│   ├── ask.py               # POST /ask handler
│   └── health.py            # GET /health handler
├── middleware/
│   ├── error_handler.py     # Centralized error handling
│   └── logging.py           # Request/response logging
└── utils/
    ├── validators.py        # Input validation
    └── formatters.py        # Citation formatting

tests/
├── unit/                     # Mocked external services
├── integration/              # End-to-end with real services
└── contract/                 # OpenAPI schema validation
```

### Next Steps

**Phase 2**: Generate `tasks.md` with `/sp.tasks` command:
- Break down implementation into testable tasks
- Define red-green-refactor cycles
- Create acceptance test cases per functional requirement

**Recommended ADR**: This planning phase identified several architecturally significant decisions that may warrant ADRs:

📋 **Architectural decision detected**: OpenAI Assistants API function calling for RAG grounding vs. LangChain or custom agent loops
   Document reasoning and tradeoffs? Run `/sp.adr openai-agents-rag-pattern`

📋 **Architectural decision detected**: In-memory session management (MVP) vs. Neon Postgres persistent storage
   Document reasoning and tradeoffs? Run `/sp.adr session-storage-strategy`

## Outcome

- ✅ Impact: Comprehensive architecture plan ready for implementation. All research questions resolved, all entities modeled, complete API contract defined.
- 🧪 Tests: No code tests yet (planning phase). Contract tests defined in OpenAPI spec.
- 📁 Files: plan.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md, CLAUDE.md (updated)
- 🔁 Next prompts: `/sp.tasks` (generate implementation tasks), `/sp.adr openai-agents-rag-pattern` (document agent pattern decision), `/sp.adr session-storage-strategy` (document storage decision)
- 🧠 Reflection: Planning phase successfully separated concerns (embedding, vector, agent, API) and validated against constitution. All unknowns from spec (OpenAI Agents SDK integration, Cohere/Qdrant patterns, hallucination prevention) resolved through research. Ready for task breakdown.

## Evaluation notes (flywheel)

- Failure modes observed: None - all planning artifacts generated successfully
- Graders run and results (PASS/FAIL):
  - Constitution Check: PASS (1 justified deviation documented)
  - Research Completeness: PASS (5 research tasks, all NEEDS CLARIFICATION resolved)
  - Data Model Coverage: PASS (7 entities, all spec requirements mapped)
  - API Contract Accuracy: PASS (OpenAPI spec matches spec.md FR-003, FR-004)
  - Quickstart Usability: PASS (step-by-step guide with troubleshooting)
- Prompt variant (if applicable): N/A (standard /sp.plan flow with user-provided high-level steps)
- Next experiment (smallest change to try): For future /sp.plan commands, consider adding "Deployment Architecture" section to plan.md showing how backend integrates with existing frontend (Docusaurus ChatBot component)
