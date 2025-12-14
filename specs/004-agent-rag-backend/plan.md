# Implementation Plan: Agent-Based RAG Backend

**Branch**: `004-agent-rag-backend` | **Date**: 2025-12-13 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/004-agent-rag-backend/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Build a FastAPI backend that integrates OpenAI Agents SDK with Qdrant vector database to create a retrieval-augmented generation (RAG) system. The agent will answer user questions strictly based on retrieved book content, preventing hallucination through controlled grounding. The system will support single-turn and multi-turn conversations, chapter-scoped queries, and provide source citations for all responses.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- FastAPI 0.104+ (web framework, per hackathon requirements)
- OpenAI Python SDK 1.10+ (Agents SDK, GPT-4, embeddings)
- qdrant-client 1.7+ (vector database client)
- cohere 4.37+ (query embedding generation, same as feature 002)
- Pydantic 2.5+ (request/response validation)
- python-dotenv 1.0+ (environment configuration)

**Storage**:
- Qdrant Cloud Free Tier (vector database for book chunk embeddings)
- In-memory session management (conversation history per session)
- No persistent storage for chat history in MVP (future: Neon Postgres)

**Testing**: pytest 7.4+, pytest-asyncio 0.21+, httpx 0.25+ (API testing)

**Target Platform**: Linux server (development), Vercel/Railway deployment (production)

**Project Type**: Single backend API service (RESTful FastAPI)

**Performance Goals**:
- Query response time: <3 seconds (p95) for single-turn questions
- Concurrent users: Support 50 concurrent requests
- Qdrant vector search: <500ms per query
- OpenAI API calls: <2 seconds average

**Constraints**:
- Must use Cohere embed-english-v3.0 (1024 dimensions) for query embeddings to match feature 002
- Qdrant Cloud Free Tier limits: 1GB storage, shared resources
- OpenAI API rate limits: Tier-based (need to handle 429 errors)
- Stateless backend (no session storage in-process)
- Must prevent hallucination (agent cannot use external knowledge)

**Scale/Scope**:
- Target: 50 concurrent users during demo/testing
- Vector database: ~5000 book chunks (estimated from 8-chapter book)
- API endpoints: 2 primary (/ask, /health)
- Response size: <2KB average (answer + citations)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### RAG Backend Architecture (Section XIII) ✅ PASS
- ✅ Using FastAPI framework (per hackathon requirements)
- ✅ Clean separation: Embedding Service, Vector Storage Service, Retrieval Service, Response Generation Service
- ✅ Stateless backend design (no session state in backend process)
- ✅ Pydantic models for request/response validation
- ✅ Configuration externalized via environment variables
- ✅ Centralized error handling planned

### Embedding Pipeline Excellence (Section XIV) ✅ PASS
- ✅ Using Cohere embed-english-v3.0 (same model as feature 002 for consistency)
- ✅ Query embeddings will match document embedding dimensions (1024)
- ✅ Metadata preservation (chapter, section) for filtered queries
- ✅ Idempotent design (re-querying produces consistent results)

### Vector Storage Strategy (Section XV) ✅ PASS
- ✅ Qdrant Cloud Free Tier (per hackathon requirements)
- ✅ Collections namespaced by chapter (chapter_1_vectors, chapter_2_vectors, etc.)
- ✅ Metadata payload includes: chapter_id, section_id, text, heading
- ✅ Support for metadata filtering (chapter-scoped queries)

### Context Retrieval Precision (Section XVI) ✅ PASS
- ✅ Top-K retrieval (default K=5 per spec)
- ✅ Configurable similarity threshold (default 0.7 per spec assumptions)
- ✅ Query embedding uses same model as documents (Cohere embed-english-v3.0)
- ✅ Chapter-scoped queries via metadata filtering

### Response Generation Quality (Section XVII) ✅ PASS
- ✅ Using OpenAI Agents SDK (per hackathon requirements)
- ✅ System prompt enforces grounding (FR-001: "answer using only retrieved content")
- ✅ Low temperature for factual consistency (planned)
- ✅ Citations included in responses (FR-008)
- ✅ Graceful error handling (FR-014)

### Technology Stack Compliance ⚠️ PARTIAL
- ✅ FastAPI (required per hackathon spec)
- ✅ OpenAI Agents SDK (required per hackathon spec)
- ✅ Qdrant Cloud Free Tier (required per hackathon spec)
- ⚠️ **DEVIATION**: Not using Neon Serverless Postgres in MVP (in-memory session management)
  - **Justification**: Spec assumption #3 states "Conversation history is maintained in-memory for a single session (no persistent cross-session history)"
  - **Future**: Can add Neon Postgres later for persistent chat history and analytics
  - **Impact**: No violation of core requirements; aligns with spec assumptions

### RAG API Standards (Constitution Section) ✅ PASS
- ✅ POST /ask endpoint planned (maps to /api/query standard)
- ✅ GET /health endpoint planned
- ✅ Request/response validation via Pydantic
- ✅ Proper HTTP status codes (200, 400, 422, 500, 503)
- ✅ Error response format: {error: str, detail: str}

### Performance Requirements ✅ PASS
- ✅ Query response time target: <3s (p95) - matches constitution requirement
- ✅ Qdrant vector search target: <500ms - within constitution requirement
- ✅ Concurrent users: 50 - matches constitution requirement

### Security Requirements ✅ PASS
- ✅ API keys in environment variables (never hardcoded)
- ✅ Input validation via Pydantic (prevents injection)
- ✅ No secrets in logs or error messages

### GATE STATUS: ✅ APPROVED WITH JUSTIFICATION
- **Deviations**: 1 (Neon Postgres deferred to post-MVP)
- **Justification**: Aligns with spec assumption #3 (in-memory session management)
- **Re-check Required**: After Phase 1 design (confirm architecture decisions)

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/agent-rag/
├── main.py                   # FastAPI application entrypoint
├── config.py                 # Environment configuration (API keys, endpoints, thresholds)
├── models/
│   ├── requests.py          # Pydantic request models (AskRequest, HealthRequest)
│   └── responses.py         # Pydantic response models (AskResponse, HealthResponse, Citation)
├── services/
│   ├── embedding.py         # Cohere embedding generation service
│   ├── vector.py            # Qdrant vector storage and retrieval service
│   ├── agent.py             # OpenAI Agents SDK integration
│   └── session.py           # In-memory session management (conversation history)
├── api/
│   ├── ask.py               # POST /ask endpoint handler
│   └── health.py            # GET /health endpoint handler
├── middleware/
│   ├── error_handler.py     # Centralized error handling
│   └── logging.py           # Request/response logging
└── utils/
    ├── validators.py        # Input validation helpers
    └── formatters.py        # Citation formatting, context assembly

tests/
├── unit/
│   ├── test_embedding.py    # Test Cohere embedding service
│   ├── test_vector.py       # Test Qdrant retrieval logic (mocked)
│   ├── test_agent.py        # Test OpenAI agent initialization (mocked)
│   └── test_validators.py  # Test input validation
├── integration/
│   ├── test_ask_endpoint.py     # Test /ask endpoint end-to-end (mocked externals)
│   └── test_health_endpoint.py  # Test /health endpoint
└── contract/
    └── test_openapi.py      # Validate OpenAPI schema matches spec

.env.example                  # Template environment variables
requirements.txt              # Python dependencies
README.md                     # Setup and deployment instructions
```

**Structure Decision**: Single backend API service structure. This feature adds a new `backend/agent-rag/` directory to the repository root, separate from existing `backend/embedding-pipeline/` (feature 002). The separation is justified because:
1. Different runtime models: embedding-pipeline is a batch script, agent-rag is a persistent API server
2. Different deployment targets: embedding-pipeline runs locally for data ingestion, agent-rag deploys to Vercel/Railway
3. Minimal shared code: both use Qdrant and Cohere but with different patterns (bulk upsert vs. single query)

Future optimization: Extract shared Qdrant/Cohere utilities to `backend/shared/` if code duplication exceeds 20%.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Neon Postgres deferred to post-MVP | Spec assumption #3 explicitly states in-memory session management for MVP | Adding Postgres now would violate spec assumptions and add unnecessary complexity for initial launch. In-memory sessions are sufficient for testing 0% hallucination and citation requirements (SC-001, SC-005). Postgres can be added later for persistent chat history without changing API contracts. |
