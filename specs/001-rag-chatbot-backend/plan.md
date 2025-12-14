# Implementation Plan: RAG Chatbot Backend

**Branch**: `001-rag-chatbot-backend` | **Date**: 2025-12-10 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot-backend/spec.md`

## Summary

Build a production-ready RAG (Retrieval-Augmented Generation) chatbot backend using FastAPI that enables users to ask questions about the Physical AI Textbook content. The system converts user queries to embeddings via OpenAI API, retrieves semantically similar book chunks from Qdrant vector database, and generates accurate answers with GPT-4 while maintaining chat history in Neon Serverless Postgres. The backend supports advanced features including selected-text queries, chapter-scoped retrieval, streaming responses, and user feedback collection.

**Technical Approach**: Clean architecture with FastAPI framework, service layer separation, async/await patterns for I/O operations, and environment-based configuration. The RAG pipeline follows a three-stage process: (1) Embed user query with OpenAI text-embedding-3-small, (2) Retrieve top-5 semantically similar chunks from Qdrant with metadata filtering, (3) Generate contextual answer with GPT-4-turbo using retrieved chunks as context with proper citations.

## Technical Context

**Language/Version**: Python 3.11+ (required for async improvements and performance)
**Primary Dependencies**: FastAPI 0.104+, qdrant-client 1.7+, openai 1.3+, SQLAlchemy 2.0+, asyncpg 0.29+, pydantic 2.5+
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (chat history + analytics)
**Testing**: pytest 7.4+, pytest-asyncio 0.21+, httpx 0.25+ (async test client)
**Target Platform**: Linux server (Docker containerized), deployed on Vercel/Railway/similar PaaS
**Project Type**: Web backend API (standalone service, consumed by existing Docusaurus frontend)
**Performance Goals**: <3s p95 query-to-answer latency, 50 concurrent users, <500ms vector search
**Constraints**: <200ms embedding API call, Qdrant Free Tier (1GB vectors), Neon Free Tier limits, 100 req/min rate limit per user
**Scale/Scope**: ~3 book chapters (~150k tokens), ~300 embedded chunks, hundreds of daily queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Constitution v1.3.0

**✅ XIII. RAG Backend Architecture Compliance**:
- Clean architecture with service layer separation (FastAPI structure)
- Async/await patterns for all I/O operations (Qdrant, OpenAI, Neon)
- Environment-based configuration (.env file already configured)
- Health checks and observability endpoints included

**✅ XIV. Embedding Pipeline Excellence Compliance**:
- OpenAI text-embedding-3-small for embeddings (1536 dimensions)
- Chunking strategy: 1000 tokens with 200 token overlap (configured in .env)
- Batch processing for content indexing (admin endpoint design)
- Metadata preservation (chapter_id, section_id, heading, page)

**✅ XV. Vector Storage Strategy Compliance**:
- Qdrant Cloud Free Tier integration (credentials configured)
- Collection: book_embeddings with chapter namespace
- Metadata filtering for chapter-scoped queries
- Similarity threshold: 0.7 cosine similarity (configured)

**✅ XVI. Context Retrieval Precision Compliance**:
- Semantic search via Qdrant cosine similarity
- Top-5 chunk retrieval (configurable via .env)
- Chapter-scoped filtering when requested
- Selected text prioritization for focused queries

**✅ XVII. Response Generation Quality Compliance**:
- GPT-4-turbo-preview for answer generation
- Prompt engineering with system context and retrieved chunks
- Citations included (chapter/section references)
- Streaming support via Server-Sent Events (SSE)

**✅ XVIII. Database Management Compliance**:
- Neon Serverless Postgres for chat history (credentials configured)
- AsyncPG + SQLAlchemy 2.0 async ORM
- Connection pooling and SSL/TLS required
- Chat session persistence across login/logout

**✅ XIX. Selected Text Query Support Compliance**:
- Accept optional selected_text parameter
- Prioritize selected text in retrieval
- Expand context for short selections (<100 chars)
- Validate selected text exists in vector database

**✅ RAG API Standards Compliance**:
- Required endpoints: POST /api/query, POST /api/embed, GET /api/health, POST /api/feedback
- Pydantic request/response validation
- Appropriate HTTP status codes (200, 400, 422, 500, 503)
- Request ID tracing in all responses

**Gate Status**: ✅ **PASS** - All constitution principles aligned with design

### Potential Violations (Justified)

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Additional backend service | RAG requires specialized infrastructure (vector DB, embeddings) not suitable for frontend | Embedding RAG in Docusaurus frontend would bloat bundle, expose API keys, and prevent scaling |
| Multiple databases (Qdrant + Neon) | Vector search requires specialized database; relational data (chat history) requires Postgres | Single database solution would sacrifice either vector search performance or relational integrity |

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot-backend/
├── spec.md              # Feature specification (COMPLETED)
├── plan.md              # This file (IN PROGRESS)
├── research.md          # Phase 0 output (PENDING)
├── data-model.md        # Phase 1 output (PENDING)
├── quickstart.md        # Phase 1 output (PENDING)
├── contracts/           # Phase 1 output (PENDING)
│   └── openapi.yaml     # OpenAPI 3.0 specification
└── tasks.md             # Phase 2 output via /sp.tasks (NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/                          # New directory for RAG backend service
├── src/
│   ├── main.py                  # FastAPI application entry point
│   ├── config.py                # Environment configuration (loads .env)
│   ├── models/                  # Pydantic models + SQLAlchemy entities
│   │   ├── request_models.py   # API request schemas
│   │   ├── response_models.py  # API response schemas
│   │   └── db_models.py        # SQLAlchemy models (ChatSession, ChatMessage, Feedback)
│   ├── services/                # Business logic layer
│   │   ├── embedding_service.py    # OpenAI embedding generation
│   │   ├── vector_service.py       # Qdrant vector operations
│   │   ├── query_service.py        # RAG query orchestration
│   │   ├── chat_service.py         # Chat history management
│   │   └── feedback_service.py     # Feedback collection
│   ├── api/                     # API route handlers
│   │   ├── query.py            # POST /api/query endpoint
│   │   ├── embed.py            # POST /api/embed (admin)
│   │   ├── health.py           # GET /api/health
│   │   └── feedback.py         # POST /api/feedback
│   ├── middleware/              # FastAPI middleware
│   │   ├── rate_limiter.py     # Rate limiting (100 req/min)
│   │   ├── cors.py             # CORS configuration
│   │   └── error_handler.py    # Global error handling
│   └── utils/                   # Utility functions
│       ├── chunking.py         # Text chunking logic
│       ├── logger.py           # Logging configuration
│       └── validators.py       # Input validation helpers
├── tests/
│   ├── unit/                    # Unit tests (isolated, mocked)
│   │   ├── test_embedding_service.py
│   │   ├── test_vector_service.py
│   │   ├── test_query_service.py
│   │   └── test_chunking.py
│   ├── integration/             # Integration tests (real services)
│   │   ├── test_qdrant_integration.py
│   │   ├── test_neon_integration.py
│   │   └── test_openai_integration.py
│   └── contract/                # API contract tests (OpenAPI validation)
│       └── test_api_contracts.py
├── requirements.txt             # Python dependencies
├── Dockerfile                   # Container image for deployment
├── .dockerignore               # Docker build exclusions
└── README.md                   # Backend setup and deployment guide

.env                             # Environment variables (CONFIGURED - see spec.md)
.env.example                     # Template (CONFIGURED - see spec.md)
```

**Structure Decision**: Web application backend option selected. The backend is a standalone FastAPI service separate from the existing Docusaurus frontend. This separation enables:
1. Independent deployment and scaling of RAG infrastructure
2. Language choice (Python) optimized for ML/AI libraries (OpenAI, vector DBs)
3. Security isolation (API keys never exposed to frontend)
4. Technology flexibility (frontend remains Node.js/React, backend uses Python ecosystem)

The `backend/` directory is placed at repository root alongside existing `docs/`, `src/` (frontend), and `specs/` directories.

## Complexity Tracking

> **No unjustified complexity violations detected**

All complexity (additional service, multiple databases) is justified by functional requirements and aligned with RAG Backend Architecture principles in constitution v1.3.0.

---

## Phase 0: Research (COMPLETED)

**Output**: [research.md](./research.md)

All technical unknowns have been resolved through comprehensive research:

✅ **Text Chunking**: Recursive chunking with 400-600 tokens, 50-100 token overlap
✅ **Embedding Strategy**: OpenAI text-embedding-3-small with batch processing
✅ **Vector Search**: Qdrant single collection with metadata filtering
✅ **Context Retrieval**: Two-stage retrieval (over-fetch → re-rank) for 48% accuracy improvement
✅ **Prompt Engineering**: Structured system prompt with mandatory citations
✅ **Async Patterns**: Fully async FastAPI with SSE streaming
✅ **Error Handling**: Circuit breaker pattern with graceful degradation
✅ **Security**: Multi-layered defense against prompt injection

---

## Phase 1: Design & Contracts (COMPLETED)

**Outputs**: [data-model.md](./data-model.md), [contracts/openapi.yaml](./contracts/openapi.yaml), [quickstart.md](./quickstart.md)

### Data Model Summary

**Vector Database (Qdrant)**:
- Collection: `book_embeddings` (1536 dimensions, cosine similarity)
- Entities: `BookChunk` with chapter/section metadata and payload indexes

**Relational Database (Neon Postgres)**:
- `ChatSession`: Conversation sessions (authenticated + anonymous)
- `ChatMessage`: User queries and assistant responses
- `Feedback`: Thumbs up/down ratings with optional text
- `UsageLog`: API usage tracking and analytics

### API Contracts Summary

**Endpoints**:
1. `POST /api/query` - Question-answering with citations (<3s p95 latency)
2. `POST /api/query/stream` - Streaming responses via SSE
3. `POST /api/embed` - Admin content embedding (batch processing)
4. `POST /api/feedback` - User feedback collection
5. `GET /api/health` - Service health checks (OpenAI, Qdrant, Neon)

**Authentication**:
- User endpoints: JWT Bearer token (better-auth.com)
- Admin endpoints: API key header (X-Admin-API-Key)

### Agent Context Update

✅ Updated `CLAUDE.md` with RAG backend technology stack, project structure, and commands

---

## Re-Evaluation: Constitution Check (Post-Design)

*Gate passed. All design artifacts align with constitution v1.3.0.*

### Constitution Compliance Verification

**✅ XIII. RAG Backend Architecture**:
- Clean architecture implemented: FastAPI → Services → Models
- Async/await patterns in all service layers (embedding_service, vector_service, query_service)
- Environment-based config (.env validated at startup)
- Health check endpoint for all dependencies

**✅ XIV. Embedding Pipeline Excellence**:
- OpenAI text-embedding-3-small selected (1536 dimensions)
- Recursive chunking strategy: 400-600 tokens with 50-100 overlap (from research)
- Batch embedding in admin endpoint (100-500 chunks per request)
- Metadata preservation in Qdrant payload (chapter_id, section_id, heading, page)

**✅ XV. Vector Storage Strategy**:
- Qdrant Cloud Free Tier configured (credentials in .env)
- Single collection with chapter namespace via metadata filtering
- Payload indexes on chapter_id and section_id for filter performance
- Cosine similarity with 0.7 threshold (tunable via .env)

**✅ XVI. Context Retrieval Precision**:
- Two-stage retrieval: top-10-30 from Qdrant → re-rank → top-3-5 for GPT
- Chapter-scoped filtering via Qdrant metadata
- Selected text prioritization in query_service
- Similarity threshold enforcement

**✅ XVII. Response Generation Quality**:
- GPT-4-turbo-preview configured for answer generation
- Structured prompt engineering with retrieved chunks as context
- Mandatory citations in response format [1][2]
- SSE streaming support for progressive display

**✅ XVIII. Database Management**:
- Neon Serverless Postgres configured (credentials in .env)
- AsyncPG + SQLAlchemy 2.0 async ORM
- Connection pooling (5-20 connections)
- Chat session persistence with cascade deletion rules

**✅ XIX. Selected Text Query Support**:
- `selected_text` parameter in QueryRequest schema
- Validation in query_service (exists in vector DB)
- Context expansion for short selections (<100 chars)
- Prioritization logic in retrieval stage

**✅ RAG API Standards**:
- All 4 required endpoints implemented (query, embed, health, feedback)
- Pydantic 2.5+ validation for all request/response models
- HTTP status codes: 200 (success), 400 (validation), 422 (unprocessable), 429 (rate limit), 500 (error), 503 (unavailable)
- Request ID tracing in all responses

**Gate Status**: ✅ **PASS** - All design artifacts comply with constitution principles.

---

## Architectural Decision Records (ADR Suggestions)

The following architecturally significant decisions were made during planning and should be documented:

### 1. Two-Stage Retrieval Architecture

📋 **Architectural decision detected**: Two-stage retrieval (over-fetch from Qdrant → re-rank with cross-encoder → send to GPT) for 48% accuracy improvement vs single-stage retrieval.

**Impact**: Long-term (affects answer quality, latency, cost)
**Alternatives**: Single-stage retrieval, hybrid search, query expansion
**Scope**: Cross-cutting (affects all query operations)

**Recommendation**: Document reasoning? Run `/sp.adr "Two-Stage Retrieval for RAG Pipeline"`

### 2. Single Qdrant Collection with Metadata Filtering

📋 **Architectural decision detected**: Use single Qdrant collection with chapter/section metadata filtering instead of multiple collections per chapter.

**Impact**: Long-term (affects scaling, query patterns, data management)
**Alternatives**: Multiple collections (one per chapter), namespace-based separation
**Scope**: Cross-cutting (affects embedding strategy, querying, scaling)
**Sitemap URL**: https://physical-ai-robotics-book.vercel.app/sitemap.xml

**Recommendation**: Document reasoning? Run `/sp.adr "Single Collection Architecture for Vector Storage"`

### 3. Graceful Degradation with Circuit Breaker Pattern

📋 **Architectural decision detected**: Implement circuit breaker pattern for external services (OpenAI, Qdrant) with graceful degradation instead of hard failures.

**Impact**: Long-term (affects reliability, user experience, operational complexity)
**Alternatives**: Fail fast, simple retry logic, no error handling
**Scope**: Cross-cutting (affects all external service calls)

**Recommendation**: Document reasoning? Run `/sp.adr "Circuit Breaker Pattern for Service Resilience"`

---

## Next Steps

**Planning Complete** ✅

The architectural plan is finalized. Next phase:

1. **Generate Tasks**: Run `/sp.tasks` to break down implementation into testable, executable tasks
2. **Document ADRs** (Optional): Create ADRs for the 3 significant decisions identified above
3. **Implement**: Run `/sp.implement` to execute tasks from tasks.md

---

**STATUS**: ✅ Planning complete. All Phase 0 (Research) and Phase 1 (Design) artifacts generated.

**Artifacts Created**:
- ✅ `plan.md` (this file)
- ✅ `research.md` (best practices research)
- ✅ `data-model.md` (entities, relationships, validation rules)
- ✅ `contracts/openapi.yaml` (API specification)
- ✅ `quickstart.md` (developer setup guide)
- ✅ `CLAUDE.md` updated (agent context)

**Branch**: `001-rag-chatbot-backend`
**Ready for**: Task generation (`/sp.tasks`)
