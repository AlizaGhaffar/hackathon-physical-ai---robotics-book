---
id: 002
title: RAG Backend Architectural Plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot-backend
branch: 001-rag-chatbot-backend
user: affil
command: /sp.plan
labels: ["rag", "fastapi", "qdrant", "neon", "openai", "architecture", "planning"]
links:
  spec: specs/001-rag-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot-backend/plan.md
  - specs/001-rag-chatbot-backend/research.md
  - specs/001-rag-chatbot-backend/data-model.md
  - specs/001-rag-chatbot-backend/contracts/openapi.yaml
  - specs/001-rag-chatbot-backend/quickstart.md
  - CLAUDE.md
tests:
  - None (planning phase)
---

## Prompt

/sp.plan

(User requested to continue the planning workflow after credential setup and spec review)

## Response snapshot

Successfully completed `/sp.plan` workflow for RAG Chatbot Backend feature.

**Phase 0: Research (COMPLETED)**
- Researched RAG implementation best practices for FastAPI + OpenAI + Qdrant + Neon stack
- Resolved 8 key technical decisions:
  1. Text chunking: Recursive chunking with 400-600 tokens, 50-100 token overlap
  2. Embedding: OpenAI text-embedding-3-small with batch processing (100-500 inputs)
  3. Vector search: Single Qdrant collection with metadata filtering, cosine similarity 0.7-0.8
  4. Context retrieval: Two-stage retrieval (over-fetch → re-rank) for 48% accuracy improvement
  5. Prompt engineering: Structured system prompt with mandatory citations
  6. Async patterns: Fully async FastAPI with SSE streaming
  7. Error handling: Circuit breaker pattern with graceful degradation
  8. Security: Multi-layered defense (input validation + output filtering + rate limiting)
- Generated comprehensive research.md with 40+ authoritative sources from 2025

**Phase 1: Design & Contracts (COMPLETED)**
- Created data-model.md with:
  - Qdrant BookChunk entity (1536 dimensions, payload indexes on chapter_id/section_id)
  - Neon Postgres entities: ChatSession, ChatMessage, Feedback, UsageLog
  - Validation rules, relationships, state transitions
- Generated OpenAPI 3.0 specification (contracts/openapi.yaml):
  - 5 endpoints: /api/query (+ stream), /api/embed, /api/feedback, /api/health
  - Complete request/response schemas with examples
  - Authentication: JWT for users, API key for admin
- Created quickstart.md developer guide:
  - 5-minute setup instructions
  - Common tasks (embed content, ask questions, submit feedback)
  - Testing, troubleshooting, deployment guides
- Updated CLAUDE.md agent context with RAG backend technology stack

**Constitution Check (RE-EVALUATED - PASSED)**
- All 9 RAG-specific constitution principles (XIII-XIX + tech stack + API standards) validated
- All design artifacts align with constitution v1.3.0
- No unjustified complexity violations

**ADR Suggestions (3 significant decisions identified)**:
1. Two-stage retrieval architecture (over-fetch → re-rank)
2. Single Qdrant collection with metadata filtering
3. Circuit breaker pattern for service resilience

**Project Structure**:
```
backend/
├── src/
│   ├── main.py, config.py
│   ├── models/ (Pydantic + SQLAlchemy)
│   ├── services/ (embedding, vector, query, chat, feedback)
│   ├── api/ (route handlers)
│   ├── middleware/ (rate limiting, CORS, errors)
│   └── utils/ (chunking, logging, validators)
├── tests/ (unit, integration, contract)
└── requirements.txt, Dockerfile, README.md
```

## Outcome

- ✅ Impact: Complete architectural plan for production-ready RAG chatbot backend. All technical unknowns resolved through research. Design artifacts (data model, API contracts, quickstart guide) ready for task generation and implementation. Identified 3 significant architectural decisions for ADR documentation.
- 🧪 Tests: None (planning phase - no code written yet)
- 📁 Files: Created plan.md, research.md, data-model.md, contracts/openapi.yaml, quickstart.md. Updated CLAUDE.md with RAG backend context.
- 🔁 Next prompts: Run /sp.tasks to generate implementation task breakdown, optionally /sp.adr for 3 identified decisions
- 🧠 Reflection: Planning workflow executed successfully despite PowerShell script unavailability (manually extracted feature context). Research agent provided comprehensive best practices from 2025 sources. Two-stage retrieval emerged as critical quality improvement (48% accuracy gain). Design artifacts complete and constitution-compliant. Ready for task generation.

## Evaluation notes (flywheel)

- Failure modes observed: PowerShell script unavailable (pwsh command not found in Linux/WSL environment). Mitigated by manually extracting feature context from branch name and existing spec.
- Graders run and results (PASS/FAIL): PASS - All constitution gates passed, design artifacts complete, no unresolved technical unknowns
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to task generation with /sp.tasks. Test whether research findings (especially two-stage retrieval recommendation) translate to actionable, testable tasks.
