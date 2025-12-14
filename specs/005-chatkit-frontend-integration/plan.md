# Implementation Plan: ChatKit Frontend Integration

**Branch**: `005-chatkit-frontend-integration` | **Date**: 2025-12-13 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/005-chatkit-frontend-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Integrate the FastAPI RAG backend with the Docusaurus book frontend by creating a chat interface using the ChatScope Chat UI Kit (already installed as `@chatscope/chat-ui-kit-react`). Users will be able to ask questions about book content within each chapter page, with questions automatically scoped to the current chapter context. The system will send user queries to the `/api/query` endpoint, display loading states during retrieval, and render AI-generated answers with source citations.

## Technical Context

**Language/Version**: TypeScript 5.3+ / React 18.2+
**Primary Dependencies**:
  - Frontend: `@chatscope/chat-ui-kit-react` v2.1.1 (already installed), Docusaurus 3.0, React 18.2
  - Backend: FastAPI (Python 3.11), running at `http://localhost:8000` (development)
  - API Client: Native `fetch` API with async/await
**Storage**:
  - Client-side: Session storage for conversation history (temporary, per-session)
  - Backend: Neon Postgres (chat history), Qdrant (vector embeddings) - already configured
**Testing**:
  - Manual testing via browser (development phase)
  - Future: Jest/React Testing Library for component tests
**Target Platform**: Modern web browsers (Chrome, Firefox, Safari, Edge - last 2 versions), responsive 320px-1920px
**Project Type**: Web application (Docusaurus frontend + FastAPI backend)
**Performance Goals**:
  - <2 second initial chat component load
  - <15 second end-to-end query response (frontend + backend)
  - UI remains responsive during API calls (loading indicators)
**Constraints**:
  - Must not break existing Docusaurus book layout
  - Chat interface <100KB bundle size impact
  - CORS must be configured on backend for frontend origin
  - Must work in local development environment first
**Scale/Scope**:
  - Single-user sessions (no multi-user chat)
  - 20+ message pairs per session without performance degradation
  - Chapter-scoped queries (filter by chapter_id in backend)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Component Reusability (Principle III)
- **Status**: PASS
- **Analysis**: This feature creates a NEW chat component for the frontend. It does not modify existing Chapter 1 components. The component will be designed for reusability across all chapters via props (chapter_id parameter).
- **Action**: Ensure ChatBot component accepts chapter_id as prop for multi-chapter support.

### ✅ RAG Backend Architecture (Principle XIII)
- **Status**: PASS
- **Analysis**: Backend `/api/query` endpoint already exists with FastAPI, proper request/response validation (Pydantic), and separation of concerns (embedding, vector, retrieval, response services).
- **Action**: No backend architecture changes needed. CORS configuration already in place.

### ✅ Multi-Chapter Architecture (Principle I)
- **Status**: PASS
- **Analysis**: Chat interface will support chapter-scoping via `chapter_id` parameter in API requests. No hardcoded chapter values. Chapter context determined dynamically from page URL/metadata.
- **Action**: Implement chapter detection logic in component (read from page context or URL).

### ✅ RAG API Standards (Constitution: RAG API Standards)
- **Status**: PASS
- **Analysis**: Backend endpoint `/api/query` follows required contract:
  - Request: `{query: str, chapter_id: str, session_id?: str, selected_text?: str}`
  - Response: `{answer: str, sources: [...], session_id: str, token_usage, response_time_ms}`
  - Timeout: <15s (spec requirement matches)
  - CORS: Configured in main.py
- **Action**: Frontend must match this contract exactly.

### ✅ Performance Requirements (Constitution)
- **Status**: PASS with monitoring
- **Analysis**:
  - Spec: <15s end-to-end (frontend + backend)
  - Constitution: <3s p95 query endpoint
  - Combined: Within acceptable range for MVP
- **Action**: Add response time tracking, optimize if needed.

### ✅ Security Requirements (Constitution)
- **Status**: PASS
- **Analysis**:
  - API keys stored in backend .env (not exposed to frontend)
  - CORS properly configured
  - HTTPS enforcement deferred to production deployment
  - No sensitive data in frontend code
- **Action**: Ensure no API keys or secrets in frontend bundle.

### ⚠️ Future-Proof Architecture (Principle XII)
- **Status**: PASS with note
- **Analysis**: Current implementation uses REST API. Constitution mentions streaming support (SSE) for progressive display. This is out of scope for MVP but architecture should not prevent future addition.
- **Action**: Design API client service layer to allow streaming in future without breaking changes.

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
# Frontend (Docusaurus + React + TypeScript)
src/
├── components/
│   ├── ChatBot.tsx               # NEW: Main chat interface component
│   ├── ChatBot.module.css        # NEW: Chat component styles
│   └── [existing components...]  # PersonalizeButton, TranslateButton, etc.
├── services/
│   └── chatService.ts            # NEW: API client for /api/query endpoint
├── types/
│   └── chat.ts                   # NEW: TypeScript interfaces for chat
└── css/
    └── custom.css                # Existing global styles

# Backend (FastAPI - already implemented, no changes needed)
backend/
├── src/
│   ├── api/
│   │   └── query.py              # EXISTING: /api/query endpoint
│   ├── models/
│   │   ├── request_models.py     # EXISTING: QueryRequest model
│   │   └── response_models.py    # EXISTING: QueryResponse model
│   └── main.py                   # EXISTING: CORS already configured
└── .env                          # EXISTING: Update CORS_ORIGINS if needed

# Configuration
.env.example                      # NEW: Document REACT_APP_API_URL
docusaurus.config.ts              # EXISTING: May need custom fields for API URL
```

**Structure Decision**: Web application with **frontend-only changes**. Backend API (`/api/query`) is already implemented and tested. This feature adds:
1. React chat component (`src/components/ChatBot.tsx`)
2. API service layer (`src/services/chatService.ts`)
3. TypeScript type definitions (`src/types/chat.ts`)

No backend code changes required - only potential CORS origin update in `.env` if frontend URL differs from `http://localhost:3000`.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

**No violations detected.** All Constitution Check gates passed. No complexity justifications required.
