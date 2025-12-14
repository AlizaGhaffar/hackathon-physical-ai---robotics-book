# API Contract: ChatKit Frontend ↔ FastAPI Backend

**Feature**: 005-chatkit-frontend-integration
**Date**: 2025-12-13
**Backend**: FastAPI RAG Chatbot (`backend/src/api/query.py`)
**Frontend**: Docusaurus + React TypeScript

---

## Base URL

**Development**: `http://localhost:8000`
**Production**: `TBD` (configured via Docusaurus customFields)

---

## Authentication

**None required for MVP**. All endpoints are public.

Future enhancement: Add API key authentication for production deployment.

---

## CORS Configuration

**Backend CORS Middleware**: `backend/src/main.py:42-48`

**Allowed Origins** (from `.env`):
- `http://localhost:3000` (Docusaurus dev server)
- `http://localhost:8000` (FastAPI docs/testing)

**Methods**: `*` (all methods allowed)
**Headers**: `*` (all headers allowed)
**Credentials**: Enabled

**Production**: Update `CORS_ORIGINS` environment variable to include production frontend URL.

---

## Endpoints Used by Frontend

### 1. POST /api/query

**Purpose**: Send user question, receive AI-generated answer with source citations.

**Request**:
```http
POST /api/query HTTP/1.1
Host: localhost:8000
Content-Type: application/json

{
  "query": "What is ROS 2?",
  "chapter_id": "chapter-1",
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Request Body** (JSON):
| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| `query` | string | ✅ Yes | 1-1000 characters | User's question about book content |
| `chapter_id` | string | ✅ Yes | Must start with "chapter-" or "Chapter " | Chapter to query (for scoped retrieval) |
| `session_id` | string | ❌ No | Valid UUID | Session identifier (generated if not provided) |
| `selected_text` | string | ❌ No | Max 5000 characters | User-selected text for focused queries (out of scope for MVP) |

**Validation Rules** (Backend Pydantic):
- `query`: Stripped of whitespace, must not be empty
- `chapter_id`: Validated by regex pattern
- `session_id`: Must be valid UUID format if provided

**Success Response** (200 OK):
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "request_id": "660e8400-e29b-41d4-a716-446655440000",
  "answer": "ROS 2 (Robot Operating System 2) is the second generation...",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "sources": [
    {
      "chunk_id": "chunk_abc123",
      "content": "ROS 2 is a robot middleware...",
      "score": 0.89,
      "source_file": "docs/chapter-1/intro.mdx",
      "chapter_id": "chapter-1"
    }
  ],
  "token_usage": {
    "prompt": 450,
    "completion": 120,
    "total": 570
  },
  "response_time_ms": 3245.5
}
```

**Response Body** (JSON):
| Field | Type | Required | Description |
|-------|------|----------|-------------|
| `request_id` | string | ✅ Yes | Unique request identifier (UUID) for tracing |
| `answer` | string | ✅ Yes | AI-generated answer (may contain markdown) |
| `session_id` | string | ✅ Yes | Session identifier (echoed or newly generated) |
| `sources` | array | ✅ Yes | Retrieved context chunks (may be empty array) |
| `sources[].chunk_id` | string | ✅ Yes | Unique chunk identifier |
| `sources[].content` | string | ✅ Yes | Text content of the chunk |
| `sources[].score` | number | ✅ Yes | Relevance score (0.0-1.0) |
| `sources[].source_file` | string | ✅ Yes | Source file path |
| `sources[].chapter_id` | string | ✅ Yes | Chapter identifier |
| `token_usage` | object | ❌ No | OpenAI API token counts |
| `token_usage.prompt` | number | ❌ No | Prompt tokens used |
| `token_usage.completion` | number | ❌ No | Completion tokens used |
| `token_usage.total` | number | ❌ No | Total tokens used |
| `response_time_ms` | number | ❌ No | Backend processing time in milliseconds |

**Error Responses**:

**422 Unprocessable Entity** (Validation Error):
```http
HTTP/1.1 422 Unprocessable Entity
Content-Type: application/json

{
  "detail": [
    {
      "loc": ["body", "query"],
      "msg": "Query cannot be empty or whitespace only",
      "type": "value_error"
    }
  ]
}
```

**500 Internal Server Error** (Backend Failure):
```http
HTTP/1.1 500 Internal Server Error
Content-Type: application/json

{
  "error": "query_processing_failed",
  "message": "Failed to process query. Please try again.",
  "request_id": "660e8400-e29b-41d4-a716-446655440000"
}
```

**Error Handling** (Frontend):
| HTTP Status | Error Type | User Message | Action |
|-------------|------------|--------------|--------|
| 422 | Validation | "Invalid question format. Please try rephrasing." | Show error message, allow retry |
| 500, 503 | Server | "The chatbot service is temporarily unavailable. Please try again later." | Show error message, allow retry |
| Network error | Network | "Unable to connect to the chatbot service. Please check your connection." | Show error message, allow retry |
| Timeout (>30s) | Timeout | "Request took too long. Please try again." | Show error message, allow retry |

**Performance**:
- **Target**: <3s p95 (backend processing)
- **Spec requirement**: <15s end-to-end (frontend + backend)
- **Timeout**: 30s (frontend aborts request after 30s)

---

### 2. GET /api/health

**Purpose**: Health check endpoint (optional for MVP, useful for debugging).

**Request**:
```http
GET /api/health HTTP/1.1
Host: localhost:8000
```

**No Request Body**

**Success Response** (200 OK):
```http
HTTP/1.1 200 OK
Content-Type: application/json

{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-13T10:30:00Z",
  "services": {
    "openai": "ok",
    "qdrant": "ok",
    "neon": "ok"
  }
}
```

**Response Body** (JSON):
| Field | Type | Description |
|-------|------|-------------|
| `status` | string | Overall health status ("healthy" or "unhealthy") |
| `version` | string | API version |
| `timestamp` | string | Current server timestamp (ISO 8601) |
| `services` | object | Status of dependent services |
| `services.openai` | string | OpenAI API status ("ok" or "error") |
| `services.qdrant` | string | Qdrant vector database status |
| `services.neon` | string | Neon Postgres database status |

**Usage**:
- Display in UI footer: "Backend: ✅ Connected" vs "Backend: ❌ Disconnected"
- Optional: Check on component mount, show warning if unhealthy
- Not critical for MVP (can be added later)

---

## Frontend Implementation

### TypeScript Service Layer

**File**: `src/services/chatService.ts`

```typescript
const API_BASE_URL = 'http://localhost:8000'; // From Docusaurus customFields

export interface QueryRequest {
  query: string;
  chapter_id: string;
  session_id?: string;
}

export interface QueryResponse {
  request_id: string;
  answer: string;
  session_id: string;
  sources: RetrievedChunk[];
  token_usage?: {
    prompt: number;
    completion: number;
    total: number;
  };
  response_time_ms?: number;
}

export async function sendQuery(
  query: string,
  chapterId: string,
  sessionId?: string
): Promise<QueryResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 30000);

  try {
    const response = await fetch(`${API_BASE_URL}/api/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query,
        chapter_id: chapterId,
        session_id: sessionId,
      }),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      if (response.status === 422) {
        throw new ChatServiceError('validation', 'Invalid question format', 422);
      }
      if (response.status >= 500) {
        throw new ChatServiceError('server', 'Backend service error', response.status);
      }
    }

    const data: QueryResponse = await response.json();
    return data;
  } catch (error) {
    clearTimeout(timeoutId);
    if (error.name === 'AbortError') {
      throw new ChatServiceError('timeout', 'Request timeout after 30 seconds');
    }
    if (error instanceof ChatServiceError) {
      throw error;
    }
    throw new ChatServiceError('network', 'Failed to connect to backend');
  }
}
```

---

## Request/Response Examples

### Example 1: Successful Query

**Request**:
```json
POST /api/query
{
  "query": "What are the main components of ROS 2?",
  "chapter_id": "chapter-1",
  "session_id": "a1b2c3d4-e5f6-4789-0123-456789abcdef"
}
```

**Response** (200 OK):
```json
{
  "request_id": "f1e2d3c4-b5a6-4879-0213-654321fedcba",
  "answer": "The main components of ROS 2 are:\n\n1. **Nodes**: Independent processes that perform computation\n2. **Topics**: Named buses for asynchronous message passing\n3. **Services**: Synchronous request-reply communication\n4. **Actions**: Asynchronous goal-based tasks with feedback\n\nThese components work together to create a distributed robotics system.",
  "session_id": "a1b2c3d4-e5f6-4789-0123-456789abcdef",
  "sources": [
    {
      "chunk_id": "chunk_1a2b3c",
      "content": "ROS 2 nodes are independent processes that communicate via topics...",
      "score": 0.92,
      "source_file": "docs/chapter-1/nodes.mdx",
      "chapter_id": "chapter-1"
    },
    {
      "chunk_id": "chunk_4d5e6f",
      "content": "Topics provide a publish-subscribe mechanism for asynchronous data flow...",
      "score": 0.88,
      "source_file": "docs/chapter-1/topics.mdx",
      "chapter_id": "chapter-1"
    }
  ],
  "token_usage": {
    "prompt": 520,
    "completion": 95,
    "total": 615
  },
  "response_time_ms": 2847.3
}
```

**Frontend Actions**:
1. Display answer in assistant message with markdown formatting
2. Show sources in expandable section below answer
3. Display footer: "Answer generated in 2.8s • 615 tokens used"

---

### Example 2: Empty Query (Validation Error)

**Request**:
```json
POST /api/query
{
  "query": "   ",
  "chapter_id": "chapter-1"
}
```

**Response** (422 Unprocessable Entity):
```json
{
  "detail": [
    {
      "loc": ["body", "query"],
      "msg": "Query cannot be empty or whitespace only",
      "type": "value_error"
    }
  ]
}
```

**Frontend Actions**:
1. Catch 422 error
2. Display error message: "Invalid question format. Please try rephrasing."
3. Keep input field active for retry

---

### Example 3: Network Error

**Request**: Sent, but backend is unreachable

**Error**: `fetch` throws network error

**Frontend Actions**:
1. Catch ChatServiceError type='network'
2. Display error message: "Unable to connect to the chatbot service. Please check your connection."
3. Log to console: `[ChatBot] network error: Failed to connect to backend`
4. Keep input field active for retry

---

### Example 4: Timeout (>30s)

**Request**: Sent, but backend takes too long to respond

**Error**: AbortController triggers abort after 30s

**Frontend Actions**:
1. Catch ChatServiceError type='timeout'
2. Display error message: "Request took too long. Please try again."
3. Log to console: `[ChatBot] timeout error: Request timeout after 30 seconds`
4. Keep input field active for retry

---

## Testing Strategy

### Manual Testing Checklist

1. **✅ Successful query**: Send valid question, verify answer displays
2. **✅ Empty query**: Send empty string, verify validation error shows
3. **✅ Long query**: Send 1000+ character question, verify backend rejects
4. **✅ Network error**: Stop backend server, verify error message shows
5. **✅ Timeout**: Mock slow backend (add sleep), verify timeout after 30s
6. **✅ Server error**: Break backend (invalid API key), verify 500 error handling
7. **✅ CORS**: Open frontend from different port, verify CORS error (or success if configured)
8. **✅ Chapter switching**: Navigate to different chapter, verify chapter_id updates
9. **✅ Session persistence**: Refresh page, verify session_id persists (sessionStorage)
10. **✅ Multiple queries**: Send 5-10 queries, verify conversation history builds correctly

### Contract Validation

**Backend Contract** (Source of truth):
- `backend/src/models/request_models.py` (QueryRequest Pydantic model)
- `backend/src/models/response_models.py` (QueryResponse Pydantic model)

**Frontend Contract** (Must match):
- `src/types/chat.ts` (TypeScript interfaces)

**Validation Method**:
1. Compare Pydantic field types with TypeScript types
2. Ensure all required fields marked as required
3. Ensure optional fields marked as optional (`?`)
4. Test with real backend to catch serialization mismatches

---

## Future Enhancements (Out of Scope)

### Streaming Responses (Server-Sent Events)
- Backend: Implement SSE endpoint `/api/query/stream`
- Frontend: Use EventSource API to receive progressive chunks
- Benefits: Display answer word-by-word (better UX for long responses)

### Selected Text Queries
- Frontend: Add text selection listener
- Send selected text in `selected_text` field
- Backend: Use selected text for focused retrieval

### Chat History Retrieval
- Frontend: Call GET `/api/history/{session_id}` on component mount
- Display previous conversation from backend database
- Enable cross-session conversation continuity

### Feedback System
- Frontend: Add thumbs up/down buttons to assistant messages
- Call POST `/api/feedback` with rating
- Backend: Store feedback for response quality analysis

---

## Summary

**Primary Endpoint**: `POST /api/query`
**Request**: `{query, chapter_id, session_id?}`
**Response**: `{answer, sources, session_id, ...}`
**Error Handling**: 422 (validation), 500 (server), network, timeout
**CORS**: Configured for localhost:3000
**Frontend Service**: `chatService.ts` with TypeScript types
**Performance**: <30s timeout, <15s target end-to-end

Contract matches existing backend implementation. No backend changes required.
