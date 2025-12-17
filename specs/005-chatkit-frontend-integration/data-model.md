# Data Model: ChatKit Frontend Integration

**Feature**: 005-chatkit-frontend-integration
**Date**: 2025-12-13
**Phase**: 1 (Design & Contracts)

## Overview

This document defines the frontend data structures for the chat interface. The backend data models already exist in `backend/src/models/` and are not modified by this feature.

---

## Frontend Entities

### 1. Message (Client-Side)

Represents a single chat message in the UI, either from the user or the AI assistant.

**TypeScript Interface**:
```typescript
interface Message {
  id: string;                    // Unique message ID (UUID)
  role: 'user' | 'assistant';    // Who sent the message
  content: string;               // Message text content
  timestamp: Date;               // When message was created
  sources?: RetrievedChunk[];    // Source citations (assistant messages only)
  isError?: boolean;             // Flag for error messages
}
```

**Fields**:
- `id` (required): Generated UUID for React key prop and list management
- `role` (required): Determines message styling and alignment (user: right-aligned, assistant: left-aligned)
- `content` (required): Displayed message text. For user messages: raw input. For assistant: markdown-formatted answer from backend
- `timestamp` (required): Used for message sorting and optional display of time
- `sources` (optional): Array of retrieved chunks from backend QueryResponse. Only present for assistant messages when backend provides sources
- `isError` (optional): Indicates error message (network failure, timeout, etc.). Used for special error styling

**Validation Rules**:
- `role` must be exactly `'user'` or `'assistant'` (no other values)
- `content` must not be empty string (minimum 1 character)
- `timestamp` must be valid Date object
- `sources` array must contain valid RetrievedChunk objects if present

**State Transitions**:
```
User sends message → Message created with role='user', no sources
Backend responds → Message created with role='assistant', includes sources
Network error → Message created with role='assistant', isError=true, content=error message
```

---

### 2. ChatSession

Represents the current chat session state, including all messages and session metadata.

**TypeScript Interface**:
```typescript
interface ChatSession {
  sessionId: string;             // Persistent session ID (from sessionStorage)
  chapterId: string;             // Current chapter context (from URL)
  messages: Message[];           // Conversation history
  isLoading: boolean;            // True when waiting for backend response
}
```

**Fields**:
- `sessionId` (required): UUID persisted in sessionStorage. Generated once per browser tab session. Sent to backend to enable conversation history tracking
- `chapterId` (required): Extracted from URL (e.g., "chapter-1"). Sent to backend to scope queries to current chapter
- `messages` (required): Array of all messages in chronological order. Used to render MessageList. Stored in component state (React useState)
- `isLoading` (required): Boolean flag. True while API request is pending. Used to show typing indicator and disable input

**Validation Rules**:
- `sessionId` must be valid UUID format
- `chapterId` must match pattern: `chapter-\d+` (e.g., "chapter-1", "chapter-2")
- `messages` array ordered by timestamp (oldest first)
- `isLoading` must be false when no API call is in progress

**Persistence**:
- `sessionId`: Persisted in sessionStorage (survives page navigation)
- `chapterId`: Derived from URL (changes when user navigates chapters)
- `messages`: Component state only (cleared on page refresh)
- `isLoading`: Component state only (always false on mount)

---

### 3. RetrievedChunk (Backend Contract)

Source citation from backend QueryResponse. Matches backend Pydantic model `RetrievedChunk` exactly.

**TypeScript Interface**:
```typescript
interface RetrievedChunk {
  chunk_id: string;              // Unique chunk identifier
  content: string;               // Text content of the chunk
  score: number;                 // Relevance score (0.0-1.0)
  source_file: string;           // Source file path
  chapter_id: string;            // Chapter this chunk belongs to
}
```

**Fields**:
- `chunk_id` (required): Unique identifier for the chunk in Qdrant vector database
- `content` (required): Text snippet from the book that was used to generate the answer. May contain markdown
- `score` (required): Cosine similarity score (0.0 = no match, 1.0 = perfect match). Used to show relevance to user
- `source_file` (required): File path in book repository (e.g., "docs/chapter-1/intro.mdx")
- `chapter_id` (required): Chapter identifier (e.g., "chapter-1"). Should match session chapterId for chapter-scoped queries

**Validation Rules**:
- `score` must be between 0.0 and 1.0
- `content` must not be empty
- `chapter_id` format validated by backend, trusted by frontend

**Usage**:
- Display in expandable "Sources" section below AI answers
- Show source_file and score to indicate answer provenance
- Link to source file if possible (future enhancement)

---

### 4. QueryRequest (Backend Contract)

Request payload sent to `/api/query` endpoint. Matches backend Pydantic model `QueryRequest` exactly.

**TypeScript Interface**:
```typescript
interface QueryRequest {
  query: string;                 // User's question
  chapter_id: string;            // Chapter to query (required)
  session_id?: string;           // Session identifier (optional)
  selected_text?: string;        // User-selected text (optional, future)
}
```

**Fields**:
- `query` (required): User's question, 1-1000 characters. Validated by backend (Pydantic min_length=1, max_length=1000)
- `chapter_id` (required): Current chapter ID from URL. Backend validates format (must start with "chapter-" or "Chapter ")
- `session_id` (optional): UUID from sessionStorage. If not provided, backend generates new session ID
- `selected_text` (optional): Out of scope for MVP. Placeholder for future feature (Spec FR-019)

**Validation Rules** (Backend enforces, frontend should pre-validate):
- `query`: Strip whitespace, check not empty
- `chapter_id`: Must match URL-extracted pattern
- `session_id`: Must be valid UUID if provided

**Example**:
```json
{
  "query": "What is ROS 2?",
  "chapter_id": "chapter-1",
  "session_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

---

### 5. QueryResponse (Backend Contract)

Response from `/api/query` endpoint. Matches backend Pydantic model `QueryResponse` exactly.

**TypeScript Interface**:
```typescript
interface QueryResponse {
  request_id: string;            // Unique request identifier (for tracing)
  answer: string;                // Generated answer from RAG
  session_id: string;            // Session identifier (returned or generated)
  sources: RetrievedChunk[];     // Retrieved context chunks
  token_usage?: TokenUsage;      // OpenAI API token counts (optional)
  response_time_ms?: number;     // Backend processing time (optional)
}

interface TokenUsage {
  prompt: number;                // Prompt tokens used
  completion: number;            // Completion tokens used
  total: number;                 // Total tokens used
}
```

**Fields**:
- `request_id` (required): UUID generated by backend for request tracing. Logged to console for debugging
- `answer` (required): AI-generated answer text. May contain markdown formatting. Displayed in assistant message
- `session_id` (required): Session ID (either echoed from request or newly generated). Used to track conversation continuity
- `sources` (required): Array of retrieved chunks (may be empty array if no sources found)
- `token_usage` (optional): OpenAI API usage stats. Can be displayed in footer for transparency
- `response_time_ms` (optional): Backend processing time. Can be displayed in footer ("Answer generated in 3.2s")

**Example**:
```json
{
  "request_id": "660e8400-e29b-41d4-a716-446655440000",
  "answer": "ROS 2 is the second generation of the Robot Operating System...",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "sources": [
    {
      "chunk_id": "chunk_123",
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
  "response_time_ms": 3245
}
```

---

### 6. ChatServiceError (Custom Frontend Error)

Custom error class for API communication errors. Extends JavaScript Error.

**TypeScript Class**:
```typescript
class ChatServiceError extends Error {
  type: 'network' | 'timeout' | 'server' | 'validation';
  statusCode?: number;

  constructor(
    type: 'network' | 'timeout' | 'server' | 'validation',
    message: string,
    statusCode?: number
  ) {
    super(message);
    this.name = 'ChatServiceError';
    this.type = type;
    this.statusCode = statusCode;
  }
}
```

**Fields**:
- `type` (required): Error category for different handling strategies
  - `network`: Failed to connect to backend (CORS, network down, wrong URL)
  - `timeout`: Request exceeded 30-second timeout
  - `server`: Backend returned 5xx error (internal server error)
  - `validation`: Backend returned 422 (invalid request format)
- `message` (required): User-friendly error message to display
- `statusCode` (optional): HTTP status code if available (for logging/debugging)

**Error Type Mapping**:
```typescript
// Network errors (fetch fails)
throw new ChatServiceError('network', 'Unable to connect to the chatbot service. Please check your connection.');

// Timeout errors (AbortController triggered)
throw new ChatServiceError('timeout', 'Request took too long. Please try again.');

// Server errors (500-599)
throw new ChatServiceError('server', 'The chatbot service is temporarily unavailable. Please try again later.', 503);

// Validation errors (422)
throw new ChatServiceError('validation', 'Invalid question format. Please try rephrasing.', 422);
```

**Usage in Component**:
```typescript
try {
  const response = await sendQuery(query, chapterId, sessionId);
  // Handle success...
} catch (error) {
  if (error instanceof ChatServiceError) {
    // Display user-friendly error message
    addMessage({
      id: crypto.randomUUID(),
      role: 'assistant',
      content: error.message,
      timestamp: new Date(),
      isError: true
    });

    // Log technical details for debugging
    console.error(`[ChatBot] ${error.type} error:`, error.statusCode, error.message);
  }
}
```

---

## Entity Relationships

```
ChatSession (1)
  ├── sessionId (persisted)
  ├── chapterId (from URL)
  └── messages (0..n) ──> Message (n)
                            ├── content
                            ├── timestamp
                            └── sources (0..n) ──> RetrievedChunk (n)
                                                      ├── content
                                                      ├── score
                                                      └── source_file

QueryRequest (sent)
  ├── query (from user input)
  ├── chapter_id (from ChatSession.chapterId)
  └── session_id (from ChatSession.sessionId)

QueryResponse (received)
  ├── answer ──> Message.content (assistant)
  ├── sources ──> Message.sources
  ├── session_id ──> ChatSession.sessionId (update if new)
  └── response_time_ms (display in UI)
```

---

## State Management Strategy

### Component State (React useState)
```typescript
const [messages, setMessages] = useState<Message[]>([]);
const [isLoading, setIsLoading] = useState<boolean>(false);
```

**Why useState**:
- Chat state is component-local (no sharing between components)
- Simple state shape (messages array, loading boolean)
- No complex state transitions requiring useReducer
- No global state needed (each chapter page has independent chat)

### Browser Storage (sessionStorage)
```typescript
// Session ID persists across page navigations
const sessionId = sessionStorage.getItem('chat_session_id') || crypto.randomUUID();
sessionStorage.setItem('chat_session_id', sessionId);
```

**Why sessionStorage**:
- Survives page refreshes within same tab
- Cleared when tab closes (privacy)
- Simple key-value store (no complex serialization)
- Backend-compatible (session_id is just a string)

### URL as Source of Truth (Docusaurus routing)
```typescript
const location = useLocation();
const chapterId = extractChapterId(location.pathname);
```

**Why URL**:
- Chapter context is determined by current page
- No need to synchronize state with routing
- Works with browser back/forward buttons
- Single source of truth (no stale state)

---

## Data Flow

### User Sends Message

```
1. User types message, clicks send
   ↓
2. Component validates input (not empty)
   ↓
3. Create user Message object:
   - role: 'user'
   - content: user input
   - timestamp: new Date()
   ↓
4. Add to messages array (update state)
   ↓
5. Set isLoading = true
   ↓
6. Call chatService.sendQuery():
   - Build QueryRequest { query, chapter_id, session_id }
   - POST to /api/query
   - Parse QueryResponse
   ↓
7. Create assistant Message object:
   - role: 'assistant'
   - content: response.answer
   - sources: response.sources
   - timestamp: new Date()
   ↓
8. Add to messages array (update state)
   ↓
9. Set isLoading = false
   ↓
10. Auto-scroll to latest message
```

### Error Handling Flow

```
1. API call fails (network, timeout, server error)
   ↓
2. catch block receives ChatServiceError
   ↓
3. Create error Message object:
   - role: 'assistant'
   - content: error.message (user-friendly)
   - isError: true
   - timestamp: new Date()
   ↓
4. Add to messages array (update state)
   ↓
5. Set isLoading = false
   ↓
6. Log error details to console
```

---

## Validation Rules Summary

### Client-Side Validation (Before API Call)
- User input not empty after trim
- Chapter ID extracted from URL is valid
- Session ID is valid UUID (if exists)

### Backend Validation (Pydantic Models)
- Query length: 1-1000 characters
- Chapter ID format: starts with "chapter-" or "Chapter "
- Session ID format: valid UUID (if provided)

### TypeScript Type Checking (Compile-Time)
- All interfaces strictly typed
- No `any` types in data models
- Optional fields explicitly marked with `?`

---

## Performance Considerations

### Message Array Growth
- **Spec requirement SC-004**: Support 20 message pairs without degradation
- 20 pairs = 40 messages maximum (20 user + 20 assistant)
- Each message ~1KB (content + metadata) = 40KB total
- ChatScope MessageList uses virtualization for performance
- No pagination needed for MVP (<100 messages)

### State Updates
- Use functional setState to avoid stale closures:
  ```typescript
  setMessages(prev => [...prev, newMessage]);
  ```
- Avoid unnecessary re-renders:
  - Extract stable values (sessionId, chapterId) outside render
  - Memoize heavy computations with useMemo (if needed)

### Storage Limits
- sessionStorage limit: ~5MB per origin
- Session ID: 36 bytes (UUID)
- Well within limits

---

## Future Enhancements (Out of Scope for MVP)

### Persistent History
- Store messages in localStorage or backend database
- Retrieve history on page load
- Requires backend `/api/history/{session_id}` endpoint (already exists)

### Message Editing
- Add `edited` field to Message interface
- Track edit history
- Show "edited" indicator in UI

### Message Reactions
- Add `reactions` field to Message interface
- Store thumbs up/down feedback
- Send to backend `/api/feedback` endpoint (already exists)

### Multi-Modal Messages
- Add `attachments` field for images/files
- Support vision-based queries (screenshot understanding)
- Requires backend support for image embeddings

---

## Summary

Frontend data model consists of:
1. **Message**: Chat message (user or assistant)
2. **ChatSession**: Session state and message history
3. **RetrievedChunk**: Source citation from backend
4. **QueryRequest**: API request payload
5. **QueryResponse**: API response payload
6. **ChatServiceError**: Custom error handling

All entities align with backend Pydantic models. State management uses React useState for component state, sessionStorage for session persistence, and URL as source of truth for chapter context.

**Next Steps**: Define API contracts in `contracts/` directory.
