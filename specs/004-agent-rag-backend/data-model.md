# Phase 1: Data Model - Agent-Based RAG Backend

**Feature**: 004-agent-rag-backend
**Date**: 2025-12-13
**Status**: Complete

## Entity Definitions

### 1. Query

**Purpose**: Represents a user's question submitted to the RAG system

**Fields**:
- `question` (string, required): User's question text (1-1000 characters)
- `chapter_filter` (integer, optional): Chapter ID to scope retrieval (1-8)
- `section_filter` (string, optional): Section name to scope retrieval
- `session_id` (string, optional): Session identifier for multi-turn conversations
- `selected_text` (string, optional): User-highlighted text for context-specific queries

**Validation Rules**:
- `question` must not be empty or whitespace-only
- `question` length: 1-1000 characters
- `chapter_filter` must be in range 1-8 if provided
- `section_filter` must be alphanumeric with spaces/hyphens only
- `session_id` must be UUID v4 format if provided
- At most one of `chapter_filter` OR `section_filter` can be specified (not both)

**Relationships**:
- Produces one `AgentResponse`
- Belongs to zero or one `ConversationHistory` (if session_id provided)

**State Transitions**: Stateless (no persistence in MVP)

---

### 2. Chunk

**Purpose**: Represents a segment of book text stored in Qdrant vector database

**Fields**:
- `id` (string, required): Unique identifier (URL-based from feature 002)
- `text` (string, required): Book content (up to 500 tokens)
- `embedding` (float[], required): 1024-dimensional vector (Cohere embed-english-v3.0)
- `chapter_id` (integer, required): Chapter number (1-8)
- `section` (string, required): Section title
- `page` (integer, optional): Page number (may be null for digital-only books)
- `heading` (string, optional): Subsection heading
- `source_url` (string, required): Original Docusaurus page URL

**Validation Rules**:
- `id` must be unique across collection
- `embedding` dimension must be exactly 1024
- `chapter_id` must be 1-8
- `text` must not be empty
- `source_url` must be valid HTTPS URL

**Relationships**:
- Retrieved by `RetrievalResult` (many-to-many via vector search)
- Referenced in `Citation` entities

**State Transitions**: Immutable after creation (read-only in this feature)

**Storage**: Qdrant Cloud collection `rag_embedding` (created by feature 002)

---

### 3. RetrievalResult

**Purpose**: Set of relevant chunks returned from Qdrant vector search

**Fields**:
- `chunks` (Chunk[], required): List of retrieved book chunks (ordered by similarity)
- `scores` (float[], required): Similarity scores corresponding to chunks (0.0-1.0)
- `query_embedding` (float[], required): Embedded query vector (1024-dim)
- `filters_applied` (dict, optional): Metadata filters used (e.g., {"chapter_id": 3})
- `total_results` (integer, required): Number of chunks returned (typically 5)

**Validation Rules**:
- `chunks` length must equal `scores` length
- `scores` must be in descending order (highest similarity first)
- All `scores` must be in range [0.0, 1.0]
- `total_results` must equal `chunks` length
- `query_embedding` dimension must be 1024

**Relationships**:
- Produced by `services/vector.py` retrieval function
- Consumed by `services/agent.py` for context assembly
- Derived from `Chunk` entities via vector similarity search

**State Transitions**: Ephemeral (not persisted, created per query)

---

### 4. AgentResponse

**Purpose**: The agent's answer to a user query, including citations

**Fields**:
- `answer` (string, required): Generated response text (50-500 characters)
- `sources` (Citation[], required): List of source citations (min 0, max 10)
- `confidence` (string, required): Confidence level ("high", "medium", "low")
- `session_id` (string, optional): Session identifier (if multi-turn conversation)
- `retrieval_count` (integer, required): Number of chunks retrieved
- `processing_time_ms` (integer, required): Total response time in milliseconds

**Validation Rules**:
- `answer` must not be empty
- `sources` must have at least 1 citation if `answer` is not a deflection response
- `confidence` must be one of: "high", "medium", "low"
- `processing_time_ms` must be positive integer
- If `answer` contains "couldn't find information", `sources` can be empty

**Relationships**:
- Produced from one `Query`
- References multiple `Chunk` entities via `sources`
- Appended to `ConversationHistory` (if session_id provided)

**State Transitions**: Immutable after generation (not persisted in MVP)

---

### 5. Citation

**Purpose**: Reference to a book chunk used in generating the response

**Fields**:
- `chapter_id` (integer, required): Chapter number (1-8)
- `section` (string, required): Section title
- `page` (integer, optional): Page number (if available)
- `chunk_id` (string, required): Unique chunk identifier
- `source_url` (string, required): URL to original content
- `relevance_score` (float, optional): Similarity score (0.0-1.0)

**Validation Rules**:
- `chapter_id` must be 1-8
- `section` must not be empty
- `chunk_id` must match a `Chunk.id` in Qdrant
- `source_url` must be valid HTTPS URL
- `relevance_score` must be in range [0.0, 1.0] if provided

**Relationships**:
- Derived from `Chunk` metadata
- Embedded in `AgentResponse.sources`

**State Transitions**: Immutable (created from Qdrant metadata)

**Display Format** (for frontend):
```
Chapter 3, Section "Sensor Fusion" (Page 42)
https://book.example.com/chapter3/sensor-fusion
```

---

### 6. ConversationHistory

**Purpose**: Sequence of queries and responses within a single user session

**Fields**:
- `session_id` (string, required): UUID v4 identifier
- `messages` (Message[], required): Ordered list of user queries and agent responses
- `created_at` (datetime, required): Session start timestamp (UTC)
- `last_updated` (datetime, required): Last message timestamp (UTC)
- `message_count` (integer, required): Total messages in session

**Nested Entity: Message**:
- `role` (string, required): "user" or "assistant"
- `content` (string, required): Message text
- `timestamp` (datetime, required): Message creation time (UTC)

**Validation Rules**:
- `session_id` must be UUID v4 format
- `messages` must alternate between "user" and "assistant" roles
- First message must have `role="user"`
- `created_at` must be <= `last_updated`
- `message_count` must equal `messages` length

**Relationships**:
- Contains multiple `Query` and `AgentResponse` pairs
- Stored in-memory (Python dict keyed by `session_id`)

**State Transitions**:
```
Created (first query) → Active (subsequent queries) → Expired (30 min inactivity)
```

**Storage**: In-memory dictionary in `services/session.py` (MVP)
**Expiration**: Sessions expire after 30 minutes of inactivity

**Future Enhancement**: Persist to Neon Postgres for cross-session history

---

### 7. HealthStatus

**Purpose**: System health check response

**Fields**:
- `status` (string, required): Overall status ("healthy", "degraded", "unhealthy")
- `qdrant_available` (boolean, required): Qdrant connection status
- `openai_available` (boolean, required): OpenAI API availability
- `cohere_available` (boolean, required): Cohere API availability
- `timestamp` (datetime, required): Health check time (UTC)
- `uptime_seconds` (integer, required): Server uptime since startup

**Validation Rules**:
- `status` must be one of: "healthy", "degraded", "unhealthy"
- `status="healthy"` requires all service flags to be `true`
- `status="degraded"` if 1-2 services are `false`
- `status="unhealthy"` if all services are `false`
- `uptime_seconds` must be positive integer

**Relationships**: Standalone entity (no relations)

**State Transitions**: Ephemeral (computed on each /health request)

---

## Data Flow Diagrams

### Single-Turn Query Flow

```
User Query
    ↓
[AskRequest] → Validation (Pydantic)
    ↓
[services/embedding.py] → Cohere API → Query Embedding (1024-dim)
    ↓
[services/vector.py] → Qdrant search (top-5, filters) → RetrievalResult
    ↓
[services/agent.py] → OpenAI Assistants API (retrieval tool) → AgentResponse
    ↓
[AskResponse] → JSON response with citations
    ↓
User receives answer + sources
```

### Multi-Turn Conversation Flow

```
User Query + session_id
    ↓
[services/session.py] → Load ConversationHistory (in-memory)
    ↓
Append user Query to messages
    ↓
[Standard Query Flow] → AgentResponse (with conversation context)
    ↓
Append AgentResponse to ConversationHistory
    ↓
Update last_updated timestamp
    ↓
Return response to user
```

### Health Check Flow

```
GET /health
    ↓
[api/health.py] → Parallel health checks:
    ├─ [services/vector.py] → Qdrant.list_collections() → boolean
    ├─ [services/agent.py] → OpenAI.models.list() → boolean
    └─ [services/embedding.py] → Cohere.check_api_key() → boolean
    ↓
Aggregate results → HealthStatus
    ↓
Return JSON response
```

## Entity Relationships Summary

```
Query 1:1 AgentResponse
Query N:1 ConversationHistory (optional)
AgentResponse 1:N Citation
Citation N:1 Chunk (reference)
RetrievalResult N:N Chunk (via vector search)
ConversationHistory 1:N Message (nested)
```

## Storage Strategy

| Entity | Storage | Persistence | Indexed By |
|--------|---------|-------------|------------|
| Query | Request payload only | No (ephemeral) | N/A |
| Chunk | Qdrant Cloud | Yes (feature 002) | embedding vector, chapter_id, section |
| RetrievalResult | Memory (per-request) | No | N/A |
| AgentResponse | Response payload only | No (MVP) | N/A |
| Citation | Embedded in AgentResponse | No | N/A |
| ConversationHistory | In-memory dict | No (MVP) | session_id |
| HealthStatus | Computed on-demand | No | N/A |

**MVP Storage Decisions**:
- No database persistence (aligned with spec assumption #3)
- Qdrant stores all chunks (read-only from feature 002)
- In-memory session management (30-minute expiration)
- Future: Migrate ConversationHistory to Neon Postgres for analytics

## Data Validation Hierarchy

1. **API Layer** (Pydantic models in `models/requests.py`):
   - Field presence, type checking
   - String length, numeric ranges
   - Format validation (UUID, URLs)

2. **Service Layer** (business logic in `services/`):
   - Cross-field validation (e.g., chapter_filter XOR section_filter)
   - External service availability
   - Retrieved data quality (non-empty chunks)

3. **Response Layer** (Pydantic models in `models/responses.py`):
   - Output format compliance
   - Citation completeness
   - Confidence level assignment
