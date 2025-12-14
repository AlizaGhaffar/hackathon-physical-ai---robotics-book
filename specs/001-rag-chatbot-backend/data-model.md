# Data Model: RAG Chatbot Backend

**Feature**: RAG Chatbot Backend
**Date**: 2025-12-10
**Source**: Extracted from [spec.md](./spec.md) requirements and [research.md](./research.md) best practices

---

## Overview

This document defines the data entities, relationships, and validation rules for the RAG chatbot backend. The system uses two primary data stores:

1. **Qdrant Vector Database**: Stores embedded book chunks with metadata for semantic search
2. **Neon Serverless Postgres**: Stores chat sessions, messages, feedback, and usage analytics

---

## Vector Database Entities (Qdrant)

### BookChunk (Vector Collection)

Represents a semantic chunk of book content stored as a vector embedding in Qdrant.

**Collection Name**: `book_embeddings`
**Vector Dimension**: 1536 (OpenAI text-embedding-3-small)
**Distance Metric**: Cosine similarity

#### Fields

| Field | Type | Description | Required | Indexed |
|-------|------|-------------|----------|---------|
| `id` | UUID | Unique chunk identifier | Yes | Primary |
| `vector` | float[1536] | Embedding vector from OpenAI | Yes | HNSW |
| `payload.chapter_id` | string | Chapter identifier (e.g., "chapter-1") | Yes | Yes |
| `payload.section_id` | string | Section identifier (e.g., "what-is-ros2") | Yes | Yes |
| `payload.chunk_text` | string | Original text content (400-600 tokens) | Yes | No |
| `payload.heading` | string | Section heading for citations | Yes | No |
| `payload.page` | integer | Page number in book (optional) | No | No |
| `payload.chunk_index` | integer | Sequential index within section | Yes | No |
| `payload.token_count` | integer | Exact token count of chunk | Yes | No |
| `payload.indexed_at` | timestamp | When chunk was embedded | Yes | No |

#### Indexes

```python
# Qdrant payload indexes for filtering performance
collection.create_payload_index(
    field_name="chapter_id",
    field_schema="keyword"
)

collection.create_payload_index(
    field_name="section_id",
    field_schema="keyword"
)
```

#### Validation Rules

- `chunk_text` length: 100 - 3000 characters (maps to 400-600 tokens)
- `chapter_id` format: `^chapter-\\d+$` (e.g., "chapter-1", "chapter-2")
- `section_id` format: `^[a-z0-9-]+$` (kebab-case)
- `vector` must have exactly 1536 dimensions
- `similarity_threshold`: 0.7 minimum for retrieval

#### Example Document

```json
{
  "id": "a1b2c3d4-e5f6-7890-abcd-ef1234567890",
  "vector": [0.023, -0.145, 0.089, ...],  // 1536 dimensions
  "payload": {
    "chapter_id": "chapter-1",
    "section_id": "what-is-ros2",
    "chunk_text": "ROS 2 (Robot Operating System 2) is an open-source framework for robot software development. It provides a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms.",
    "heading": "What is ROS 2?",
    "page": 5,
    "chunk_index": 0,
    "token_count": 512,
    "indexed_at": "2025-12-10T10:30:00Z"
  }
}
```

---

## Relational Database Entities (Neon Postgres)

### ChatSession

Represents a conversation session between a user and the chatbot.

#### Schema

```sql
CREATE TABLE chat_sessions (
    session_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NULL,  -- NULL for anonymous sessions
    started_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    last_active_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    is_authenticated BOOLEAN NOT NULL DEFAULT FALSE,
    metadata JSONB NULL,  -- Additional session context
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    updated_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_chat_sessions_user_id ON chat_sessions(user_id);
CREATE INDEX idx_chat_sessions_last_active ON chat_sessions(last_active_at DESC);
```

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `session_id` | UUID | PRIMARY KEY | Unique session identifier |
| `user_id` | UUID | NULL, FOREIGN KEY | User ID from better-auth (NULL for anonymous) |
| `started_at` | TIMESTAMPTZ | NOT NULL | Session start time |
| `last_active_at` | TIMESTAMPTZ | NOT NULL | Last message timestamp (for cleanup) |
| `is_authenticated` | BOOLEAN | NOT NULL | Whether session is linked to authenticated user |
| `metadata` | JSONB | NULL | Optional context (e.g., {"initial_chapter": "chapter-1"}) |
| `created_at` | TIMESTAMPTZ | NOT NULL | Record creation timestamp |
| `updated_at` | TIMESTAMPTZ | NOT NULL | Last update timestamp |

#### Validation Rules

- `user_id`: Must exist in better-auth user table (if not NULL)
- `last_active_at` >= `started_at`
- Anonymous sessions (user_id=NULL) deleted after 24 hours of inactivity
- Authenticated sessions preserved indefinitely

---

### ChatMessage

Represents a single message in a chat session (user query or bot response).

#### Schema

```sql
CREATE TABLE chat_messages (
    message_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id UUID NOT NULL REFERENCES chat_sessions(session_id) ON DELETE CASCADE,
    role VARCHAR(10) NOT NULL CHECK (role IN ('user', 'assistant')),
    content TEXT NOT NULL,
    query_metadata JSONB NULL,  -- For user messages: selected_text, chapter_scope
    response_metadata JSONB NULL,  -- For assistant messages: sources, confidence_score
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_chat_messages_session_id ON chat_messages(session_id, timestamp DESC);
CREATE INDEX idx_chat_messages_timestamp ON chat_messages(timestamp DESC);
```

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `message_id` | UUID | PRIMARY KEY | Unique message identifier |
| `session_id` | UUID | NOT NULL, FK | Parent chat session |
| `role` | VARCHAR(10) | NOT NULL, CHECK | Message author: 'user' or 'assistant' |
| `content` | TEXT | NOT NULL | Message text (query or answer) |
| `query_metadata` | JSONB | NULL | User query metadata (see below) |
| `response_metadata` | JSONB | NULL | Assistant response metadata (see below) |
| `timestamp` | TIMESTAMPTZ | NOT NULL | Message timestamp |
| `created_at` | TIMESTAMPTZ | NOT NULL | Record creation timestamp |

#### Query Metadata Structure (role='user')

```json
{
  "selected_text": "ROS 2 nodes communicate via topics",
  "chapter_scope": "chapter-1",
  "query_length": 45
}
```

#### Response Metadata Structure (role='assistant')

```json
{
  "sources": [
    {
      "chunk_id": "a1b2c3d4-...",
      "chapter_id": "chapter-1",
      "section_id": "core-concepts-nodes",
      "heading": "ROS 2 Nodes",
      "similarity_score": 0.89
    }
  ],
  "confidence_score": 0.85,
  "generation_time_ms": 1842,
  "model_used": "gpt-4-turbo-preview"
}
```

#### Validation Rules

- `content` length: 1 - 10,000 characters
- `role`: Must be exactly 'user' or 'assistant'
- User messages: `query_metadata` may be present, `response_metadata` must be NULL
- Assistant messages: `response_metadata` may be present, `query_metadata` must be NULL

---

### Feedback

Represents user feedback on chatbot responses (thumbs up/down, optional text).

#### Schema

```sql
CREATE TABLE feedback (
    feedback_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES chat_messages(message_id) ON DELETE CASCADE,
    user_id UUID NULL,  -- NULL for anonymous feedback
    rating VARCHAR(20) NOT NULL CHECK (rating IN ('thumbs_up', 'thumbs_down')),
    feedback_text TEXT NULL,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_feedback_message_id ON feedback(message_id);
CREATE INDEX idx_feedback_rating ON feedback(rating);
CREATE INDEX idx_feedback_timestamp ON feedback(timestamp DESC);
```

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `feedback_id` | UUID | PRIMARY KEY | Unique feedback identifier |
| `message_id` | UUID | NOT NULL, FK | Response message being rated |
| `user_id` | UUID | NULL | User who submitted feedback (NULL for anonymous) |
| `rating` | VARCHAR(20) | NOT NULL, CHECK | 'thumbs_up' or 'thumbs_down' |
| `feedback_text` | TEXT | NULL | Optional detailed feedback |
| `timestamp` | TIMESTAMPTZ | NOT NULL | Feedback submission time |
| `created_at` | TIMESTAMPTZ | NOT NULL | Record creation timestamp |

#### Validation Rules

- `message_id`: Must reference a message with `role='assistant'`
- `rating`: Must be exactly 'thumbs_up' or 'thumbs_down'
- `feedback_text` length: 0 - 1,000 characters (if provided)
- One feedback per user per message (UNIQUE constraint on user_id + message_id)

---

### UsageLog

Represents API usage tracking for analytics and monitoring.

#### Schema

```sql
CREATE TABLE usage_logs (
    log_id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    request_id VARCHAR(50) NOT NULL UNIQUE,
    endpoint VARCHAR(100) NOT NULL,
    user_id UUID NULL,
    session_id UUID NULL,
    http_method VARCHAR(10) NOT NULL,
    status_code INTEGER NOT NULL,
    response_time_ms INTEGER NOT NULL,
    error_message TEXT NULL,
    timestamp TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW(),
    created_at TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()
);

CREATE INDEX idx_usage_logs_endpoint ON usage_logs(endpoint);
CREATE INDEX idx_usage_logs_status_code ON usage_logs(status_code);
CREATE INDEX idx_usage_logs_timestamp ON usage_logs(timestamp DESC);
CREATE INDEX idx_usage_logs_request_id ON usage_logs(request_id);
```

#### Fields

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `log_id` | UUID | PRIMARY KEY | Unique log identifier |
| `request_id` | VARCHAR(50) | NOT NULL, UNIQUE | Request trace ID |
| `endpoint` | VARCHAR(100) | NOT NULL | API endpoint (e.g., "/api/query") |
| `user_id` | UUID | NULL | User making request (if authenticated) |
| `session_id` | UUID | NULL | Chat session ID (if applicable) |
| `http_method` | VARCHAR(10) | NOT NULL | HTTP method (GET, POST, etc.) |
| `status_code` | INTEGER | NOT NULL | HTTP response status code |
| `response_time_ms` | INTEGER | NOT NULL | Response time in milliseconds |
| `error_message` | TEXT | NULL | Error message if status_code >= 400 |
| `timestamp` | TIMESTAMPTZ | NOT NULL | Request timestamp |
| `created_at` | TIMESTAMPTZ | NOT NULL | Record creation timestamp |

#### Validation Rules

- `status_code`: 100-599 (valid HTTP codes)
- `response_time_ms`: >= 0
- `error_message`: Required when status_code >= 400, NULL otherwise

---

## Entity Relationships

```
ChatSession (1) ──── (Many) ChatMessage
ChatMessage (1) ──── (Many) Feedback
```

**Cascade Deletion Rules**:
- Deleting a `ChatSession` cascades to delete all `ChatMessage` records
- Deleting a `ChatMessage` cascades to delete all `Feedback` records
- `UsageLog` is independent (no foreign key dependencies for audit integrity)

---

## State Transitions

### ChatSession Lifecycle

```
CREATED (is_authenticated=false)
  → USER_AUTHENTICATED (user_id assigned, is_authenticated=true)
  → ACTIVE (messages being exchanged, last_active_at updates)
  → INACTIVE (no activity for 24h)
  → DELETED (anonymous sessions only; authenticated sessions preserved)
```

### ChatMessage Lifecycle

```
CREATED (user sends query)
  → RESPONSE_GENERATED (assistant message created)
  → FEEDBACK_COLLECTED (optional feedback added)
```

---

## Performance Considerations

### Qdrant Vector Database

- **Read Performance**: <100ms for top-5 retrieval with metadata filtering (HNSW index)
- **Write Performance**: Batch insert 100-500 chunks per request
- **Storage**: ~6KB per chunk (1536 float32 dimensions + metadata)
- **Estimated Size**: 300 chunks × 6KB = 1.8MB (well under 1GB free tier)

### Neon Postgres

- **Connection Pooling**: 5-20 async connections (asyncpg pool)
- **Indexes**: All foreign keys and query-critical fields indexed
- **Cleanup Policy**: Anonymous sessions deleted after 24h of inactivity
- **Read Performance**: <50ms for recent chat history (indexed session_id + timestamp)

---

## Data Retention & Privacy

1. **Anonymous Sessions**: Auto-deleted after 24 hours of inactivity
2. **Authenticated Sessions**: Preserved indefinitely; users can request deletion (GDPR compliance)
3. **Usage Logs**: Retained for 90 days for monitoring, then archived/deleted
4. **Feedback**: Preserved indefinitely for quality improvement
5. **Vector Database**: Content re-indexed when book updates; old vectors removed

---

## Security & Validation

1. **Input Sanitization**: All text fields sanitized to prevent SQL injection
2. **Parameterized Queries**: SQLAlchemy ORM prevents injection attacks
3. **UUID Generation**: Cryptographically secure random UUIDs
4. **Timestamp Validation**: All timestamps validated to be reasonable (not future dates)
5. **JSONB Validation**: Metadata fields validated against Pydantic schemas before storage

---

## Migrations Strategy

**Tool**: Alembic (SQLAlchemy migrations)

```bash
# Generate migration from models
alembic revision --autogenerate -m "Initial schema"

# Apply migrations
alembic upgrade head
```

**Migration Files Location**: `backend/migrations/versions/`

---

**Status**: Data model complete and ready for API contract generation.
