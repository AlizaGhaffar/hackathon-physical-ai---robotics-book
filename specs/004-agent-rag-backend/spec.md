# Feature Specification: Agent-Based RAG Backend

**Feature Branch**: `004-agent-rag-backend`
**Created**: 2025-12-13
**Status**: Draft
**Input**: User description: "Agent-Based RAG Backend - Build a FastAPI backend that uses the OpenAI Agents SDK and integrates retrieval from Qdrant. The agent should answer user questions strictly based on book content."

## Clarifications

### Session 2025-12-13

- Q: What should be explicitly excluded from this feature's scope to prevent scope creep? → A: Basic RAG only - no re-ranking, no query expansion, no result caching
- Q: How long should conversation history persist in memory before expiring? → A: Session expires after 30 minutes of inactivity
- Q: What structure should error responses follow for consistent client handling? → A: Standard JSON: {"error": "error_type", "message": "human message", "detail": {...}}
- Q: Should the system retry failed requests to external services (Qdrant, OpenAI, Cohere)? → A: Retry 3 times with exponential backoff (1s, 2s, 4s delays)
- Q: Which metrics are critical for monitoring production health and debugging issues? → A: Core metrics: latency (p50/p95/p99), error rate by type, avg similarity score, chunks retrieved per query

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Single-Turn Book Question (Priority: P1)

A user asks a direct question about book content and receives an accurate answer grounded only in retrieved book chunks.

**Why this priority**: This is the core value proposition - answering book questions accurately without hallucination. Without this, the system has no value.

**Independent Test**: Can be fully tested by sending a POST request to `/ask` with a simple question like "What is reinforcement learning?" and verifying the response contains only information from book content with proper citations.

**Acceptance Scenarios**:

1. **Given** the vector database contains embedded book chunks, **When** user asks "What are the three main components of a robot?", **Then** the agent retrieves relevant chunks from Qdrant and responds with an answer strictly based on retrieved content, including source references
2. **Given** the user asks a question outside book scope, **When** the agent searches Qdrant and finds no relevant chunks (low similarity scores), **Then** the agent responds "I don't have information about that in the book content available to me"
3. **Given** the user asks a vague question, **When** the agent retrieves multiple relevant chunks from different chapters, **Then** the agent synthesizes a coherent answer citing all relevant sections

---

### User Story 2 - Multi-Turn Conversation (Priority: P2)

A user has a back-and-forth conversation with follow-up questions, and the agent maintains context while staying grounded in book content.

**Why this priority**: Enables natural dialogue flow and deeper exploration of topics. Enhances user experience but requires P1 to work first.

**Independent Test**: Can be fully tested by sending a sequence of related questions (e.g., "What is sensor fusion?" followed by "How does it work in autonomous vehicles?") and verifying the agent maintains conversation context while only using book content.

**Acceptance Scenarios**:

1. **Given** the user previously asked about "robot perception", **When** they follow up with "Can you explain more about the camera types?", **Then** the agent understands the context and retrieves relevant chunks about cameras in robot perception systems
2. **Given** a multi-turn conversation about path planning, **When** the user asks "What are the limitations?", **Then** the agent knows to search for limitations of path planning algorithms (not a generic search)
3. **Given** the conversation has drifted away from book topics, **When** the user asks an unrelated question, **Then** the agent still refuses to answer outside book scope even with conversation history

---

### User Story 3 - Chapter-Scoped Queries (Priority: P3)

A user wants to ask questions limited to a specific chapter or section of the book.

**Why this priority**: Useful for focused study or research, but not essential for initial launch. Users can get value from P1 and P2 without scoping.

**Independent Test**: Can be fully tested by sending a request with metadata filter `{"chapter": "Chapter 3"}` and verifying all retrieved chunks and answers come only from Chapter 3.

**Acceptance Scenarios**:

1. **Given** the user specifies `chapter_filter: "Chapter 5"`, **When** they ask "What are the key concepts?", **Then** the agent only searches and retrieves from Chapter 5 content
2. **Given** the user specifies `section_filter: "Sensor Fusion"`, **When** they ask about sensors, **Then** results are limited to the Sensor Fusion section even if other sections mention sensors
3. **Given** the user specifies an invalid chapter name, **When** they submit a query, **Then** the system returns a clear error message listing valid chapter names

---

### Edge Cases

- What happens when the user asks a question and Qdrant returns empty results (no relevant chunks)?
  - Agent should respond with "I don't have information about that in the book" rather than hallucinating
- How does the system handle extremely long questions (>1000 words)?
  - System should truncate or summarize the question for embedding while preserving intent
- What happens when Qdrant or OpenAI services are temporarily unavailable?
  - System should return a clear error message (503 Service Unavailable) with retry guidance
- How does the agent handle ambiguous pronouns in follow-up questions without sufficient context?
  - Agent should ask clarifying questions or explicitly state uncertainty
- What happens when retrieved chunks contain contradictory information?
  - Agent should acknowledge both perspectives and cite the specific sources

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST create an OpenAI agent using the OpenAI Agents SDK with a system prompt that enforces strict grounding in retrieved content only
- **FR-002**: System MUST define a retrieval tool that the agent can call to search Qdrant vector database for relevant book chunks
- **FR-003**: System MUST expose a `/ask` endpoint accepting user question, optional conversation history, and optional metadata filters (chapter, section)
- **FR-004**: System MUST expose a `/health` endpoint returning service status including Qdrant connection, OpenAI API availability, and agent initialization state
- **FR-005**: System MUST retrieve book chunks from Qdrant using semantic similarity search with a configurable top-k parameter (default: 5 chunks)
- **FR-006**: System MUST pass retrieved chunks to the agent as context before generating responses
- **FR-007**: System MUST maintain conversation history across multiple turns within a single session
- **FR-008**: System MUST include source citations in responses indicating which book chunks were used
- **FR-009**: System MUST reject or deflect questions when no relevant book content is found (similarity score below threshold)
- **FR-010**: System MUST support metadata filtering on chapter and section fields when querying Qdrant
- **FR-011**: System MUST log all queries, retrieved chunks, agent tool calls, and final responses for debugging and monitoring, and MUST emit the following metrics: response latency percentiles (p50, p95, p99), error rate by error type, average similarity score per query, and number of chunks retrieved per query
- **FR-012**: System MUST use environment variables for all configuration (API keys, endpoints, model names, thresholds)
- **FR-013**: System MUST validate input requests using Pydantic models (question length, filter values)
- **FR-014**: System MUST handle errors gracefully and return structured error responses with appropriate HTTP status codes, using the format: `{"error": "error_type", "message": "human-readable message", "detail": {...}}` where detail contains additional context (e.g., validation errors, service status)
- **FR-015**: System MUST implement retry logic for external service calls (Qdrant, OpenAI, Cohere) with 3 retry attempts using exponential backoff (1s, 2s, 4s delays) before failing
- **FR-016**: System MUST prevent the agent from using external knowledge or tools beyond the defined retrieval tool

### Key Entities

- **Query**: User's question or statement, along with optional conversation history and metadata filters (chapter, section)
- **Chunk**: A segment of book text previously embedded and stored in Qdrant, with metadata (chapter, section, page, source URL)
- **AgentResponse**: The agent's answer including response text, source citations (chunk IDs/URLs), and confidence indicators
- **ConversationHistory**: Sequence of prior user queries and agent responses within a session, used for context in multi-turn interactions
- **RetrievalResult**: Set of book chunks returned from Qdrant search, including similarity scores and metadata

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Agent responds to single-turn questions with answers grounded only in book content (0% hallucination in test set of 50 questions)
- **SC-002**: Agent maintains context across at least 5 conversation turns without losing coherence or introducing external information
- **SC-003**: System retrieves and returns responses within 3 seconds for 95% of queries
- **SC-004**: System correctly refuses to answer out-of-scope questions at least 90% of the time (evaluated on 20 off-topic test questions)
- **SC-005**: All responses include source citations linking back to specific book chunks or sections
- **SC-006**: System handles at least 50 concurrent users without degradation in response time or accuracy
- **SC-007**: Health endpoint reports accurate service status with 99.9% uptime during testing period
- **SC-008**: Metadata filtering (chapter/section) returns only chunks from specified scope in 100% of filtered queries

## Assumptions

1. **Vector Database Pre-populated**: Assumes Qdrant already contains embedded book chunks from the embedding pipeline (feature 002). This feature does not include document ingestion.

2. **OpenAI Agents SDK Compatibility**: Assumes OpenAI Agents SDK supports tool/function calling patterns similar to the Assistants API, allowing custom retrieval tool definition.

3. **Session Management**: Conversation history is maintained in-memory for a single session (no persistent cross-session history). Sessions expire after 30 minutes of inactivity. Clients receive a session_id on first request and must include it in subsequent requests to maintain context.

4. **Embedding Model Consistency**: Assumes query embeddings use the same model (Cohere embed-english-v3.0, 1024 dimensions) as document chunks for compatibility.

5. **Authentication Not Required**: Assumes this backend is for internal/development use without authentication or rate limiting (can be added in future iteration).

6. **Similarity Threshold**: Assumes a default similarity score threshold of 0.7 (on 0-1 scale) below which results are considered irrelevant. This may require tuning based on actual performance.

7. **Chunk Size Alignment**: Assumes book chunks are sized appropriately (500 tokens with 100-token overlap from feature 002) to fit within agent context windows and provide coherent answers.

8. **Citation Format**: Source citations will reference chunk metadata (chapter, section, page) rather than exact page numbers, as digital books may not have fixed pagination.

## Out of Scope

The following are explicitly NOT included in this feature to maintain focus on core RAG functionality:

1. **Advanced Retrieval Techniques**: No re-ranking with cross-encoders, no query expansion, no hybrid search (dense + sparse)
2. **Result Caching**: No caching layer for frequently asked questions or retrieved chunks
3. **Optimization Layers**: No semantic caching, no embedding caching, no response memoization
4. **Alternative Retrieval Methods**: Single-stage dense retrieval only (no multi-stage retrieval pipelines)
