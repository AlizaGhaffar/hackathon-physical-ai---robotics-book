# Feature Specification: RAG Chatbot Backend

**Feature Branch**: `001-rag-chatbot-backend`
**Created**: 2025-12-08
**Status**: Draft
**Input**: Build RAG chatbot backend with OpenAI Agents SDK, FastAPI, Qdrant vector storage, and Neon Postgres database to answer questions about book content, including user-selected text

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Basic Book Q&A (Priority: P1)

A reader is studying the book and has a question about a concept they just read. They type their question into the chatbot (e.g., "What is ROS 2?") and receive an accurate answer based on the relevant book sections, along with citations showing which chapter/section the answer came from.

**Why this priority**: This is the core value proposition of the RAG chatbot. Without accurate question-answering on book content, the chatbot provides no value. This is the MVP that must work first.

**Independent Test**: Can be fully tested by loading book content into the vector database, asking a known question, and verifying the answer matches the book content with correct citations. Delivers immediate value to readers seeking clarification.

**Acceptance Scenarios**:

1. **Given** book content is indexed in the vector database, **When** a user asks "What is ROS 2?", **Then** the system returns an accurate answer with citations to the relevant chapter and section
2. **Given** a user asks a question about Chapter 1 content, **When** the query is processed, **Then** the response is generated within 3 seconds
3. **Given** a user asks a question not covered in the book, **When** the system cannot find relevant context, **Then** it responds with "I couldn't find that information in the book"
4. **Given** the book contains similar concepts in multiple chapters, **When** a user asks a general question, **Then** the system retrieves context from the most relevant sections across all chapters

---

### User Story 2 - Selected Text Q&A (Priority: P2)

A reader is confused about a specific paragraph or code snippet they're currently reading. They select the text in the UI, type a question about it (e.g., "Can you explain this code?"), and receive an answer focused specifically on that selected content rather than the entire book.

**Why this priority**: This enhances precision and user experience by allowing focused queries. While not essential for MVP, it significantly improves usability for learners who need clarification on specific sections.

**Independent Test**: Can be tested by selecting a paragraph from the book, asking a question about it, and verifying the answer focuses on the selected text rather than retrieving unrelated content. Delivers targeted help for confusion on specific sections.

**Acceptance Scenarios**:

1. **Given** a user selects a paragraph about "ROS 2 nodes", **When** they ask "How does this work?", **Then** the system prioritizes the selected text in context retrieval
2. **Given** a user selects code snippet A and asks about it, **When** the query is processed, **Then** the response references the selected code specifically, not other code examples
3. **Given** selected text is very short (<100 characters), **When** the system processes it, **Then** it automatically expands context by retrieving surrounding chunks
4. **Given** a user provides selected text with metadata (chapter, section), **When** the query is sent, **Then** the system uses this metadata to enrich context retrieval

---

### User Story 3 - Chapter-Scoped Queries (Priority: P2)

A reader wants to ask questions specifically about Chapter 2 without mixing in information from other chapters. They can specify the chapter scope (either via UI selection or in their query), and the system only retrieves context from that chapter.

**Why this priority**: As the book grows to multiple chapters, users may want focused answers from specific chapters to avoid confusion. This enables progressive learning without information overload.

**Independent Test**: Can be tested by asking a question that could be answered from multiple chapters while specifying Chapter 2 scope, and verifying only Chapter 2 content is used. Delivers focused learning experience for chapter-specific study.

**Acceptance Scenarios**:

1. **Given** a user specifies "Chapter 2" scope, **When** they ask "What is Gazebo?", **Then** the system retrieves context only from Chapter 2 vectors
2. **Given** book content across 3 chapters, **When** a user asks an unscoped question, **Then** the system retrieves the most relevant content regardless of chapter
3. **Given** a user is viewing Chapter 1 in the UI, **When** they ask a question, **Then** the system defaults to Chapter 1 scope unless otherwise specified
4. **Given** a chapter-scoped query returns no relevant results, **When** the similarity threshold isn't met, **Then** the system suggests checking other chapters

---

### User Story 4 - Chat History Persistence (Priority: P3)

A reader asks multiple questions during a study session. When they return later (after logout/login), they can see their previous questions and answers, allowing them to review their learning progress and continue conversations.

**Why this priority**: This improves long-term learning by enabling conversation continuity and review. While nice to have, it's not essential for the core Q&A functionality.

**Independent Test**: Can be tested by creating a chat session, asking questions, logging out, logging back in, and verifying the chat history is preserved. Delivers continuity for returning users.

**Acceptance Scenarios**:

1. **Given** a user asks 3 questions in a session, **When** they log out and log back in, **Then** their chat history shows all previous questions and answers
2. **Given** an authenticated user, **When** they start a chat, **Then** the system creates a session linked to their user ID
3. **Given** a user has multiple chat sessions, **When** they view history, **Then** sessions are organized by date/time
4. **Given** an anonymous user (not logged in), **When** they ask questions, **Then** chat history persists for the browser session only (no database storage)

---

### User Story 5 - Response Quality Feedback (Priority: P3)

After receiving an answer, a reader can rate it (thumbs up/down) and optionally provide text feedback. This data is stored to help improve the chatbot over time and identify problematic queries.

**Why this priority**: Enables quality monitoring and continuous improvement. Not critical for initial launch but valuable for long-term maintenance.

**Independent Test**: Can be tested by receiving an answer, submitting feedback (rating + text), and verifying it's stored in the database. Delivers insights for improvement.

**Acceptance Scenarios**:

1. **Given** a user receives an answer, **When** they click thumbs up, **Then** the feedback is stored with the query ID and timestamp
2. **Given** a user clicks thumbs down, **When** they optionally provide text feedback, **Then** both rating and text are stored together
3. **Given** an admin reviews feedback, **When** they query the database, **Then** they see feedback organized by query with user context
4. **Given** a user provides feedback, **When** it's submitted, **Then** the UI shows confirmation without interrupting their workflow

---

### Edge Cases

- What happens when the OpenAI API is down or rate-limited? System should return a friendly error message and log the failure for monitoring
- What happens when Qdrant vector database is unreachable? System should fail gracefully with appropriate error message and retry logic
- What happens when a user submits an extremely long question (>1000 characters)? System should truncate or return a validation error
- What happens when book content is re-indexed with updated text? Old vectors should be removed/replaced to avoid duplicate or stale information
- What happens when multiple users query simultaneously? System should handle concurrent requests without degradation up to 50 concurrent users
- What happens when selected text doesn't exist in the vector database? System should validate and either reject or fall back to general query
- What happens when a user's chat history grows very large (>1000 messages)? Consider pagination or archival strategy
- What happens when database connection pool is exhausted? System should queue requests or return 503 Service Unavailable

## Requirements *(mandatory)*

### Functional Requirements

**Core RAG Pipeline:**
- **FR-001**: System MUST accept user queries as text input and return natural language answers
- **FR-002**: System MUST convert user queries to embeddings using OpenAI embedding models
- **FR-003**: System MUST retrieve the top 3-5 most semantically similar book chunks from Qdrant vector database
- **FR-004**: System MUST send retrieved context to OpenAI GPT model to generate final answer
- **FR-005**: System MUST include citations (chapter, section, page) with each answer showing source of information
- **FR-006**: System MUST respond "I couldn't find that information in the book" when no relevant context is found

**Selected Text Queries:**
- **FR-007**: System MUST accept optional selected_text parameter with user queries
- **FR-008**: System MUST prioritize selected text in context retrieval when provided
- **FR-009**: System MUST expand context by retrieving surrounding chunks when selected text is very short (<100 chars)
- **FR-010**: System MUST validate that selected text exists in the vector database before processing

**Chapter Scoping:**
- **FR-011**: System MUST accept optional chapter_id parameter to scope queries to specific chapters
- **FR-012**: System MUST filter vector search by chapter metadata when chapter scope is specified
- **FR-013**: System MUST support unscoped queries that search across all chapters

**Chat History:**
- **FR-014**: System MUST store all queries and responses in Neon Postgres database with timestamps
- **FR-015**: System MUST link chat messages to user sessions for authenticated users
- **FR-016**: System MUST persist chat sessions across login/logout cycles
- **FR-017**: System MUST support anonymous chat sessions (browser session only, no database persistence)

**Feedback:**
- **FR-018**: System MUST accept user feedback (rating: thumbs up/down, optional text) for each response
- **FR-019**: System MUST store feedback linked to specific query IDs in the database
- **FR-020**: System MUST timestamp all feedback submissions

**Content Management:**
- **FR-021**: System MUST provide admin endpoint to generate embeddings for book content
- **FR-022**: System MUST chunk book content into semantic units (500-1000 tokens with 100-200 token overlap)
- **FR-023**: System MUST store chunk metadata (chapter_id, section_id, heading, page) with each vector
- **FR-024**: System MUST support batch embedding generation (process multiple chunks per API call)
- **FR-025**: System MUST prevent duplicate vectors for the same content

**API Design:**
- **FR-026**: System MUST expose POST /api/query endpoint for question-answering
- **FR-027**: System MUST expose POST /api/embed endpoint for admin content indexing
- **FR-028**: System MUST expose GET /api/health endpoint for health checks
- **FR-029**: System MUST expose POST /api/feedback endpoint for user feedback
- **FR-030**: System MUST validate all request payloads using Pydantic models
- **FR-031**: System MUST return appropriate HTTP status codes (200, 400, 422, 500, 503)
- **FR-032**: System MUST include request_id in all responses for tracing

**Performance:**
- **FR-033**: System MUST respond to queries within 3 seconds (p95)
- **FR-034**: System MUST support streaming responses (Server-Sent Events) for progressive display
- **FR-035**: System MUST handle 50 concurrent users without degradation

**Security:**
- **FR-036**: System MUST store all API keys (OpenAI, Qdrant) in environment variables
- **FR-037**: System MUST store database credentials in environment variables
- **FR-038**: System MUST sanitize user inputs to prevent injection attacks
- **FR-039**: System MUST implement rate limiting (100 requests/minute per user)
- **FR-040**: System MUST require admin authentication for content embedding endpoints

**Error Handling:**
- **FR-041**: System MUST handle OpenAI API failures gracefully with user-friendly error messages
- **FR-042**: System MUST implement retry logic with exponential backoff for failed API calls (max 3 retries)
- **FR-043**: System MUST log all errors with sufficient context for debugging
- **FR-044**: System MUST never expose API keys or credentials in error messages or logs

### Key Entities

- **Query**: A user's question submitted to the chatbot. Attributes: query_id, user_id (optional), session_id, query_text, chapter_scope (optional), selected_text (optional), timestamp
- **Response**: The chatbot's answer to a query. Attributes: response_id, query_id, answer_text, sources (list of chunks), confidence_score, generation_time, timestamp
- **ChatSession**: A conversation session between user and chatbot. Attributes: session_id, user_id (optional), started_at, last_active_at, is_authenticated
- **ChatMessage**: A single message in a chat session (can be user query or bot response). Attributes: message_id, session_id, role (user/assistant), content, timestamp
- **Feedback**: User feedback on a response. Attributes: feedback_id, query_id, user_id (optional), rating (thumbs_up/thumbs_down), feedback_text (optional), timestamp
- **BookChunk**: A semantic chunk of book content stored as vector. Attributes: chunk_id, chapter_id, section_id, chunk_text, embedding_vector, metadata (heading, page), indexed_at
- **UsageLog**: Tracking for analytics and monitoring. Attributes: log_id, endpoint, user_id (optional), request_id, response_time, status_code, timestamp

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Users receive accurate answers to book questions within 3 seconds for 95% of queries
- **SC-002**: Users asking questions about Chapter 1 content receive answers citing Chapter 1 sections 90% of the time (high precision)
- **SC-003**: Users asking questions with selected text receive responses focused on that text 95% of the time
- **SC-004**: System successfully handles 50 concurrent users asking questions without response time degradation
- **SC-005**: Users receive "not found" message when asking off-topic questions 100% of the time (no hallucinations)
- **SC-006**: Authenticated users can access their complete chat history after logout/login 100% of the time
- **SC-007**: System maintains 99.5% uptime during normal operation (excluding planned maintenance)
- **SC-008**: Admin can index an entire chapter's content (5000+ words) in under 2 minutes
- **SC-009**: User feedback is successfully stored for 100% of submissions
- **SC-010**: Answers include source citations (chapter/section) 100% of the time when context is found

## Assumptions

1. **Book Format**: Book content is available in a structured format (Markdown, HTML, or JSON) that can be parsed into chapters and sections
2. **Authentication**: User authentication is handled by the existing better-auth.com system; this backend receives user_id from authenticated requests
3. **Frontend Integration**: Frontend chatbot UI already exists and will call these backend APIs
4. **Content Updates**: Book content updates are infrequent enough that re-indexing can be done manually via admin endpoint
5. **Language**: All book content and queries are in English (Urdu translation handled separately by frontend)
6. **Deployment**: Backend will be deployed as a separate service from the Docusaurus frontend
7. **Volume**: Expected query volume is moderate (hundreds of queries per day, not thousands per hour)
8. **OpenAI Quotas**: OpenAI API quotas are sufficient for expected usage (embeddings + GPT completions)
9. **Qdrant Free Tier**: Qdrant Cloud Free Tier limits (1GB storage, 100 collections) are sufficient for book content
10. **Neon Free Tier**: Neon Serverless Postgres free tier limits are sufficient for chat history and analytics

## Out of Scope

The following are explicitly NOT part of this specification:

1. **Frontend UI**: Chatbot interface design, React components, UI/UX
2. **Authentication System**: User registration, login, session management (handled by better-auth.com)
3. **Translation**: Urdu translation of answers (handled by separate translation service)
4. **Personalization**: Adapting answers based on user profile/background
5. **Multi-Modal**: Processing images, diagrams, or videos from the book
6. **Voice Input**: Speech-to-text or text-to-speech capabilities
7. **Advanced RAG**: Hybrid search, re-ranking with cross-encoders, query expansion
8. **Analytics Dashboard**: Admin UI for viewing usage statistics and feedback
9. **Load Balancing**: Horizontal scaling, auto-scaling infrastructure
10. **Payment/Billing**: Any premium features or usage-based billing
11. **Mobile Apps**: Native iOS/Android chatbot interfaces
12. **Offline Mode**: Caching or offline query capabilities

## Dependencies

**External Services:**
- OpenAI API (embeddings + GPT-4/3.5-turbo)
- Qdrant Cloud Free Tier (vector database)
- Neon Serverless Postgres (relational database)

**Infrastructure:**
- Deployment platform for FastAPI backend (Vercel, Railway, or similar)
- Environment variable management for secrets
- CORS configuration for frontend domain

**Data Dependencies:**
- Book content in parseable format (Markdown/HTML/JSON)
- Existing user authentication system (better-auth.com)
- Frontend chatbot UI to consume backend APIs

**Libraries/SDKs:**
- OpenAI Agents SDK (per user-provided example)
- FastAPI framework
- qdrant-client Python SDK
- asyncpg + SQLAlchemy (Postgres)
- Pydantic (validation)

## Technical Constraints

While avoiding implementation details, these constraints affect the feature design:

1. **Response Time Budget**: Total query-to-answer time must be <3s, distributed across: embedding query (0.3s), vector search (0.5s), GPT generation (2s), overhead (0.2s)
2. **Context Window**: Retrieved context must fit within OpenAI model limits minus prompt overhead (~6000 tokens for GPT-3.5-turbo)
3. **Free Tier Limits**: Must operate within Qdrant (1GB vectors) and Neon (free tier storage/connections) constraints
4. **Concurrency**: Must support 50 concurrent users as design target
5. **Rate Limiting**: Must enforce 100 req/min per user to prevent abuse and manage API costs
6. **CORS**: Must allow frontend domain(s) to call APIs

## Credential Management & Environment Setup

**Status**: ✅ Configured (2025-12-10)

### Environment Files

Two environment files are maintained in the project root:

1. **`.env`** (NEVER commit - contains actual secrets)
   - Contains all production credentials for Qdrant, Neon, and OpenAI
   - Protected by `.gitignore` to prevent accidental commits
   - Location: `/book/.env`

2. **`.env.example`** (Template - safe to commit)
   - Template with placeholder values
   - Documentation for required environment variables
   - Developers copy this to `.env` and fill in actual values
   - Location: `/book/.env.example`

### Configured Credentials

The following credentials have been stored in `.env`:

**Qdrant Cloud (Vector Database):**
- `QDRANT_API_KEY`: JWT token for Qdrant Cloud API authentication
- `QDRANT_CLUSTER_ID`: Cluster identifier (74f15079-969a-487c-91a2-6693752617a9)
- `QDRANT_URL`: Europe-west3 GCP endpoint
- `QDRANT_COLLECTION_NAME`: book_embeddings

**Neon Serverless Postgres (Relational Database):**
- `DATABASE_URL`: Full connection string with SSL enabled
- Individual components (DB_HOST, DB_NAME, DB_USER, DB_PASSWORD, DB_PORT)
- Region: us-east-1 AWS
- Connection pooling enabled

**OpenAI API (Embeddings & Generation):**
- `OPENAI_API_KEY`: Project API key
- `OPENAI_MODEL`: gpt-4-turbo-preview (for answer generation)
- `OPENAI_EMBEDDING_MODEL`: text-embedding-3-small (for vector embeddings)

### Security Measures

1. **Version Control Protection**:
   - `.env` is listed in `.gitignore` (line 17, 68)
   - Pattern `.env.*` excluded (except `.env.example`)
   - Additional protection for `*.key`, `*.pem`, `credentials.json`

2. **Environment Variable Validation**:
   - Application MUST validate all required env vars exist at startup (fail fast)
   - Missing credentials should trigger clear error message before any API calls

3. **Secrets Rotation Policy**:
   - OpenAI API keys: Rotate every 90 days or immediately if compromised
   - Qdrant API key: Regenerate from cloud console if exposed
   - Neon credentials: Reset password via Neon console if leaked
   - All rotations must update `.env` file immediately

4. **Access Control**:
   - `.env` file permissions: Read/write for owner only (chmod 600)
   - Never log, print, or expose env var values in application code
   - Never send env vars in error messages or API responses

### Configuration Variables

Beyond credentials, `.env` includes tunable settings:

**Application Settings:**
- `ENVIRONMENT`: development/staging/production
- `API_HOST`, `API_PORT`: Server binding configuration
- `CORS_ORIGINS`: Allowed frontend domains (comma-separated)

**RAG Pipeline Tuning:**
- `CHUNK_SIZE`: 1000 (tokens per chunk)
- `CHUNK_OVERLAP`: 200 (overlapping tokens between chunks)
- `TOP_K_RESULTS`: 5 (number of chunks retrieved per query)
- `SIMILARITY_THRESHOLD`: 0.7 (minimum cosine similarity)
- `MAX_RESPONSE_TOKENS`: 500 (max tokens in generated answer)
- `TEMPERATURE`: 0.7 (GPT creativity/randomness)

**Feature Flags:**
- `ENABLE_STREAMING`: true (server-sent events for responses)
- `ENABLE_CHAT_HISTORY`: true (persist conversations in Neon)
- `ENABLE_FEEDBACK`: true (thumbs up/down collection)
- `ENABLE_SELECTED_TEXT_QUERY`: true (focused queries on selected text)
- `ENABLE_CHAPTER_SCOPING`: true (chapter-filtered queries)

**Security Settings:**
- `RATE_LIMIT_PER_MINUTE`: 100 (requests per user per minute)
- `ADMIN_API_KEY`: Separate key for admin endpoints (must change in production)

### Setup Instructions for Developers

1. Copy template: `cp .env.example .env`
2. Request credentials from team lead or project maintainer
3. Fill in actual values in `.env` (never commit this file)
4. Verify setup: Run health check endpoint to test all connections
5. Test locally: Ensure all API integrations work before deployment

### Production Deployment Notes

- For production environments, use platform-specific secret management:
  - **Vercel**: Environment Variables in project settings
  - **Railway**: Environment Variables in project dashboard
  - **AWS**: AWS Secrets Manager or Parameter Store
  - **Azure**: Azure Key Vault
- Never deploy with `.env` file in production containers
- Use separate production credentials (never reuse dev credentials)
- Enable credential rotation automation where supported
