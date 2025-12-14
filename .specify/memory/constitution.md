<!--
Sync Impact Report
------------------
Version Change: 1.2.0 → 1.3.0
Modified Principles: None (existing principles retained)
Added Sections:
  - XIII. RAG Backend Architecture
  - XIV. Embedding Pipeline Excellence
  - XV. Vector Storage Strategy
  - XVI. Context Retrieval Precision
  - XVII. Response Generation Quality
  - XVIII. Database Management
  - XIX. Selected Text Query Support
  - RAG Backend Technology Stack
  - RAG API Standards
Removed Sections: None
Templates Requiring Updates:
  - ✅ .specify/templates/plan-template.md (validated - RAG backend aligns with web app structure)
  - ✅ .specify/templates/spec-template.md (validated - RAG features can be specified as user stories)
  - ✅ .specify/templates/tasks-template.md (validated - RAG tasks follow phase-based structure)
Follow-up TODOs:
  - Create RAG backend specification with FastAPI architecture
  - Design embedding pipeline for book content processing
  - Set up Qdrant Cloud Free Tier vector database
  - Configure Neon Serverless Postgres for metadata storage
  - Implement selected text querying feature
  - Integrate with existing frontend chatbot UI
-->

# Physical AI Textbook - Multi-Chapter Constitution with Chapter 3 Climax

## Core Principles

### I. Multi-Chapter Architecture
**PRINCIPLE**: Development MUST support multiple chapters (starting with Chapter 2) while maintaining Chapter 1's proven architecture, with each chapter being independently functional yet seamlessly integrated.

**RATIONALE**: Chapter 1 successfully demonstrated a production-ready educational module with full bonus features. Expanding to Chapter 2 proves architectural scalability and reusability, which were explicit design goals. Multi-chapter support increases educational value and demonstrates system maturity without compromising existing functionality.

**NON-NEGOTIABLE RULES**:
- All architectural patterns from Chapter 1 MUST be reused (no reinvention)
- Chapter 2 MUST match Chapter 1's design language and UX patterns
- All bonus features MUST work identically in both chapters
- Navigation between chapters MUST be intuitive and consistent
- Each chapter MUST be independently accessible and functional
- Progression tracking MUST work across all chapters
- Architecture MUST support addition of Chapter N without modifying Chapter 1 or 2 core logic

### II. Consistency First
**PRINCIPLE**: Chapter 2 MUST achieve visual, functional, and experiential parity with Chapter 1, creating a cohesive multi-chapter learning experience.

**RATIONALE**: Inconsistent UX between chapters creates cognitive load, reduces perceived quality, and breaks user trust. Users should feel they are navigating a unified textbook, not disconnected modules.

**NON-NEGOTIABLE RULES**:
- Use identical UI components (PersonalizeButton, TranslateButton, ChatBot, etc.)
- Match Chapter 1's layout, typography, color scheme, and spacing
- Maintain identical interaction patterns (button positions, modal behaviors, navigation flows)
- Preserve authentication and user profile integration across chapters
- Ensure identical performance characteristics (load times, response times)
- Use same error handling and messaging patterns

### III. Component Reusability
**PRINCIPLE**: All interactive components developed for Chapter 1 MUST be reused in Chapter 2 without modification to core component code.

**RATIONALE**: Component reusability was a foundational design goal from Chapter 1. Rewriting or significantly modifying components for Chapter 2 indicates architectural failure and creates maintenance burden.

**NON-NEGOTIABLE RULES**:
- PersonalizeButton component MUST work with Chapter 2 content via props/config only
- TranslateButton component MUST translate Chapter 2 content without code changes
- RAG chatbot MUST support Chapter 2 via chapter-specific vector stores (no core logic changes)
- Authentication system MUST operate identically across all chapters
- User profile system MUST apply personalization to any chapter content
- Progress tracking components MUST extend to multi-chapter state without breaking existing functionality

### IV. Chapter Independence with Navigation
**PRINCIPLE**: Each chapter MUST deliver complete educational value independently while providing seamless navigation to other chapters.

**RATIONALE**: Users may not progress linearly through the textbook. Each chapter must stand alone pedagogically, but cross-chapter navigation enables flexible learning paths and demonstrates architectural maturity.

**NON-NEGOTIABLE RULES**:
- No broken navigation links (all chapter links MUST point to implemented chapters)
- Users MUST be able to start learning from any implemented chapter
- Chapter content MUST NOT assume knowledge from unimplemented chapters
- Learning outcomes for each chapter MUST be achievable with that chapter's content alone
- RAG chatbot MUST answer questions using only the current chapter's content by default
- Navigation UI MUST clearly indicate which chapters are available
- Chapter switching MUST preserve user authentication and profile state

### V. Multi-Chapter Progression
**PRINCIPLE**: System MUST track user progress independently per chapter while maintaining global completion state.

**RATIONALE**: Users need visibility into their learning progress across the entire textbook. Progression tracking motivates continued learning and enables resumption of studies across sessions.

**NON-NEGOTIABLE RULES**:
- Track completion status for each chapter independently (e.g., Chapter 1: 100%, Chapter 2: 45%)
- Persist progression state in Neon Postgres linked to user accounts
- Display global progress indicator (e.g., "2 of 8 chapters completed")
- Display per-chapter progress on chapter navigation UI
- Allow users to resume from last accessed chapter
- Progression MUST survive logout/login cycles
- Progression MUST NOT break if new chapters are added

### VI. Scalability to N Chapters
**PRINCIPLE**: All architectural decisions MUST support addition of Chapter 3, 4, ..., N without refactoring existing chapter implementations.

**RATIONALE**: The textbook is designed to eventually cover 8 chapters. Hardcoding for 2 chapters creates technical debt. Proper abstraction now prevents costly rewrites later.

**NON-NEGOTIABLE RULES**:
- Chapter configuration MUST be data-driven (JSON/YAML config file, not hardcoded)
- Chapter routing MUST use dynamic paths (e.g., `/chapter/:id` not `/chapter1`, `/chapter2`)
- Vector stores MUST namespace by chapter ID (e.g., `chapter_1`, `chapter_2`, `chapter_N`)
- Database schemas MUST use chapter identifiers, not chapter-specific tables
- Navigation components MUST render from chapter metadata array
- Adding Chapter N MUST only require: (1) new content, (2) config entry, (3) vector store population
- No conditional logic based on chapter count (e.g., avoid `if chapters.length === 2`)

### VII. Bonus Features Continuity
**PRINCIPLE**: All bonus features implemented in Chapter 1 MUST work identically in Chapter 2 without feature-specific code duplication.

**RATIONALE**: Bonus features represent significant point value (200 points). Breaking bonus features in Chapter 2 or requiring per-chapter reimplementation undermines the original investment and demonstrates poor architecture.

**NON-NEGOTIABLE RULES**:
- **Personalization**: MUST adapt Chapter 2 content based on user's software/hardware background
- **Urdu Translation**: MUST translate Chapter 2 content preserving technical accuracy
- **User Authentication**: MUST maintain session state across chapter navigation
- **User Profiling**: Profile questions MUST apply to all chapter content personalization
- **RAG Chatbot**: MUST answer Chapter 2 questions using Chapter 2 vector embeddings
- All bonus features MUST be demonstrable in updated demo materials

## Chapter 3 Excellence Principles

### VIII. Climax Chapter Excellence
**PRINCIPLE**: Chapter 3 MUST be the most impressive, polished, and cutting-edge chapter, serving as the textbook's showcase of AI-powered robotics convergence.

**RATIONALE**: Chapter 3 represents the culmination of foundational knowledge (Ch1: ROS 2, Ch2: Gazebo) and demonstrates the practical application of LLMs in robotics. This is the "wow factor" chapter that differentiates the textbook from traditional robotics materials and showcases hackathon innovation.

**NON-NEGOTIABLE RULES**:
- Chapter 3 content quality MUST exceed Chapters 1 and 2 in depth, examples, and interactivity
- Every feature MUST be polished to production-grade quality (no TODOs or placeholders in user-facing content)
- Interactive demos MUST be visually stunning and functionally robust
- Code examples MUST be complete, tested, and commented with best practices
- All explanations MUST be crystal clear with visual aids, diagrams, and interactive elements
- Chapter 3 MUST be the primary demo chapter for hackathon judging (first 90 seconds)

### IX. AI-Robotics Convergence
**PRINCIPLE**: Chapter 3 MUST demonstrate practical integration of Large Language Models (LLMs) with robotic systems, showing how Generative AI transforms robot autonomy.

**RATIONALE**: The hackathon theme is "Generative AI & Robotics." Chapter 3 is where this convergence becomes tangible through LLM-powered robot control, natural language interfaces, and AI-assisted development workflows.

**NON-NEGOTIABLE RULES**:
- Chapter 3 MUST include practical examples of LLM-controlled robots (e.g., natural language motion planning)
- MUST demonstrate GPT-4/Claude integration with ROS 2 systems
- MUST show code generation for robot behaviors using LLMs
- MUST include vision-language model (VLM) integration for robot perception
- RAG chatbot MUST have advanced capabilities in Chapter 3 (multi-modal responses, code generation, debugging assistance)
- All AI integrations MUST use OpenAI Agents/ChatKit SDKs (per hackathon requirements)

### X. Capstone Project Readiness
**PRINCIPLE**: Chapter 3 MUST culminate in a capstone project that integrates all three chapters' concepts into a complete autonomous robot system.

**RATIONALE**: The capstone demonstrates mastery of the entire textbook curriculum and provides a portfolio-worthy project for learners. It also serves as the ultimate demo for hackathon judging.

**NON-NEGOTIABLE RULES**:
- Capstone MUST integrate: ROS 2 architecture (Ch1) + Gazebo simulation (Ch2) + LLM control (Ch3)
- Capstone project MUST be achievable within 2-3 hours for motivated learners
- Step-by-step instructions MUST be clear, tested, and error-free
- Capstone MUST produce a visually impressive result (simulation + real-world if possible)
- Capstone code MUST be provided in GitHub repository with full documentation
- Capstone MUST be showcased in the 90-second demo video

### XI. Demo Showcase Optimization
**PRINCIPLE**: Chapter 3 content and capstone MUST be optimized for a 90-second hackathon demo that maximizes judge impact.

**RATIONALE**: Hackathon judging is time-constrained. The first 90 seconds must immediately demonstrate innovation, technical excellence, and practical value. Chapter 3 is the hero chapter for this demo.

**NON-NEGOTIABLE RULES**:
- Demo script MUST be rehearsed and timed to ≤90 seconds
- Demo MUST show: (1) LLM-robot interaction, (2) Gazebo simulation, (3) RAG chatbot answering robotics questions
- Demo MUST highlight all bonus features (personalization, translation, auth, chatbot)
- Demo MUST be visually polished (professional UI, smooth transitions, no loading delays)
- Demo video MUST be recorded in high quality (1080p minimum, clear audio)
- Demo MUST emphasize uniqueness: "AI-powered robotics textbook with adaptive learning"

### XII. Future-Proof Architecture
**PRINCIPLE**: Chapter 3 implementation MUST be architected to support advanced RAG features, multi-modal AI, and extensibility for future chapters.

**RATIONALE**: As AI capabilities evolve (GPT-5, multi-modal models, robotics foundation models), the textbook architecture must accommodate upgrades without major rewrites. Chapter 3 sets the standard for AI integration patterns.

**NON-NEGOTIABLE RULES**:
- RAG architecture MUST support multi-modal embeddings (text + images + code)
- AI service layer MUST be abstracted (easy to swap OpenAI → Claude → future models)
- Chapter 3 vector store MUST support hybrid search (dense + sparse)
- Code generation features MUST be modular and testable
- All AI integrations MUST have fallback mechanisms (graceful degradation if API fails)
- Architecture MUST support future chapters adding new AI capabilities without modifying Chapter 3

## RAG Chatbot Backend Principles

### XIII. RAG Backend Architecture
**PRINCIPLE**: The RAG chatbot backend MUST be built with FastAPI, following clean architecture patterns with clear separation between embedding generation, vector storage, retrieval, and response generation layers.

**RATIONALE**: A well-architected RAG backend ensures maintainability, testability, and scalability. Separation of concerns allows independent optimization of each pipeline stage and easier debugging when issues arise.

**NON-NEGOTIABLE RULES**:
- Backend MUST use FastAPI framework (per hackathon requirements)
- Architecture MUST separate: (1) Embedding Service, (2) Vector Storage Service, (3) Retrieval Service, (4) Response Generation Service
- All services MUST have defined interfaces (dependency injection pattern)
- Configuration MUST be externalized (environment variables, config files)
- API endpoints MUST follow REST principles (proper HTTP verbs, status codes)
- All endpoints MUST have request/response validation using Pydantic models
- Backend MUST be stateless (no session state stored in backend process)
- Error handling MUST be centralized (consistent error response format)

### XIV. Embedding Pipeline Excellence
**PRINCIPLE**: Text-to-embedding conversion MUST be robust, efficient, and produce high-quality vector representations that enable accurate semantic search.

**RATIONALE**: Embedding quality directly determines RAG chatbot accuracy. Poor embeddings lead to irrelevant context retrieval and incorrect answers. Efficient processing ensures scalability as book content grows.

**NON-NEGOTIABLE RULES**:
- Embeddings MUST use OpenAI's text-embedding-3-small or text-embedding-3-large models
- Text chunking MUST preserve semantic coherence (avoid splitting mid-sentence or mid-concept)
- Chunk size MUST be optimized for context window (recommended: 500-1000 tokens with 100-200 token overlap)
- Metadata MUST be preserved with each chunk (chapter, section, page, heading hierarchy)
- Batch processing MUST be implemented for bulk embedding generation (process multiple chunks per API call)
- Embedding generation MUST be idempotent (re-running on same content produces same results)
- Failed embedding requests MUST be retried with exponential backoff (max 3 retries)
- Embedding service MUST log statistics (chunks processed, tokens consumed, API latency)

### XV. Vector Storage Strategy
**PRINCIPLE**: Vector embeddings MUST be stored in Qdrant Cloud Free Tier with proper indexing, namespacing, and metadata to enable fast, accurate retrieval.

**RATIONALE**: Qdrant provides efficient vector similarity search at scale. Proper storage design ensures sub-second query response times and accurate context retrieval even as content volume grows.

**NON-NEGOTIABLE RULES**:
- Vector database MUST use Qdrant Cloud Free Tier (per hackathon requirements)
- Collections MUST be namespaced by chapter (e.g., `chapter_1_vectors`, `chapter_2_vectors`)
- Each vector entry MUST include payload metadata: `{chapter_id, section_id, chunk_id, text, heading, page}`
- Index type MUST be optimized for search speed (HNSW recommended for <1M vectors)
- Qdrant client MUST implement connection pooling and timeout limits (≤5s per query)
- Vector upsert operations MUST be batched (100-500 vectors per batch)
- Duplicate detection MUST be implemented (prevent re-indexing same content)
- Collections MUST support filtering by metadata (enable chapter-specific searches)

### XVI. Context Retrieval Precision
**PRINCIPLE**: Retrieval MUST find the most semantically relevant book sections for a given query, with configurable precision-recall tradeoffs and support for hybrid search strategies.

**RATIONALE**: Context quality determines answer accuracy. Retrieving too few chunks risks missing critical information; too many chunks add noise and cost. Hybrid search (dense + keyword) improves robustness.

**NON-NEGOTIABLE RULES**:
- Default retrieval MUST return top-K most similar chunks (K=3-5 recommended)
- Similarity threshold MUST be configurable (default: 0.7 cosine similarity)
- Query embedding MUST use same model as document embeddings (consistency)
- Retrieval MUST support chapter-scoped queries (filter by chapter_id in metadata)
- Re-ranking MUST be applied to initial results (use cross-encoder or MMR for diversity)
- Retrieved chunks MUST be sorted by relevance score (highest first)
- Context length MUST not exceed OpenAI model limit (subtract prompt overhead from max tokens)
- Fallback retrieval MUST trigger if no results meet similarity threshold (return top-K regardless)

### XVII. Response Generation Quality
**PRINCIPLE**: Final responses MUST be generated by sending retrieved context to OpenAI's GPT models with carefully engineered prompts that ensure accurate, grounded, and helpful answers.

**RATIONALE**: The response generation stage synthesizes retrieved context into user-facing answers. Prompt quality, context formatting, and model parameters critically affect answer accuracy and user satisfaction.

**NON-NEGOTIABLE RULES**:
- Response generation MUST use OpenAI ChatCompletion API (GPT-4 or GPT-3.5-turbo)
- System prompt MUST instruct model to answer ONLY using provided context (prevent hallucination)
- System prompt MUST specify: "If context doesn't contain answer, say 'I couldn't find that information in the book.'"
- Retrieved chunks MUST be formatted clearly in user message (e.g., "Context:\n\n[chunk1]\n\n[chunk2]")
- Temperature MUST be low for factual queries (0.0-0.3 recommended)
- Max tokens MUST be set appropriately (150-300 for concise answers)
- Streaming responses MUST be supported (enable progressive display in frontend)
- Citations MUST be included (reference chapter/section of source chunks)
- API errors MUST be handled gracefully (return friendly error message, log technical details)

### XVIII. Database Management
**PRINCIPLE**: Neon Serverless Postgres MUST be used for storing chat history, user sessions, usage analytics, and metadata that requires relational queries and ACID guarantees.

**RATIONALE**: While Qdrant handles vector storage, Postgres provides structured storage for relational data. Neon's serverless model eliminates operational overhead while providing PostgreSQL compatibility.

**NON-NEGOTIABLE RULES**:
- Database MUST use Neon Serverless Postgres (per hackathon requirements)
- Schema MUST include tables: `users`, `chat_sessions`, `chat_messages`, `usage_logs`, `feedback`
- All timestamps MUST use UTC timezone (avoid timezone confusion)
- User identifiers MUST be indexed (fast lookups by user_id)
- Chat history MUST link to user sessions (enable conversation continuity)
- Database credentials MUST be stored in environment variables (never hardcoded)
- Connection pooling MUST be configured (max connections appropriate for free tier limits)
- Database migrations MUST be versioned and reversible (use Alembic or similar)
- Sensitive user data MUST be encrypted at rest (use Neon's built-in encryption)

### XIX. Selected Text Query Support
**PRINCIPLE**: The RAG chatbot MUST support answering questions specifically about user-selected text, enabling focused queries on specific book sections without full-chapter retrieval.

**RATIONALE**: Users may want to ask questions about specific paragraphs or code snippets they're currently reading. This feature improves precision and reduces irrelevant context in responses.

**NON-NEGOTIABLE RULES**:
- API MUST accept optional `selected_text` parameter in query request
- When `selected_text` is provided, it MUST be embedded and used for retrieval (supplement or replace general query)
- Selected text MUST be treated as high-priority context (ranked above general retrieval results)
- If selected text is short (<100 chars), expand context by retrieving surrounding chunks
- Selected text source MUST be validated (ensure it exists in vector database to prevent injection)
- Query response MUST indicate if answer is based on selected text vs. general retrieval
- Frontend MUST send selected text position metadata (chapter, section) for context enrichment
- Selected text feature MUST work seamlessly with chapter-scoped queries

## Technical Standards

### Technology Stack (NON-NEGOTIABLE)
**Frontend**:
- Docusaurus (required per hackathon spec)
- React components for interactive features
- TypeScript for type safety

**Backend**:
- FastAPI (required per hackathon spec)
- Python 3.11+

**Database**:
- Neon Serverless Postgres (required per hackathon spec)
- Qdrant Cloud Free Tier for vector storage (required per hackathon spec)

**Authentication**:
- better-auth.com (required for 50 bonus points)

**AI/ML**:
- OpenAI Agents/ChatKit SDKs (required per hackathon spec)
- OpenAI GPT-4 for RAG responses
- OpenAI embeddings for vector storage
- OpenAI Vision API for multi-modal features (Chapter 3)

**Translation**:
- OpenAI GPT-4 for Urdu translation (contextual, preserves technical terms)

**Deployment**:
- GitHub Pages or Vercel (required per hackathon spec)
- Public GitHub repository (required for submission)

### RAG Backend Technology Stack (NON-NEGOTIABLE)

**Backend Framework**:
- FastAPI 0.104+ (required per hackathon spec)
- Python 3.11+ (async/await support, modern type hints)
- Uvicorn (ASGI server for production)

**AI/ML Services**:
- OpenAI API (Agents/ChatKit SDKs per hackathon requirements)
- OpenAI GPT-4 or GPT-3.5-turbo (response generation)
- OpenAI text-embedding-3-small or text-embedding-3-large (embeddings)

**Vector Database**:
- Qdrant Cloud Free Tier (required per hackathon spec)
- qdrant-client Python SDK

**Relational Database**:
- Neon Serverless Postgres (required per hackathon spec)
- asyncpg (async PostgreSQL adapter)
- SQLAlchemy 2.0+ (ORM with async support)
- Alembic (database migrations)

**Backend Dependencies**:
- Pydantic 2.0+ (request/response validation)
- python-dotenv (environment variable management)
- httpx (async HTTP client for external APIs)
- tenacity (retry logic with exponential backoff)
- loguru (structured logging)

**Testing & Development**:
- pytest (unit/integration tests)
- pytest-asyncio (async test support)
- httpx (API testing)
- faker (test data generation)

### RAG API Standards

**Required Endpoints**:

1. **POST /api/query** - Main RAG query endpoint
   - Request: `{query: str, chapter_id?: int, selected_text?: str, user_id?: str}`
   - Response: `{answer: str, sources: [{chapter, section, page}], confidence: float}`
   - Timeout: 10s max
   - Streaming: Support SSE (Server-Sent Events) for progressive responses

2. **POST /api/embed** - Generate embeddings for content (admin/setup)
   - Request: `{texts: List[str], chapter_id: int, metadata: Dict}`
   - Response: `{status: str, chunks_processed: int, vectors_stored: int}`
   - Authentication: Admin API key required

3. **GET /api/health** - Health check endpoint
   - Response: `{status: str, qdrant: bool, neon: bool, openai: bool}`
   - Timeout: 2s max

4. **POST /api/feedback** - User feedback on responses
   - Request: `{query_id: str, rating: int, feedback?: str}`
   - Response: `{status: str}`

**API Design Principles**:
- All endpoints MUST return proper HTTP status codes (200, 400, 422, 500, 503)
- Error responses MUST follow format: `{error: str, detail?: str, request_id: str}`
- All requests MUST include request_id for tracing
- Rate limiting MUST be implemented (100 req/min per user)
- CORS MUST be configured for frontend domain(s)
- API versioning MUST use URL prefix (e.g., `/api/v1/query`)
- Request validation errors MUST return 422 with field-level details
- All timestamps MUST be ISO 8601 format (UTC)

**Performance Requirements for RAG Backend**:
- Query endpoint response time: <3s (p95)
- Embedding generation: <5s per 1000 tokens
- Qdrant vector search: <500ms
- Database queries: <100ms
- Concurrent requests: Support 50 concurrent users
- Memory usage: <512MB per worker process

**Security Requirements for RAG Backend**:
- OpenAI API keys MUST be stored in environment variables
- Qdrant API keys MUST be stored in environment variables
- Neon database credentials MUST be stored in environment variables
- Input sanitization MUST prevent injection attacks
- Rate limiting MUST prevent abuse
- Admin endpoints MUST require authentication
- HTTPS MUST be enforced in production
- Secrets MUST NEVER be logged or exposed in error messages

### Chapter 3 Specific Standards
**AI Integration Quality**:
- LLM responses MUST be deterministic where possible (temperature=0 for code generation)
- Prompt engineering MUST follow best practices (system prompts, few-shot examples)
- All AI calls MUST have timeout limits (≤10 seconds)
- Error messages MUST be user-friendly (no raw API errors exposed)

**Code Quality for Chapter 3**:
- All robot control code MUST be production-ready (type-hinted, documented, tested)
- LLM-generated code examples MUST be validated (no hallucinated APIs)
- Code examples MUST run without modification (copy-paste-run principle)
- All dependencies MUST be version-pinned (requirements.txt with exact versions)

**Performance for Chapter 3**:
- RAG chatbot MUST support streaming responses (progressive display)
- Multi-modal RAG MUST respond within 5 seconds (text + image queries)
- Code generation MUST complete within 8 seconds (or show progress indicator)
- Capstone project simulation MUST run at real-time speeds (≥20 FPS in Gazebo)

### Multi-Chapter Quality Gates (Updated for Chapter 3)
**Pre-Deployment Checklist for Chapter 3**:
- ✅ All bonus features functional in Chapter 3
- ✅ RAG chatbot answers Chapter 3 questions accurately (LLM + robotics queries)
- ✅ RAG chatbot supports code generation and debugging assistance
- ✅ Capstone project runs successfully end-to-end
- ✅ LLM-robot integration demo works flawlessly
- ✅ Multi-modal RAG features functional (if implemented)
- ✅ 90-second demo video recorded and polished
- ✅ All code examples tested and validated
- ✅ Visual consistency with Chapters 1 and 2 maintained
- ✅ No console errors in browser DevTools
- ✅ Mobile responsive (all three chapters)
- ✅ GitHub repository complete with README, setup instructions, and capstone guide

### Performance Requirements
- Page load time: <3 seconds (GitHub Pages/Vercel CDN)
- RAG chatbot response time: <3 seconds (text queries), <5 seconds (multi-modal)
- Translation response time: <5 seconds (acceptable for demo, can show loading state)
- Personalization response time: <2 seconds
- Chapter switching: <1 second (client-side navigation preferred)
- Code generation: <8 seconds (show streaming progress)

### Security Requirements
- User passwords MUST be hashed (better-auth.com handles this)
- API keys MUST be stored in environment variables (never committed to git)
- Database credentials MUST be stored in environment variables
- CORS MUST be properly configured for frontend-backend communication
- Chapter access control (if implementing premium chapters) MUST be enforced server-side
- LLM-generated code MUST be sandboxed before execution (if code execution feature added)

## Chapter 3 Content Requirements

### Required Topics (AI-Robotics Convergence)
1. **LLM-Powered Robot Control**
   - Natural language to robot motion planning
   - GPT-4 integration with ROS 2 action servers
   - Safety constraints and validation

2. **Vision-Language Models for Robotics**
   - Scene understanding with GPT-4 Vision
   - Object detection and manipulation planning
   - Visual question answering for robot perception

3. **AI-Assisted Robot Development**
   - Code generation for ROS 2 nodes
   - Debugging assistance with LLMs
   - Automated URDF generation from natural language

4. **Capstone Project: Autonomous LLM-Controlled Robot**
   - End-to-end system integration
   - Natural language task planning
   - Simulation and real-world deployment guide

### Required Interactive Features (Chapter 3)
- **Live Code Editor with AI Assistance**: Users can generate robot code via natural language prompts
- **Interactive LLM-Robot Playground**: Test natural language commands in live Gazebo simulation
- **Multi-Modal RAG Chatbot**: Answer questions using text, images, and code examples
- **Capstone Project Builder**: Step-by-step wizard guiding users through final project

### Required Examples (Minimum 5)
1. Natural language motion planning ("Move forward 2 meters and turn left")
2. Vision-based object manipulation ("Pick up the red cube")
3. LLM-generated ROS 2 publisher node
4. GPT-4 Vision scene understanding integration
5. Complete capstone project: Autonomous delivery robot with LLM planning

### Required Exercises
1. Build an LLM-controlled robot in Gazebo
2. Implement vision-language model for object detection
3. Generate and test ROS 2 code using GPT-4
4. **Capstone**: Complete autonomous robot system with LLM brain

## Development Workflow for Chapter 3

### Phase 1: Architecture & Planning (Days 1-2)
1. Design AI integration architecture (LLM service layer, prompt management)
2. Plan capstone project requirements and structure
3. Design interactive LLM-robot playground
4. Create demo script (90-second outline)
5. Set up multi-modal RAG infrastructure

### Phase 2: Core AI Integration (Days 3-5)
1. Implement LLM-ROS 2 bridge (natural language → robot commands)
2. Integrate GPT-4 Vision for robot perception
3. Build code generation feature (natural language → Python/ROS 2 code)
4. Enhance RAG chatbot with advanced features
5. Implement AI-assisted debugging

### Phase 3: Content Creation (Days 6-8)
1. Write Chapter 3 content (matching Chapters 1-2 quality)
2. Create 5+ interactive code examples
3. Build capstone project step-by-step guide
4. Develop interactive LLM-robot playground
5. Populate Chapter 3 vector embeddings

### Phase 4: Capstone & Demo (Days 9-10)
1. Build complete capstone project implementation
2. Test end-to-end capstone user journey
3. Record 90-second demo video
4. Polish all interactive features
5. Validate all code examples

### Phase 5: Integration & Testing (Days 11-12)
1. Ensure visual consistency across all 3 chapters
2. Test multi-chapter navigation and progression
3. Validate all bonus features work in Chapter 3
4. Performance optimization
5. Final quality gate validation

### Phase 6: Documentation & Deployment (Day 13)
1. Update README with Chapter 3 setup
2. Document capstone project thoroughly
3. Create deployment checklist
4. Final demo rehearsal
5. Deploy to production

### Testing Strategy for Chapter 3
**Critical Tests**:
- LLM-robot integration (10 natural language commands)
- Code generation accuracy (5 generated nodes must compile and run)
- Capstone project end-to-end (complete walkthrough)
- RAG chatbot accuracy (20 Chapter 3 questions, including multi-modal)
- Demo video validation (≤90 seconds, all features shown)

**Performance Tests**:
- Load testing (100 concurrent RAG requests)
- Latency testing (all AI features <10s)
- Simulation performance (capstone project ≥20 FPS)

## Governance

### Constitution Authority
This constitution supersedes all other development practices for the Physical AI Textbook project. Any deviation MUST be documented with justification and approved before implementation.

### Amendment Process
**MINOR version bump** (1.x.0): New principle added, section expanded
**PATCH version bump** (1.0.x): Clarifications, wording improvements
**MAJOR version bump** (x.0.0): Principle removal or fundamental redefinition

All amendments MUST:
1. Document rationale in Sync Impact Report
2. Update dependent templates (.specify/templates/*)
3. Validate against existing work
4. Commit with descriptive message

### Compliance Review
**Pre-commit validation**:
- No API keys or credentials committed
- No TODO/FIXME in user-facing content
- All bonus features functional in all chapters

**Pre-deployment validation**:
- All Quality Gates passed (including Chapter 3 gates)
- Demo materials reflect current capabilities (90-second video)
- README complete with multi-chapter setup instructions
- Visual consistency validated across all chapters
- Capstone project tested end-to-end

### Complexity Justification
Any architectural decision introducing complexity (additional services, libraries, frameworks) MUST answer:
1. Does this enable Chapter 3's AI-robotics convergence goals?
2. Does this maintain component reusability across chapters?
3. Is there a simpler alternative that achieves the same goal?
4. Does this support the 90-second demo showcase?

If answers are No/No/Yes/No, the complexity is REJECTED.

### Runtime Development Guidance
For agent-specific development guidance beyond this constitution, refer to `CLAUDE.md` for Claude Code-specific workflows and best practices.

**Version**: 1.3.0 | **Ratified**: 2025-12-03 | **Last Amended**: 2025-12-08
