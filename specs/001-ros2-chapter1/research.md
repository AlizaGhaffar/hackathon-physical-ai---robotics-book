# Research & Technology Decisions: ROS 2 Chapter 1

**Date**: 2025-12-03
**Purpose**: Resolve technical unknowns and establish best practices for implementation

---

## Decision 1: Docusaurus vs. Custom React App for Frontend

**Context**: Need a frontend framework that supports educational content (markdown), interactivity (React components), and fast deployment (GitHub Pages/Vercel).

**Decision**: **Docusaurus 3.x**

**Rationale**:
- **Built for documentation**: Docusaurus is designed for technical documentation with excellent markdown support, syntax highlighting, and versioning
- **React integration**: Supports embedding React components directly in markdown (MDX), enabling interactive diagrams, code editors, and quizzes without custom build setup
- **Zero-config deployment**: One-command deployment to GitHub Pages (`npm run deploy`)
- **SEO and performance**: Built-in optimizations for static site generation (SSG), code splitting, and lazy loading
- **Mobile responsive**: Default themes are mobile-friendly out of the box

**Alternatives Considered**:
1. **Next.js**: More flexible but requires custom markdown processing, more complex deployment setup
2. **Gatsby**: Similar to Docusaurus but steeper learning curve, slower build times
3. **Plain React**: Maximum flexibility but would need to build markdown rendering, routing, and deployment pipeline from scratch

**Best Practices**:
- Use MDX for content files to embed React components
- Leverage Docusaurus plugins for syntax highlighting (Prism)
- Keep custom components in `src/components/` for reusability
- Use Docusaurus themes for consistent styling

---

## Decision 2: FastAPI vs. Flask/Django for Backend

**Context**: Need a Python backend framework for AI services (OpenAI integration), database operations, and RESTful APIs.

**Decision**: **FastAPI 0.104+**

**Rationale**:
- **Async support**: FastAPI is built on ASGI (Starlette), enabling async/await for concurrent OpenAI API calls (chatbot, translation, embeddings)
- **Automatic API docs**: Built-in OpenAPI (Swagger) and ReDoc documentation generation - helps with contract validation
- **Type safety**: Pydantic models for request/response validation reduce bugs
- **Performance**: Faster than Flask/Django for I/O-bound operations (API calls to OpenAI, Qdrant, Neon)
- **Modern Python**: Leverages Python 3.11+ features (type hints, async)

**Alternatives Considered**:
1. **Flask**: Simpler but lacks async support, requires manual API documentation
2. **Django**: Full-featured but overkill for API-only backend, slower for async operations
3. **Express.js (Node)**: Would require switching languages, less robust for AI/ML libraries

**Best Practices**:
- Use async route handlers for all OpenAI API calls
- Define Pydantic models for all request/response schemas
- Use dependency injection for database connections and API clients
- Enable CORS middleware for frontend-backend communication

---

## Decision 3: Neon Serverless Postgres vs. Supabase/PlanetScale

**Context**: Need a Postgres database for user data, progress tracking, and chat history. Must have free tier and serverless architecture.

**Decision**: **Neon Serverless Postgres**

**Rationale**:
- **Hackathon requirement**: Explicitly specified in hackathon rules
- **Serverless**: Auto-scales to zero when idle (cost-effective for demo)
- **Postgres compatibility**: Full Postgres feature set (JSON columns, indexes, transactions)
- **Free tier**: Generous limits (3GB storage, 100 hours compute/month)
- **Fast cold starts**: <1 second wake-up time from idle

**Alternatives Considered**:
1. **Supabase**: Similar features but adds unnecessary complexity (built-in auth, realtime - not needed since we use better-auth)
2. **PlanetScale**: MySQL-based, would require rewriting SQL queries for MySQL compatibility
3. **SQLite**: Simple but lacks concurrent writes, poor fit for multi-user web app

**Best Practices**:
- Use psycopg2-binary (sync) or asyncpg (async) for Python connections
- Create indexes on frequently queried fields (user_id, chapter_id)
- Use connection pooling to reduce latency
- Store environment connection string in .env file

---

## Decision 4: Qdrant Cloud vs. Pinecone/Weaviate for Vector Database

**Context**: Need vector database for storing Chapter 1 content embeddings (RAG chatbot). Must support semantic search and have free tier.

**Decision**: **Qdrant Cloud Free Tier**

**Rationale**:
- **Hackathon requirement**: Explicitly specified in hackathon rules
- **Free tier**: 1GB storage, sufficient for ~300 chunks (Chapter 1)
- **Python client**: Well-documented qdrant-client library
- **Fast similarity search**: HNSW algorithm for sub-100ms queries
- **Metadata filtering**: Can filter by chapter_id, section, etc.

**Alternatives Considered**:
1. **Pinecone**: Similar features but free tier limited to 1 index (Qdrant supports multiple collections)
2. **Weaviate**: More complex setup, requires Docker for local dev
3. **FAISS (local)**: No managed service, would need to deploy and manage ourselves

**Best Practices**:
- Use `text-embedding-ada-002` from OpenAI (1536 dimensions)
- Chunk content semantically (by section/subsection, not fixed length)
- Store metadata: `{"chapter_id": "chapter-1", "section": "Nodes", "subsection": "Creating Nodes"}`
- Use cosine similarity for search (default in Qdrant)
- Limit top-k results to 5 chunks for RAG context window

---

## Decision 5: better-auth.com vs. Auth0/Clerk for Authentication

**Context**: Need authentication system with user signup, login, session management, and profile storage. Must integrate with React and FastAPI.

**Decision**: **better-auth.com**

**Rationale**:
- **Hackathon requirement**: Worth 50 bonus points
- **Lightweight**: Minimal SDK, no heavyweight iframe or redirect flows
- **Customizable**: Can add custom fields (software_level, hardware_level) to signup
- **Session management**: Handles JWT token generation and validation
- **Free tier**: Generous limits for hackathon scale

**Alternatives Considered**:
1. **Auth0**: Industry standard but complex setup, overkill for simple use case
2. **Clerk**: Good UX but limited customization for background questionnaire
3. **Custom JWT auth**: Would work but reinventing the wheel, more bugs

**Best Practices**:
- Store JWT tokens in localStorage (not cookies for simplicity)
- Use better-auth middleware in FastAPI to validate tokens
- Add background questionnaire fields to signup flow
- Implement session persistence (7-day token expiry)

---

## Decision 6: OpenAI GPT-4 vs. GPT-3.5-turbo for Translation & Chatbot

**Context**: Need LLM for Urdu translation (preserving technical terms) and RAG chatbot responses.

**Decision**: **GPT-4 for translation, GPT-3.5-turbo for chatbot (cost optimization)**

**Rationale**:
- **Translation (GPT-4)**: Better at instruction-following ("preserve ROS 2, Node, Topic in English"), higher quality Urdu output
- **Chatbot (GPT-3.5-turbo)**: Sufficient for factual Q&A with retrieved context, 10x cheaper than GPT-4
- **Rate limits**: GPT-3.5-turbo has higher rate limits (better for demo with multiple concurrent users)

**Alternatives Considered**:
1. **GPT-4 for both**: Best quality but expensive and slower
2. **GPT-3.5-turbo for both**: Cheaper but translation quality suffers
3. **Google Translate API for translation**: Fast but cannot preserve technical terms contextually

**Best Practices**:
- **Translation system prompt**: "You are a technical translator. Translate the following text to Urdu. PRESERVE these terms in English: ROS 2, Node, Topic, Service, Publisher, Subscriber, rclpy. Provide natural Urdu for all other text."
- **Chatbot system prompt**: "You are a helpful ROS 2 tutor. Answer questions based ONLY on the provided context from Chapter 1. If the answer is not in the context, say 'I can only answer questions about Chapter 1 content.'"
- **Temperature**: 0.3 for translation (more deterministic), 0.7 for chatbot (more conversational)
- **Max tokens**: 1000 for translation, 500 for chatbot responses

---

## Decision 7: Monaco Editor vs. CodeMirror for Live Code Editor

**Context**: Need an embeddable code editor for Python examples with syntax highlighting and "Run Code" button.

**Decision**: **Monaco Editor (@monaco-editor/react)**

**Rationale**:
- **VS Code engine**: Same editor as VS Code, excellent Python syntax highlighting
- **React integration**: Official React wrapper, easy to embed
- **Themes**: Built-in themes (VS Dark, VS Light)
- **Lightweight**: Can lazy-load editor on demand

**Alternatives Considered**:
1. **CodeMirror**: Lighter weight but requires more manual theme/language configuration
2. **Ace Editor**: Older, less active development
3. **Prism (syntax highlighting only)**: No editing capability

**Best Practices**:
- Lazy load Monaco to reduce initial page load
- Use read-only mode for examples (not editable by default)
- "Run Code" button shows pre-computed expected output (not actual execution for security)

---

## Decision 8: @react-three/fiber vs. Babylon.js for 3D Robot Viewer

**Context**: Need 3D viewer to display robot URDF models in browser.

**Decision**: **@react-three/fiber (Three.js wrapper for React)**

**Rationale**:
- **React integration**: Declarative 3D with React components
- **Lightweight**: Smaller bundle than Babylon.js
- **URDF support**: Community libraries for loading URDF files
- **Active ecosystem**: Large community, many examples

**Alternatives Considered**:
1. **Babylon.js**: More features but heavier, less React-friendly
2. **Plain Three.js**: More control but requires imperative setup
3. **Sketchfab embed**: Easy but limited customization

**Best Practices**:
- Use `@react-three/drei` for camera controls (OrbitControls)
- Load URDF models as GLB/GLTF (convert from URDF using tools)
- Add lighting and shadows for better visualization

---

## Decision 9: Deployment Platform for Backend

**Context**: Need cloud platform to host FastAPI backend. Must have free tier and support Python.

**Decision**: **Railway.app** (primary), **Render.com** (fallback)

**Rationale**:
- **Railway**: $5 free credit/month, easy GitHub integration, auto-deploy on push
- **Render**: Free tier available (sleeps after 15 min inactivity), good fallback option
- **No Docker required**: Both support Python buildpacks (detect requirements.txt)

**Alternatives Considered**:
1. **Heroku**: Deprecated free tier (now paid only)
2. **Fly.io**: Requires Dockerfile, more complex setup
3. **AWS Lambda**: Serverless but cold start issues for FastAPI

**Best Practices**:
- Use Procfile or railway.toml for start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
- Set environment variables in platform UI (OPENAI_API_KEY, NEON_DATABASE_URL, etc.)
- Enable auto-deploy from GitHub branch

---

## Decision 10: Content Chunking Strategy for RAG

**Context**: Need to split Chapter 1 markdown into chunks for vector embeddings. Chunks must be semantically meaningful for accurate retrieval.

**Decision**: **Semantic chunking by section/subsection** (not fixed character length)

**Rationale**:
- **Contextual coherence**: Section-based chunks maintain topic coherence (e.g., entire "What is ROS 2?" section in one chunk)
- **Better retrieval**: User asking "What is a Node?" retrieves the "Nodes" section, not random 500-character fragments
- **Metadata-rich**: Each chunk tagged with `{"section": "Core Concepts", "subsection": "Nodes"}`

**Alternatives Considered**:
1. **Fixed 500-character chunks**: Simple but breaks mid-sentence, loses context
2. **Sentence-level chunks**: Too granular, retrieves disconnected sentences
3. **Entire chapter as one chunk**: Exceeds token limits, poor precision

**Best Practices**:
- Parse markdown by heading levels (## = section, ### = subsection)
- Limit chunk size to ~300-500 tokens (to fit in GPT context with other chunks)
- If section exceeds 500 tokens, split at paragraph boundaries
- Store chunk metadata: `{"chapter_id": "chapter-1", "section": "...", "subsection": "...", "chunk_index": 0}`

---

## Summary of Key Technologies

| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| Frontend Framework | Docusaurus | 3.x | Educational content + React components |
| Backend Framework | FastAPI | 0.104+ | AI services API |
| Primary Database | Neon Postgres | Serverless | User/progress/chat data |
| Vector Database | Qdrant Cloud | Free tier | Content embeddings for RAG |
| Authentication | better-auth.com | Latest | Signup/login with profiles |
| LLM (Translation) | OpenAI GPT-4 | Latest | Urdu translation |
| LLM (Chatbot) | OpenAI GPT-3.5-turbo | Latest | RAG responses |
| Embeddings | text-embedding-ada-002 | Latest | Vector search |
| Code Editor | Monaco Editor | Latest | Interactive Python examples |
| 3D Viewer | @react-three/fiber | Latest | URDF robot models |
| Backend Deployment | Railway.app | N/A | FastAPI hosting |
| Frontend Deployment | GitHub Pages / Vercel | N/A | Static site hosting |

---

## Remaining Unknowns (to be resolved during implementation)

1. **Better-auth SDK specifics**: Need to review docs for exact API endpoints and token format
2. **URDF to GLB conversion**: May need external tool (urdf2gltf) if @react-three/fiber doesn't load URDF natively
3. **Qdrant collection setup**: Exact API calls for creating collection and setting distance metric
4. **Docusaurus MDX syntax**: How to embed React components with props in markdown

**Resolution Strategy**: Consult official documentation during implementation (Day 1-2), allocate buffer time for debugging.

---

**Research Complete**: All major technology decisions finalized. Ready to proceed to Phase 1 (Data Model & API Contracts).
