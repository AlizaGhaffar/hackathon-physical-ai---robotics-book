# Agent-Based RAG Backend (Feature 004)

**Single-file implementation** of retrieval-augmented generation using OpenAI Agents SDK pattern.

## File: `app.py`

Complete FastAPI backend implementing:
- **Cohere embeddings** for query vectorization
- **Qdrant retrieval** from `rag_embedding` collection
- **OpenAI GPT-4** for grounded answer generation
- **Citation extraction** from retrieved chunks
- **Health monitoring** for all services

## Quick Start

### 1. Install Additional Dependencies

```bash
cd backend
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install cohere==4.37
```

### 2. Configure Environment

Add to your `.env` file:

```ini
# Cohere API (for query embeddings)
COHERE_API_KEY=your-cohere-api-key-here

# Qdrant Configuration (should already exist from feature 002)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION=rag_embedding

# OpenAI Configuration
OPENAI_API_KEY=sk-your-openai-key-here
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_TEMPERATURE=0.2

# RAG Tuning
TOP_K_CHUNKS=5
SIMILARITY_THRESHOLD=0.7
```

### 3. Run the Agent-Based RAG Server

```bash
python app.py
```

Server starts at: http://localhost:8000

## API Usage

### Ask a Question

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the three main components of a robot?"
  }'
```

**Response**:
```json
{
  "answer": "The three main components of a robot are...",
  "sources": [
    {
      "chapter_id": 1,
      "section": "Introduction to Robotics",
      "page": 12,
      "chunk_id": "ch1-intro-001",
      "source_url": "https://book.example.com/chapter1/intro",
      "relevance_score": 0.92
    }
  ],
  "confidence": "high",
  "retrieval_count": 5,
  "processing_time_ms": 2340
}
```

### Chapter-Scoped Query

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is sensor fusion?",
    "chapter_filter": 3
  }'
```

### Health Check

```bash
curl http://localhost:8000/health
```

## Architecture

```
User Question
    ↓
embed_query() → Cohere embed-english-v3.0 (1024-dim)
    ↓
retrieve_chunks() → Qdrant semantic search (top-5)
    ↓
format_context() → Combine chunks with metadata
    ↓
OpenAI GPT-4 → Grounded answer generation
    ↓
AskResponse → answer + citations + confidence
```

## Key Functions

### `embed_query(text: str) -> List[float]`
- Uses Cohere `embed-english-v3.0` with `search_query` input type
- Returns 1024-dimensional embedding
- Handles errors → HTTP 503

### `retrieve_chunks(query_embedding, top_k=5, chapter_filter=None, section_filter=None)`
- Queries Qdrant collection `rag_embedding`
- Applies metadata filters if provided
- Returns chunks with scores above `SIMILARITY_THRESHOLD` (default 0.7)
- Handles errors → HTTP 503

### `format_context(chunks) -> str`
- Combines retrieved chunks into clean context block
- Includes chunk metadata (chapter, section, source URL)
- Returns "No relevant content" if empty

### `create_agent() -> dict`
- Returns agent configuration with:
  - System message: "Answer ONLY using book content"
  - Model: `gpt-4-turbo-preview`
  - Temperature: `0.2` (low for factual consistency)

### `calculate_confidence(chunks) -> str`
- "high": avg score > 0.8
- "medium": avg score 0.6-0.8
- "low": avg score < 0.6 or no chunks

## Grounding Strategy

**Multi-layered hallucination prevention**:
1. **Explicit system prompt**: "Answer ONLY using book content provided"
2. **Low temperature**: 0.2 (reduces creative variance)
3. **Deflection logic**: Returns "I couldn't find information..." when no chunks retrieved
4. **Citation requirement**: Every response includes source references
5. **Similarity threshold**: Filters out low-relevance chunks (< 0.7 score)

## Comparison with Feature 001 (RAG Chatbot Backend)

| Feature | 001-rag-chatbot-backend | 004-agent-rag-backend |
|---------|-------------------------|----------------------|
| **Structure** | Modular (src/, services/, api/) | Single-file (app.py) |
| **Embeddings** | OpenAI text-embedding-3-small | Cohere embed-english-v3.0 |
| **Agent Pattern** | Direct GPT-4 calls | OpenAI Agents SDK pattern |
| **Database** | Neon Postgres (chat history) | In-memory (MVP) |
| **Session Management** | Persistent | In-memory (30min expiry) |
| **Use Case** | Production multi-feature backend | Simplified RAG demo |

## Testing

### Test 1: Simple Question
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is reinforcement learning?"}'
```

### Test 2: Out-of-Scope (Should Deflect)
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "What is the weather today?"}'
```

Expected: `"I couldn't find information about that in the book."`

### Test 3: Chapter Filter
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "Explain sensor types", "chapter_filter": 2}'
```

### Test 4: Invalid Filter (Should Fail Validation)
```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{"question": "Test", "chapter_filter": 99}'
```

Expected: 422 Validation Error

## Troubleshooting

**"Qdrant vector database unavailable"**
- Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
- Check Qdrant Cloud dashboard for cluster status
- Ensure `rag_embedding` collection exists (run feature 002 first)

**"Cohere embedding service unavailable"**
- Verify `COHERE_API_KEY` is valid
- Check Cohere dashboard for API quota
- Ensure account has credits

**"OpenAI API error"**
- Verify `OPENAI_API_KEY` is valid
- Check OpenAI account credits
- Watch for rate limit errors (429)

**Response: "I couldn't find information..."**
- Expected for out-of-scope questions
- If unexpected, lower `SIMILARITY_THRESHOLD` (try 0.5)
- Verify Qdrant collection has relevant content

## Performance Tuning

Adjust in `.env`:

- **TOP_K_CHUNKS**: Increase for more context (e.g., 10), decrease for faster responses (e.g., 3)
- **SIMILARITY_THRESHOLD**: Lower for more permissive retrieval (e.g., 0.5), raise for stricter matching (e.g., 0.8)
- **OPENAI_TEMPERATURE**: 0.0 for most deterministic, up to 0.5 for slight variation

## API Documentation

Interactive docs at:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

## Next Steps

1. **Multi-turn conversations**: Add session management (see User Story 2 in tasks.md)
2. **Persistent storage**: Integrate Neon Postgres for chat history
3. **Rate limiting**: Add slowapi middleware
4. **Monitoring**: Integrate structured logging (JSON format)
5. **Production deployment**: Deploy to Vercel/Railway

## Related Files

- **Feature spec**: `specs/004-agent-rag-backend/spec.md`
- **Implementation plan**: `specs/004-agent-rag-backend/plan.md`
- **Task breakdown**: `specs/004-agent-rag-backend/tasks.md`
- **API contract**: `specs/004-agent-rag-backend/contracts/openapi.yaml`
