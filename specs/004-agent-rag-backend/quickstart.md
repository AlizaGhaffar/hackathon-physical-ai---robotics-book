# Quickstart Guide: Agent-Based RAG Backend

**Feature**: 004-agent-rag-backend
**Estimated Setup Time**: 10 minutes
**Prerequisites**: Python 3.11+, API keys (OpenAI, Cohere, Qdrant)

## Overview

This guide walks you through setting up and running the Agent-Based RAG Backend locally. The backend provides a FastAPI server that answers questions about the Physical AI textbook using retrieval-augmented generation (RAG).

## Prerequisites

1. **Python 3.11 or higher**
   ```bash
   python --version  # Should show 3.11.x or higher
   ```

2. **Required API Keys**:
   - OpenAI API key (for GPT-4 and Agents SDK)
   - Cohere API key (for query embeddings)
   - Qdrant Cloud credentials (URL + API key)

3. **Qdrant Collection Pre-populated**:
   - The `rag_embedding` collection must exist in Qdrant (created by feature 002-embedding-pipeline)
   - Collection must contain embedded book chunks with metadata

## Step 1: Clone and Navigate

```bash
cd backend/agent-rag
```

## Step 2: Create Virtual Environment

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate
```

## Step 3: Install Dependencies

```bash
pip install --upgrade pip
pip install -r requirements.txt
```

**Expected dependencies**:
- fastapi>=0.104.0
- uvicorn[standard]>=0.24.0
- openai>=1.10.0
- cohere>=4.37
- qdrant-client>=1.7.0
- pydantic>=2.5.0
- python-dotenv>=1.0.0

## Step 4: Configure Environment Variables

Create a `.env` file in `backend/agent-rag/`:

```bash
cp .env.example .env
```

Edit `.env` and add your credentials:

```ini
# OpenAI Configuration
OPENAI_API_KEY=sk-...your-key-here...
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_TEMPERATURE=0.2

# Cohere Configuration
COHERE_API_KEY=...your-key-here...
COHERE_MODEL=embed-english-v3.0

# Qdrant Configuration
QDRANT_URL=https://...your-cluster....qdrant.io
QDRANT_API_KEY=...your-key-here...
QDRANT_COLLECTION=rag_embedding

# RAG Configuration
TOP_K_CHUNKS=5
SIMILARITY_THRESHOLD=0.7
SESSION_TIMEOUT_MINUTES=30

# Server Configuration
HOST=0.0.0.0
PORT=8000
LOG_LEVEL=INFO
```

## Step 5: Verify Qdrant Collection

Ensure the Qdrant collection exists and has data:

```bash
python -c "
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()
client = QdrantClient(url=os.getenv('QDRANT_URL'), api_key=os.getenv('QDRANT_API_KEY'))

# Check collection exists
collections = client.get_collections()
print('Collections:', [c.name for c in collections.collections])

# Check collection size
collection_info = client.get_collection('rag_embedding')
print(f'Collection has {collection_info.points_count} chunks')
"
```

**Expected output**:
```
Collections: ['rag_embedding']
Collection has 5234 chunks
```

## Step 6: Run the Server

```bash
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

## Step 7: Test the API

### Test 1: Health Check

```bash
curl http://localhost:8000/health
```

**Expected response**:
```json
{
  "status": "healthy",
  "qdrant_available": true,
  "openai_available": true,
  "cohere_available": true,
  "timestamp": "2025-12-13T10:30:00Z",
  "uptime_seconds": 45
}
```

### Test 2: Simple Question

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What are the three main components of a robot?"
  }'
```

**Expected response**:
```json
{
  "answer": "The three main components of a robot are: (1) sensors for perception, (2) actuators for movement, and (3) a control system for decision-making.",
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

### Test 3: Chapter-Scoped Query

```bash
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is sensor fusion?",
    "chapter_filter": 3
  }'
```

### Test 4: Multi-Turn Conversation

```bash
# First turn (save session_id from response)
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is reinforcement learning?"
  }'

# Second turn (use session_id from above)
curl -X POST http://localhost:8000/ask \
  -H "Content-Type: application/json" \
  -d '{
    "question": "How is it used in robotics?",
    "session_id": "123e4567-e89b-12d3-a456-426614174000"
  }'
```

## Step 8: Access API Documentation

FastAPI automatically generates interactive API documentation:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

Use these for testing endpoints interactively.

## Troubleshooting

### Error: "Qdrant connection failed"

**Cause**: Invalid Qdrant credentials or network issues

**Solution**:
1. Verify `QDRANT_URL` and `QDRANT_API_KEY` in `.env`
2. Test connection: `curl -H "api-key: YOUR_KEY" https://YOUR_CLUSTER.qdrant.io/collections`
3. Check Qdrant Cloud dashboard for cluster status

### Error: "Collection 'rag_embedding' not found"

**Cause**: Embedding pipeline (feature 002) not run yet

**Solution**:
1. Navigate to `backend/embedding-pipeline/`
2. Run: `uv run python main.py`
3. Wait for ~5-10 minutes for embedding generation
4. Verify collection exists (Step 5 above)

### Error: "OpenAI API rate limit exceeded"

**Cause**: Too many requests to OpenAI API (Tier limits)

**Solution**:
1. Wait 60 seconds and retry
2. Upgrade OpenAI API tier for higher limits
3. Implement request queuing (future enhancement)

### Error: "Validation error: question required"

**Cause**: Malformed request JSON

**Solution**:
1. Ensure `question` field is present in request body
2. Check JSON syntax (use Swagger UI for validation)
3. Verify Content-Type header is `application/json`

### Warning: "Low confidence response"

**Cause**: No relevant book content found for query

**Expected Behavior**: Agent should respond "I couldn't find information about that in the book"

**Solution**:
- This is correct behavior for out-of-scope questions
- If unexpected, check query spelling or try rephrasing
- Verify Qdrant collection has relevant content

## Development Workflow

### Running Tests

```bash
# Unit tests (mocked external services)
pytest tests/unit/ -v

# Integration tests (requires API keys)
pytest tests/integration/ -v

# All tests with coverage
pytest tests/ --cov=./ --cov-report=html
```

### Code Formatting

```bash
# Format code
black .

# Check linting
ruff check .
```

### Monitoring Logs

```bash
# View logs with structured output
tail -f logs/app.log

# Filter for errors
grep ERROR logs/app.log
```

## Next Steps

1. **Frontend Integration**: Connect to Docusaurus chatbot component (see `src/components/ChatBot.tsx`)
2. **Load Testing**: Benchmark with 50 concurrent requests (target: <3s p95 response time)
3. **Monitoring**: Set up logging aggregation (Loguru → CloudWatch/Datadog)
4. **Production Deployment**: Deploy to Vercel/Railway with environment variables

## Production Checklist

Before deploying to production:

- [ ] All API keys stored in environment variables (not `.env` file in repo)
- [ ] `LOG_LEVEL=WARNING` or `ERROR` (reduce noise)
- [ ] CORS configured for production frontend domain
- [ ] Rate limiting enabled (100 req/min per user)
- [ ] Health check endpoint monitored by load balancer
- [ ] Error responses sanitized (no stack traces exposed)
- [ ] OpenAPI schema published for frontend team
- [ ] Load testing completed (50 concurrent users)
- [ ] Backup Qdrant credentials secured

## Additional Resources

- **OpenAPI Specification**: `specs/004-agent-rag-backend/contracts/openapi.yaml`
- **Data Model**: `specs/004-agent-rag-backend/data-model.md`
- **Architecture Plan**: `specs/004-agent-rag-backend/plan.md`
- **Feature Spec**: `specs/004-agent-rag-backend/spec.md`
- **OpenAI Assistants API Docs**: https://platform.openai.com/docs/assistants
- **Qdrant Python Client**: https://qdrant.tech/documentation/quick-start/
- **Cohere Embed API**: https://docs.cohere.com/reference/embed
