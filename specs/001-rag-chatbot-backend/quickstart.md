# Quickstart Guide: RAG Chatbot Backend

**Feature**: RAG Chatbot Backend for Physical AI Textbook
**Target Audience**: Developers setting up the backend for the first time
**Time to Complete**: ~15 minutes

---

## Prerequisites

Before starting, ensure you have:

- ✅ **Python 3.11+** installed (`python --version`)
- ✅ **pip** package manager
- ✅ **Git** for cloning the repository
- ✅ **Credentials** for:
  - Qdrant Cloud (API key, cluster URL) - [Get Free Tier](https://cloud.qdrant.io/)
  - Neon Serverless Postgres (connection string) - [Get Free Tier](https://neon.tech/)
  - OpenAI API (API key) - [Get API Key](https://platform.openai.com/api-keys)

**Note**: All credentials have already been configured in `.env` file (see [spec.md](./spec.md#credential-management--environment-setup))

---

## Quick Start (5 Minutes)

### 1. Clone and Navigate

```bash
cd /path/to/book
```

The repository already has the `.env` file configured with your credentials.

### 2. Create Virtual Environment

```bash
# Create virtual environment
python3.11 -m venv backend/venv

# Activate virtual environment
# On Linux/Mac:
source backend/venv/bin/activate
# On Windows:
backend\venv\Scripts\activate
```

### 3. Install Dependencies

```bash
pip install --upgrade pip
pip install -r backend/requirements.txt
```

**Required packages** (will be created in `backend/requirements.txt`):
```
fastapi==0.104.1
uvicorn[standard]==0.24.0
qdrant-client==1.7.0
openai==1.3.0
sqlalchemy==2.0.23
asyncpg==0.29.0
pydantic==2.5.2
pydantic-settings==2.1.0
python-dotenv==1.0.0
alembic==1.13.0
httpx==0.25.2
pytest==7.4.3
pytest-asyncio==0.21.1
```

### 4. Verify Environment Variables

```bash
# Check .env file exists
ls -la .env

# Verify required variables are set (don't print values!)
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print('✅ QDRANT_URL:', 'SET' if os.getenv('QDRANT_URL') else 'MISSING'); print('✅ DATABASE_URL:', 'SET' if os.getenv('DATABASE_URL') else 'MISSING'); print('✅ OPENAI_API_KEY:', 'SET' if os.getenv('OPENAI_API_KEY') else 'MISSING')"
```

Expected output:
```
✅ QDRANT_URL: SET
✅ DATABASE_URL: SET
✅ OPENAI_API_KEY: SET
```

### 5. Initialize Database

```bash
cd backend

# Generate database migrations
alembic revision --autogenerate -m "Initial schema"

# Apply migrations
alembic upgrade head
```

### 6. Create Qdrant Collection

```bash
# Run collection setup script (to be created)
python scripts/setup_qdrant.py
```

**Expected output**:
```
✅ Connected to Qdrant Cloud
✅ Created collection: book_embeddings
✅ Created payload indexes: chapter_id, section_id
```

### 7. Start Development Server

```bash
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

**Expected output**:
```
INFO:     Uvicorn running on http://0.0.0.0:8000 (Press CTRL+C to quit)
INFO:     Started reloader process
INFO:     Started server process
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

### 8. Test Health Endpoint

Open a new terminal and test:

```bash
curl http://localhost:8000/api/health
```

**Expected response**:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-10T10:30:00Z",
  "services": {
    "openai": "healthy",
    "qdrant": "healthy",
    "neon": "healthy"
  }
}
```

✅ **You're ready to go!** The backend is running and all services are connected.

---

## Common Tasks

### Embed Book Content

```bash
# Embed Chapter 1 content (admin API key required)
curl -X POST http://localhost:8000/api/embed \
  -H "Content-Type: application/json" \
  -H "X-Admin-API-Key: your-admin-key" \
  -d '{
    "chunks": [
      {
        "chapter_id": "chapter-1",
        "section_id": "what-is-ros2",
        "chunk_text": "ROS 2 (Robot Operating System 2) is an open-source framework...",
        "heading": "What is ROS 2?",
        "page": 5,
        "chunk_index": 0
      }
    ]
  }'
```

### Ask a Question

```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?"
  }'
```

**Response**:
```json
{
  "answer": "ROS 2 is an open-source framework for robot software development [1].",
  "sources": [
    {
      "chunk_id": "a1b2...",
      "chapter_id": "chapter-1",
      "section_id": "what-is-ros2",
      "heading": "What is ROS 2?",
      "similarity_score": 0.89
    }
  ],
  "confidence_score": 0.85,
  "request_id": "req_abc123",
  "generation_time_ms": 1842
}
```

### Submit Feedback

```bash
curl -X POST http://localhost:8000/api/feedback \
  -H "Content-Type: application/json" \
  -d '{
    "message_id": "msg_123e4567-e89b-12d3-a456-426614174000",
    "rating": "thumbs_up",
    "feedback_text": "Great answer!"
  }'
```

---

## Testing

### Run Unit Tests

```bash
cd backend
pytest tests/unit -v
```

### Run Integration Tests

```bash
# Requires real service credentials in .env
pytest tests/integration -v
```

### Run All Tests

```bash
pytest tests/ -v --cov=src --cov-report=html
```

---

## Development Workflow

### 1. Create a New Feature

```bash
# Create a new branch
git checkout -b feature/your-feature-name

# Make changes to backend/src/

# Run tests
pytest tests/

# Commit changes
git add .
git commit -m "feat: Add your feature description"
```

### 2. Database Migrations

When you modify SQLAlchemy models in `backend/src/models/db_models.py`:

```bash
# Generate migration
alembic revision --autogenerate -m "Descriptive migration name"

# Review generated migration in backend/migrations/versions/

# Apply migration
alembic upgrade head

# Rollback if needed
alembic downgrade -1
```

### 3. Update API Contracts

When you add/modify endpoints:

1. Update `specs/001-rag-chatbot-backend/contracts/openapi.yaml`
2. Regenerate client SDKs (if applicable)
3. Update tests to match new contracts

### 4. Monitor Performance

```bash
# View logs
tail -f backend/logs/app.log

# Monitor API requests
watch -n 1 "curl -s http://localhost:8000/api/health | jq"
```

---

## API Documentation

### Interactive API Docs

Once the server is running, visit:

- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

These provide interactive API documentation with request/response examples and the ability to test endpoints directly in the browser.

---

## Troubleshooting

### Issue: "Connection to Qdrant failed"

**Solution**:
```bash
# Verify Qdrant credentials
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print(os.getenv('QDRANT_URL'))"

# Test connection manually
python -c "from qdrant_client import QdrantClient; client = QdrantClient(url='YOUR_URL', api_key='YOUR_KEY'); print(client.get_collections())"
```

### Issue: "OpenAI API rate limit exceeded"

**Solution**:
- Check your OpenAI usage dashboard: https://platform.openai.com/usage
- Reduce `TOP_K_RESULTS` in `.env` to retrieve fewer chunks
- Implement request queuing/throttling

### Issue: "Database connection pool exhausted"

**Solution**:
```python
# In backend/src/config.py, adjust pool size:
SQLALCHEMY_POOL_SIZE = 10  # Increase from default 5
SQLALCHEMY_MAX_OVERFLOW = 20  # Increase overflow connections
```

### Issue: "Import errors after dependency updates"

**Solution**:
```bash
# Reinstall dependencies from scratch
rm -rf backend/venv
python3.11 -m venv backend/venv
source backend/venv/bin/activate
pip install -r backend/requirements.txt
```

### Issue: "Slow query responses (>3s)"

**Checklist**:
- ✅ Check OpenAI API latency (use `response_metadata.generation_time_ms`)
- ✅ Verify Qdrant vector search is <100ms (add logging)
- ✅ Ensure database queries are indexed (check `EXPLAIN ANALYZE`)
- ✅ Consider implementing response caching for common queries

---

## Environment Configuration

### Key Configuration Variables

Edit `.env` to tune performance:

```bash
# RAG Pipeline Tuning
CHUNK_SIZE=1000                # Tokens per chunk (default: 1000)
CHUNK_OVERLAP=200              # Overlap tokens (default: 200)
TOP_K_RESULTS=5                # Chunks retrieved per query (default: 5)
SIMILARITY_THRESHOLD=0.7       # Min cosine similarity (default: 0.7)
MAX_RESPONSE_TOKENS=500        # Max answer length (default: 500)
TEMPERATURE=0.7                # GPT creativity (default: 0.7)

# Performance
RATE_LIMIT_PER_MINUTE=100      # Requests per user per minute
SQLALCHEMY_POOL_SIZE=10        # DB connection pool size

# Feature Flags
ENABLE_STREAMING=true          # Server-Sent Events streaming
ENABLE_CHAT_HISTORY=true       # Persist chat sessions
ENABLE_FEEDBACK=true           # User feedback collection
ENABLE_SELECTED_TEXT_QUERY=true # Selected text queries
ENABLE_CHAPTER_SCOPING=true    # Chapter-filtered queries
```

---

## Production Deployment

### Pre-Deployment Checklist

- [ ] Update `ENVIRONMENT=production` in `.env`
- [ ] Set strong `ADMIN_API_KEY` (not default)
- [ ] Configure production `CORS_ORIGINS`
- [ ] Enable SSL/TLS for database connections
- [ ] Set up monitoring/alerting (e.g., Sentry, DataDog)
- [ ] Configure log aggregation (e.g., CloudWatch, Papertrail)
- [ ] Run security audit: `pip install safety; safety check`
- [ ] Run full test suite: `pytest tests/ -v`
- [ ] Load test endpoints: `locust -f tests/load/locustfile.py`
- [ ] Verify rate limiting works correctly
- [ ] Test graceful degradation (disable OpenAI temporarily)

### Deployment Platforms

**Recommended**: Railway, Render, or Fly.io (easiest Python deployment)

#### Railway Deployment

```bash
# Install Railway CLI
npm install -g @railway/cli

# Login
railway login

# Initialize project
railway init

# Deploy
railway up

# Set environment variables via Railway dashboard
# Add all variables from .env (use Railway's secret management)
```

#### Docker Deployment

```bash
# Build image
docker build -t rag-backend:latest -f backend/Dockerfile .

# Run container
docker run -p 8000:8000 --env-file .env rag-backend:latest
```

**Dockerfile** (to be created at `backend/Dockerfile`):
```dockerfile
FROM python:3.11-slim

WORKDIR /app

COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/src ./src
COPY backend/migrations ./migrations
COPY backend/alembic.ini .

EXPOSE 8000

CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

---

## Next Steps

1. **Embed Book Content**: Run the embedding script to index all book chapters
2. **Test Queries**: Ask questions and verify answer quality
3. **Tune Parameters**: Adjust `TOP_K_RESULTS`, `SIMILARITY_THRESHOLD` based on performance
4. **Frontend Integration**: Connect existing Docusaurus chatbot UI to this backend
5. **Monitoring**: Set up logging/monitoring for production readiness

---

## Resources

- **API Documentation**: http://localhost:8000/docs (when running)
- **OpenAPI Spec**: [contracts/openapi.yaml](./contracts/openapi.yaml)
- **Data Model**: [data-model.md](./data-model.md)
- **Research**: [research.md](./research.md)
- **Full Spec**: [spec.md](./spec.md)

---

## Support

For issues or questions:
- Check [Troubleshooting](#troubleshooting) section above
- Review [spec.md](./spec.md) for requirements and design decisions
- Consult [research.md](./research.md) for implementation best practices

---

**Status**: Quickstart guide complete. Backend ready for implementation.
