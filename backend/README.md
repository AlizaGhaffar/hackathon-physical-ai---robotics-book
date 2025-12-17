# RAG Chatbot Backend

**Feature**: Retrieval-Augmented Generation (RAG) chatbot backend for Physical AI Textbook

This backend provides question-answering capabilities on book content using:
- **FastAPI** - Modern Python web framework
- **OpenAI API** - Text embeddings (text-embedding-3-small) and GPT-4-turbo for generation
- **Qdrant Cloud** - Vector database for semantic search
- **Neon Serverless Postgres** - Chat history and analytics

---

## Quick Start

### Prerequisites

- Python 3.11+
- pip package manager
- Credentials configured in `.env` file (see root directory)

### Setup (5 minutes)

```bash
# Navigate to backend directory
cd backend

# Create virtual environment
python3.11 -m venv venv

# Activate virtual environment
# Linux/Mac:
source venv/bin/activate
# Windows:
venv\Scripts\activate

# Install dependencies
pip install --upgrade pip
pip install -r requirements.txt

# Initialize database
alembic revision --autogenerate -m "Initial schema"
alembic upgrade head

# Start development server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000
```

### Test the API

```bash
# Health check
curl http://localhost:8000/api/health

# Ask a question (after loading content)
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'
```

---

## Project Structure

```
backend/
├── src/
│   ├── main.py                  # FastAPI application entry point
│   ├── config.py                # Environment configuration
│   ├── models/                  # Pydantic + SQLAlchemy models
│   │   ├── request_models.py   # API request schemas
│   │   ├── response_models.py  # API response schemas
│   │   └── db_models.py        # Database models
│   ├── services/                # Business logic layer
│   │   ├── embedding_service.py    # OpenAI embeddings
│   │   ├── vector_service.py       # Qdrant operations
│   │   ├── query_service.py        # RAG orchestration
│   │   ├── chat_service.py         # Chat history
│   │   └── feedback_service.py     # User feedback
│   ├── api/                     # API route handlers
│   │   ├── query.py            # Question-answering
│   │   ├── embed.py            # Content embedding (admin)
│   │   ├── health.py           # Health checks
│   │   └── feedback.py         # Feedback collection
│   ├── middleware/              # FastAPI middleware
│   │   ├── rate_limiter.py     # Rate limiting
│   │   ├── cors.py             # CORS configuration
│   │   └── error_handler.py    # Global error handling
│   └── utils/                   # Utility functions
│       ├── chunking.py         # Text chunking
│       ├── logger.py           # Logging setup
│       └── validators.py       # Input validation
├── tests/
│   ├── unit/                    # Unit tests
│   ├── integration/             # Integration tests
│   └── contract/                # API contract tests
├── migrations/                  # Alembic migrations
├── scripts/                     # Utility scripts
├── requirements.txt             # Python dependencies
├── Dockerfile                   # Container image
└── README.md                   # This file
```

---

## API Endpoints

### User Endpoints

- **POST /api/query** - Ask a question about book content
- **POST /api/query/stream** - Streaming responses (SSE)
- **POST /api/feedback** - Submit feedback on responses
- **GET /api/health** - Service health check

### Admin Endpoints

- **POST /api/embed** - Embed book content into vector database (requires admin API key)

---

## Environment Variables

All credentials are configured in `.env` file at repository root. Key variables:

```bash
# Qdrant Cloud
QDRANT_API_KEY=your_api_key
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_COLLECTION_NAME=book_embeddings

# Neon Serverless Postgres
DATABASE_URL=postgresql://user:pass@host/db?sslmode=require

# OpenAI API
OPENAI_API_KEY=sk-proj-your_key
OPENAI_MODEL=gpt-4-turbo-preview
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Application
ENVIRONMENT=development
API_PORT=8000
RATE_LIMIT_PER_MINUTE=100
```

See `.env.example` for full list of configuration options.

---

## Development

### Run Tests

```bash
# Unit tests
pytest tests/unit -v

# Integration tests
pytest tests/integration -v

# All tests with coverage
pytest tests/ --cov=src --cov-report=html
```

### Database Migrations

```bash
# Generate migration
alembic revision --autogenerate -m "Description"

# Apply migrations
alembic upgrade head

# Rollback one migration
alembic downgrade -1
```

### Code Quality

```bash
# Format code
black src/ tests/

# Lint code
flake8 src/ tests/

# Type checking
mypy src/
```

---

## Deployment

### Docker

```bash
# Build image
docker build -t rag-backend:latest .

# Run container
docker run -p 8000:8000 --env-file ../.env rag-backend:latest
```

### Production Platforms

**Railway**:
```bash
railway init
railway up
```

**Vercel** (via Docker):
```bash
vercel --docker
```

**Render**:
- Connect GitHub repository
- Set environment variables in dashboard
- Deploy from `backend/` directory

---

## Troubleshooting

### "Connection to Qdrant failed"

```bash
# Verify credentials
python -c "from dotenv import load_dotenv; import os; load_dotenv(); print(os.getenv('QDRANT_URL'))"
```

### "OpenAI API rate limit exceeded"

- Check usage at https://platform.openai.com/usage
- Reduce `TOP_K_RESULTS` in `.env`
- Implement request queuing

### "Database connection pool exhausted"

Increase pool size in `src/config.py`:
```python
SQLALCHEMY_POOL_SIZE = 10
SQLALCHEMY_MAX_OVERFLOW = 20
```

---

## Documentation

- **Full Specification**: `../specs/001-rag-chatbot-backend/spec.md`
- **Architecture Plan**: `../specs/001-rag-chatbot-backend/plan.md`
- **API Contracts**: `../specs/001-rag-chatbot-backend/contracts/openapi.yaml`
- **Data Model**: `../specs/001-rag-chatbot-backend/data-model.md`
- **Research**: `../specs/001-rag-chatbot-backend/research.md`

Interactive API documentation available at:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

---

## Support

For issues or questions, consult the design documents in `specs/001-rag-chatbot-backend/`.

**License**: MIT
