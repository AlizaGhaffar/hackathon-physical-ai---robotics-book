# RAG Chatbot - Testing Guide

## Prerequisites

Before testing, ensure you have:

1. ✅ Python 3.11+ installed
2. ✅ Node.js 20+ installed
3. ✅ All environment variables configured in `.env` file
4. ✅ Virtual environment activated

## Setup Steps

### 1. Backend Setup

```bash
# Navigate to backend directory
cd backend

# Activate virtual environment
source venv/bin/activate  # Linux/Mac
# or
venv\Scripts\activate     # Windows

# Install dependencies (if not already done)
pip install -r requirements.txt

# Run database migrations
alembic upgrade head

# Start the backend server
python -m src.main
```

The backend should start on `http://localhost:8000`

### 2. Verify Backend Health

Open your browser or use curl:

```bash
curl http://localhost:8000/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "version": "1.0.0",
  "timestamp": "2025-12-10T...",
  "services": {
    "openai": "healthy",
    "qdrant": "healthy",
    "neon": "healthy"
  }
}
```

### 3. Index Book Content (First Time Only)

Before you can query, you need to embed the book content:

```bash
# Create a test embedding script
cat > backend/scripts/test_embed.py << 'EOF'
import asyncio
from src.services.embedding_service import embedding_service
from src.services.vector_service import vector_service
from src.utils.chunking import chunk_text

async def embed_test_content():
    """Embed sample content for testing."""

    # Sample Chapter 1 content
    test_content = """
    # What is ROS 2?

    ROS 2 (Robot Operating System 2) is an open-source framework for robot software development.
    It provides tools, libraries, and conventions to create complex robot applications.

    ROS 2 is the next generation of ROS, with improvements in real-time performance,
    security, and multi-robot systems.

    ## Key Features

    - Distributed architecture using DDS
    - Real-time capabilities
    - Cross-platform support (Linux, Windows, macOS)
    - Language support (Python, C++)
    """

    # Chunk the content
    chunks_data = chunk_text(test_content)

    # Prepare chunks for embedding
    chunks = []
    embeddings_list = []

    for idx, (chunk_text, start, end) in enumerate(chunks_data):
        # Embed each chunk
        embedding = await embedding_service.embed_text(chunk_text)

        chunks.append({
            "chunk_id": f"chapter-1-test-{idx}",
            "chapter_id": "chapter-1",
            "source_file": "chapter-1/intro.md",
            "chunk_index": idx,
            "content": chunk_text,
            "metadata": {
                "heading": "What is ROS 2?",
                "section": "Introduction"
            }
        })
        embeddings_list.append(embedding)

    # Upsert to Qdrant
    count = await vector_service.upsert_chunks(chunks, embeddings_list)
    print(f"✅ Successfully embedded {count} chunks for testing")

if __name__ == "__main__":
    asyncio.run(embed_test_content())
EOF

# Run the embedding script
python backend/scripts/test_embed.py
```

### 4. Test Backend API Manually

Test the query endpoint:

```bash
curl -X POST http://localhost:8000/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "chapter_id": "chapter-1"
  }'
```

Expected response:
```json
{
  "request_id": "...",
  "answer": "ROS 2 (Robot Operating System 2) is an open-source framework...",
  "session_id": "...",
  "sources": [
    {
      "chunk_id": "chapter-1-test-0",
      "content": "...",
      "score": 0.95,
      "source_file": "chapter-1/intro.md",
      "chapter_id": "chapter-1"
    }
  ],
  "token_usage": {
    "prompt_tokens": 150,
    "completion_tokens": 75,
    "total_tokens": 225
  },
  "response_time_ms": 2500
}
```

### 5. Frontend Setup

In a new terminal:

```bash
# Navigate to project root (not backend/)
cd /mnt/c/Users/affil/Desktop/book

# Install frontend dependencies (if needed)
npm install

# Start Docusaurus dev server
npm start
```

The frontend should start on `http://localhost:3000`

### 6. Test ChatBot UI

1. Open browser to `http://localhost:3000/docs/chatbot`
2. Type a question: "What is ROS 2?"
3. Press Enter or click Send
4. Verify you receive an answer with sources

## Troubleshooting

### Backend won't start

**Error: Missing environment variables**
- Check `.env` file exists in project root
- Verify all required variables are set:
  - `OPENAI_API_KEY`
  - `QDRANT_API_KEY`
  - `QDRANT_URL`
  - `DATABASE_URL`

**Error: Database connection failed**
- Check Neon Postgres credentials
- Ensure `DATABASE_URL` has `sslmode=require`
- Verify database is accessible from your network

**Error: Qdrant connection failed**
- Check `QDRANT_URL` and `QDRANT_API_KEY`
- Ensure Qdrant cluster is running
- Try accessing Qdrant dashboard directly

### ChatBot shows error

**Error: "Failed to send message"**
- Check backend is running on `http://localhost:8000`
- Open browser console (F12) to see detailed error
- Verify CORS is configured in backend

**No answer returned**
- Check that book content has been indexed (Step 3)
- Try querying backend API directly with curl
- Check backend logs for errors

### Slow responses

- First query is always slower (cold start)
- OpenAI API can take 2-3 seconds
- Check `response_time_ms` in API response
- Consider upgrading OpenAI model if needed

## Success Criteria

✅ Backend health check returns "healthy" for all services
✅ Test embedding script completes without errors
✅ Manual curl test returns valid answer with sources
✅ ChatBot UI loads without errors
✅ ChatBot responds to "What is ROS 2?" within 5 seconds
✅ Response includes citations/sources
✅ Chat history persists across multiple questions

## Next Steps

Once testing is successful:

1. **Index full book content** - Create script to embed all chapters
2. **Deploy backend** - Deploy to Railway, Render, or Vercel
3. **Update frontend config** - Change `backendUrl` to production URL
4. **Add authentication** - Integrate with better-auth.com
5. **Monitor usage** - Set up logging and analytics

## API Documentation

View full API documentation at: `http://localhost:8000/docs`

This provides interactive API testing with Swagger UI.
