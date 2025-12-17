# Quickstart Guide: Embedding Pipeline

**Feature**: 002-embedding-pipeline
**Date**: 2025-12-11
**Purpose**: Step-by-step guide to set up and run the embedding pipeline

---

## Overview

This guide walks you through:
1. Setting up the Python environment with UV
2. Installing dependencies
3. Configuring API credentials
4. Running the embedding pipeline
5. Verifying results in Qdrant

**Estimated Time**: 15-20 minutes (excluding pipeline execution time)

---

## Prerequisites

**Required**:
- Python 3.11 or higher
- UV package manager (install from: https://github.com/astral-sh/uv)
- Cohere API key (sign up at: https://cohere.ai/)
- Qdrant Cloud account (free tier at: https://qdrant.tech/)
- Internet connection for crawling deployed Docusaurus site

**Optional**:
- Git (for cloning repository)
- Code editor (VS Code, PyCharm, etc.)

---

## Step 1: Install UV Package Manager

If you don't have UV installed:

### macOS / Linux
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Windows (PowerShell)
```powershell
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

### Verify Installation
```bash
uv --version
# Expected output: uv 0.x.x
```

---

## Step 2: Initialize Project Directory

Navigate to the backend directory and create the embedding-pipeline folder:

```bash
# From repository root
cd backend

# Create embedding-pipeline directory
mkdir -p embedding-pipeline
cd embedding-pipeline
```

---

## Step 3: Initialize UV Project

Initialize a new UV project:

```bash
# Initialize UV project (creates pyproject.toml)
uv init

# Verify pyproject.toml was created
ls -la
```

Expected output: `pyproject.toml` file should exist.

---

## Step 4: Install Dependencies

Add all required dependencies using UV:

```bash
# Add core dependencies
uv add cohere
uv add qdrant-client
uv add beautifulsoup4
uv add httpx
uv add python-dotenv

# Add development dependencies
uv add --dev pytest
uv add --dev pytest-asyncio
```

**Dependency Versions** (UV will install latest compatible versions):
- `cohere`: Cohere Python SDK
- `qdrant-client`: Qdrant vector database client
- `beautifulsoup4`: HTML parsing
- `httpx`: Async HTTP client
- `python-dotenv`: Environment variable management
- `pytest` + `pytest-asyncio`: Testing framework

---

## Step 5: Configure Environment Variables

Create a `.env` file for API credentials:

```bash
# Create .env file
touch .env

# Create .env.example as template
touch .env.example
```

Add the following to `.env.example`:

```bash
# Cohere API Configuration
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster-id.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Pipeline Configuration
BASE_URL=https://physical-ai-robotics-book.vercel.app/
COLLECTION_NAME=rag_embedding
CHUNK_SIZE=500
CHUNK_OVERLAP=100
BATCH_SIZE=100

# Logging
LOG_LEVEL=INFO
```

Now edit `.env` with your actual credentials:

### Get Cohere API Key
1. Visit https://cohere.ai/
2. Sign up / Log in
3. Navigate to API Keys section
4. Copy your API key
5. Paste into `.env` file: `COHERE_API_KEY=your_actual_key`

### Get Qdrant Cloud Credentials
1. Visit https://qdrant.tech/
2. Sign up for free tier
3. Create a new cluster
4. Copy cluster URL (e.g., `https://xyz123.qdrant.io`)
5. Copy API key from cluster settings
6. Paste into `.env` file:
   - `QDRANT_URL=https://xyz123.qdrant.io`
   - `QDRANT_API_KEY=your_actual_key`

---

## Step 6: Create main.py

Create the main pipeline script:

```bash
# Create main.py file
touch main.py
```

Add the following skeleton code (full implementation will be added during task execution):

```python
"""
Embedding Pipeline for Physical AI Robotics Book

This script crawls the deployed Docusaurus site, extracts text,
generates embeddings using Cohere, and stores them in Qdrant.
"""

import asyncio
import os
from typing import List, Tuple
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configuration
BASE_URL = os.getenv("BASE_URL", "https://physical-ai-robotics-book.vercel.app/")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "rag_embedding")
CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "500"))
CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "100"))


async def get_all_urls(base_url: str) -> List[str]:
    """Crawl Docusaurus site and return all documentation page URLs."""
    # TODO: Implement during task execution
    raise NotImplementedError("To be implemented")


async def extract_text_from_url(url: str):
    """Fetch HTML and extract clean text with metadata."""
    # TODO: Implement during task execution
    raise NotImplementedError("To be implemented")


def chunk_text(document, chunk_size: int = 500, overlap: int = 100):
    """Split document text into overlapping chunks."""
    # TODO: Implement during task execution
    raise NotImplementedError("To be implemented")


async def embed(chunks):
    """Generate embeddings for text chunks using Cohere."""
    # TODO: Implement during task execution
    raise NotImplementedError("To be implemented")


async def create_collection(client, collection_name: str = "rag_embedding"):
    """Create or verify Qdrant collection exists with correct config."""
    # TODO: Implement during task execution
    raise NotImplementedError("To be implemented")


async def save_chunk_to_qdrant(client, chunks_with_embeddings, collection_name: str):
    """Batch upsert chunks with embeddings to Qdrant."""
    # TODO: Implement during task execution
    raise NotImplementedError("To be implemented")


async def main():
    """Orchestrate entire pipeline: crawl → extract → chunk → embed → store."""
    print("🚀 Starting Embedding Pipeline...")
    print(f"📍 Target URL: {BASE_URL}")
    print(f"📦 Collection: {COLLECTION_NAME}")

    # TODO: Implement orchestration logic
    print("\n✅ Pipeline execution complete!")


if __name__ == "__main__":
    asyncio.run(main())
```

---

## Step 7: Run the Pipeline (After Implementation)

Once `main.py` is fully implemented (during task execution), run the pipeline:

```bash
# Run pipeline with UV
uv run python main.py
```

**Expected Output**:
```
🚀 Starting Embedding Pipeline...
📍 Target URL: https://physical-ai-robotics-book.vercel.app/
📦 Collection: rag_embedding

📡 Step 1/6: Crawling URLs...
   ✓ Found 127 documentation pages

📄 Step 2/6: Extracting text from pages...
   ✓ Processed 127 pages (100%)

✂️  Step 3/6: Chunking text...
   ✓ Generated 1,543 chunks

🧠 Step 4/6: Generating embeddings...
   ✓ Embedded 1,543 chunks (batched)

🗄️  Step 5/6: Creating Qdrant collection...
   ✓ Collection 'rag_embedding' ready

💾 Step 6/6: Storing vectors in Qdrant...
   ✓ Upserted 1,543 vectors

✅ Pipeline execution complete!
   ⏱️  Total time: 45m 32s
   📊 Stats: 127 pages, 1543 chunks, 1543 vectors
```

---

## Step 8: Verify Results in Qdrant

Check that vectors were stored correctly:

### Option 1: Qdrant Cloud Dashboard
1. Log in to https://cloud.qdrant.io/
2. Navigate to your cluster
3. Select collection `rag_embedding`
4. Verify vector count matches expected number

### Option 2: Python Script
Create a verification script `verify.py`:

```python
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Get collection info
collection_info = client.get_collection("rag_embedding")
print(f"✅ Collection: rag_embedding")
print(f"📊 Vector count: {collection_info.vectors_count}")
print(f"📏 Vector dimensions: {collection_info.config.params.vectors.size}")

# Test search (sample query)
sample_vector = [0.0] * 1024  # Placeholder vector
results = client.search(
    collection_name="rag_embedding",
    query_vector=sample_vector,
    limit=3
)

print(f"\n🔍 Sample search results:")
for i, result in enumerate(results, 1):
    print(f"   {i}. Chapter: {result.payload.get('chapter')}")
    print(f"      Title: {result.payload.get('page_title')}")
    print(f"      Score: {result.score:.4f}")
```

Run verification:
```bash
uv run python verify.py
```

---

## Troubleshooting

### Error: "COHERE_API_KEY not found"
**Solution**: Check that `.env` file exists and contains `COHERE_API_KEY=...`

### Error: "Connection to Qdrant failed"
**Solution**: Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct. Test connection via Qdrant Cloud dashboard.

### Error: "Rate limit exceeded"
**Solution**: Cohere API rate limits hit. Wait 60 seconds and retry. Consider adding `time.sleep()` between batches.

### Error: "Vector dimension mismatch"
**Solution**: Ensure using `embed-english-v3.0` model (1024 dimensions). Check collection config.

### Pipeline runs but 0 vectors stored
**Solution**:
1. Check `get_all_urls` returns non-empty list
2. Verify `extract_text_from_url` extracts non-empty text
3. Check Qdrant collection was created successfully

---

## Next Steps

After running the pipeline successfully:

1. **Test Retrieval**: Query Qdrant with sample text to verify semantic search works
2. **Integrate with RAG Backend**: Connect the RAG query API (001-rag-chatbot-backend) to this collection
3. **Set Up Incremental Updates**: Run pipeline periodically to update embeddings as book content changes
4. **Monitor Performance**: Track embedding generation time and optimize batch sizes

---

## Project Structure (Final)

```
backend/embedding-pipeline/
├── main.py              # Main pipeline script
├── pyproject.toml       # UV project config (auto-generated)
├── .env                 # API credentials (DO NOT COMMIT)
├── .env.example         # Template for .env
├── README.md            # Project documentation
├── verify.py            # Verification script (optional)
└── tests/               # Unit tests (created during implementation)
    ├── test_crawler.py
    ├── test_extraction.py
    ├── test_chunking.py
    ├── test_embedding.py
    └── test_qdrant.py
```

---

## Development Tips

### Running Tests
```bash
# Run all tests
uv run pytest tests/

# Run specific test file
uv run pytest tests/test_crawler.py

# Run with coverage
uv run pytest tests/ --cov=. --cov-report=html
```

### Adding New Dependencies
```bash
# Add a new package
uv add package-name

# Add dev dependency
uv add --dev package-name
```

### Updating Dependencies
```bash
# Update all packages
uv update
```

---

## Security Best Practices

1. **Never commit `.env` file**: Add to `.gitignore`
2. **Use `.env.example`**: Commit template without actual credentials
3. **Rotate API keys**: Regularly rotate Cohere and Qdrant keys
4. **Restrict Qdrant access**: Use API keys with minimal required permissions

---

**Quickstart Guide Status**: ✅ COMPLETE

All setup steps documented. Ready for implementation phase.
