# Embedding Pipeline for Physical AI Robotics Book

This pipeline crawls the deployed Docusaurus site, extracts clean text, generates embeddings using Cohere, and stores them in Qdrant Cloud vector database for RAG retrieval.

## Features

- 🕷️ **URL Crawling**: Automatically discovers all documentation pages via sitemap.xml
- 📄 **Text Extraction**: Cleans HTML and extracts readable content with metadata
- ✂️ **Smart Chunking**: Splits text into overlapping chunks (500 tokens, 100 overlap)
- 🧠 **Embeddings**: Generates vector embeddings using Cohere embed-english-v3.0
- 🗄️ **Vector Storage**: Stores embeddings in Qdrant Cloud with searchable metadata

## Prerequisites

- Python 3.11 or higher
- Cohere API key ([sign up](https://cohere.ai/))
- Qdrant Cloud account ([free tier](https://qdrant.tech/))

## Quick Start

### 1. Install Dependencies

```bash
cd backend/embedding-pipeline
python -m venv .venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
pip install -e .
```

### 2. Configure Environment

```bash
cp .env.example .env
# Edit .env with your API keys
```

Required environment variables:
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_URL`: Your Qdrant cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key

### 3. Run Pipeline

```bash
python main.py
```

## Configuration

Edit `.env` to customize:

- `BASE_URL`: Target Docusaurus site (default: https://physical-ai-robotics-book.vercel.app/)
- `COLLECTION_NAME`: Qdrant collection name (default: rag_embedding)
- `CHUNK_SIZE`: Chunk size in tokens (default: 500)
- `CHUNK_OVERLAP`: Overlap between chunks in tokens (default: 100)
- `BATCH_SIZE`: Batch size for embedding generation (default: 100)
- `LOG_LEVEL`: Logging level (default: INFO)

## Pipeline Stages

1. **Crawl URLs**: Fetch sitemap.xml and extract all documentation page URLs
2. **Extract Text**: Fetch HTML and extract clean text with metadata
3. **Chunk Text**: Split into overlapping chunks for embedding
4. **Generate Embeddings**: Call Cohere API to generate 1024-dim vectors
5. **Create Collection**: Ensure Qdrant collection exists with correct config
6. **Store Vectors**: Batch upsert embeddings with metadata

## Output

The pipeline stores vectors in Qdrant with the following metadata:
- `chapter`: Chapter name/number
- `section`: Section name
- `page_title`: Page title
- `source_url`: Original page URL
- `text`: Original chunk text
- `chunk_index`: Position within document
- `created_at`: Timestamp

## Troubleshooting

### Error: "COHERE_API_KEY not found"
Check that `.env` file exists and contains `COHERE_API_KEY=...`

### Error: "Connection to Qdrant failed"
Verify `QDRANT_URL` and `QDRANT_API_KEY` are correct

### Error: "Rate limit exceeded"
Cohere API rate limits hit. Wait 60 seconds and retry

### Pipeline runs but 0 vectors stored
1. Check `get_all_urls` returns non-empty list
2. Verify `extract_text_from_url` extracts non-empty text
3. Check Qdrant collection was created successfully

## Development

### Running Tests

```bash
pip install -e ".[dev]"
pytest tests/ -v
```

### Project Structure

```
backend/embedding-pipeline/
├── main.py              # Main pipeline script
├── pyproject.toml       # Project configuration
├── .env.example         # Environment variable template
├── .env                 # API credentials (DO NOT COMMIT)
├── README.md            # This file
└── tests/               # Unit tests
```

## License

Part of the Physical AI Robotics Book project.
