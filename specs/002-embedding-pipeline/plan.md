# Implementation Plan: Embedding Pipeline Setup

**Branch**: `002-embedding-pipeline` | **Date**: 2025-12-11 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-embedding-pipeline/spec.md`

## Summary

Build a single-file Python script (`main.py`) that crawls the deployed Docusaurus site at https://physical-ai-robotics-book.vercel.app/, extracts clean text content, generates embeddings using Cohere API, and stores them in Qdrant Cloud vector database with metadata for RAG retrieval. The pipeline will be initialized using UV package manager in a dedicated `backend/embedding-pipeline/` directory.

**Technical Approach**: Sequential pipeline execution in main.py with functions: `get_all_urls` → `extract_text_from_url` → `chunk_text` → `embed` → `create_collection` (rag_embedding) → `save_chunk_to_qdrant` → `main()` orchestrator.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**:
- UV package manager (project initialization and dependency management)
- Cohere Python SDK (embedding generation - `cohere`)
- Qdrant Client (`qdrant-client`)
- BeautifulSoup4 (`beautifulsoup4`) for HTML parsing
- Requests (`httpx`) for HTTP fetching
- python-dotenv for environment variable management

**Storage**: Qdrant Cloud Free Tier (vector database)
**Testing**: pytest with async support (pytest-asyncio)
**Target Platform**: Python 3.11+ on any OS (development), Linux server (production)
**Project Type**: Single-file script (main.py) for batch processing
**Performance Goals**:
- Process 1000 pages within 2 hours
- Embedding generation: <5s per 1000 tokens
- Qdrant upsert: <2s per 100 vectors

**Constraints**:
- Cohere API rate limits (respect with exponential backoff)
- Qdrant Cloud Free Tier limits (check collection size limits)
- Single file implementation (main.py only)
- No persistent state between runs (idempotent execution)

**Scale/Scope**:
- Target: ~100-500 documentation pages
- ~1000-5000 text chunks total
- Vector dimensions: 1024 (Cohere embed-english-v3.0)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Alignment with Constitution Principles

#### XIII. RAG Backend Architecture ✅ PASS (with notes)
- **Requirement**: Backend MUST use FastAPI framework
- **Status**: ⚠️ DEVIATION - Using single-file script instead of FastAPI
- **Justification**: Embedding pipeline is a one-time batch process, not a runtime API service. FastAPI is required for the query/retrieval backend (separate from this pipeline). This pipeline generates the data that the FastAPI backend will consume.
- **Mitigation**: The RAG query backend (001-rag-chatbot-backend) already uses FastAPI per constitution. This pipeline is data preparation only.

#### XIV. Embedding Pipeline Excellence ✅ PASS
- **Requirement**: Embeddings MUST use OpenAI's text-embedding models
- **Status**: ⚠️ DEVIATION - Using Cohere embeddings instead of OpenAI
- **Justification**: User explicitly requested Cohere for this pipeline. Cohere's `embed-english-v3.0` provides equivalent quality to OpenAI's text-embedding-3-small.
- **Mitigation**: Document Cohere model version and dimensions. Ensure consistency across all embeddings. Consider OpenAI migration path if needed.

#### XV. Vector Storage Strategy ✅ PASS
- **Requirement**: Vector database MUST use Qdrant Cloud Free Tier
- **Status**: ✅ COMPLIANT - Using Qdrant Cloud with collection name `rag_embedding`
- **Validation**:
  - Collections namespaced per chapter (metadata filtering)
  - Metadata includes: chapter_id, section, URL, text
  - Batch upsert (100-500 vectors per batch)
  - Duplicate detection via URL-based ID

#### XVI. Context Retrieval Precision ✅ PASS
- **Requirement**: Retrieval MUST support chapter-scoped queries
- **Status**: ✅ COMPLIANT - Metadata includes chapter/section for filtering
- **Validation**: Chunk metadata will include all required fields for precise retrieval

#### Constitution Compliance Summary
- **Violations**: 2 deviations (FastAPI, OpenAI → Cohere)
- **Justification**: Both deviations are acceptable:
  1. FastAPI not needed for batch pipeline (separate from query API)
  2. Cohere provides equivalent embedding quality per user request
- **Gate Status**: ✅ PASS - Proceed to Phase 0

## Project Structure

### Documentation (this feature)

```text
specs/002-embedding-pipeline/
├── plan.md              # This file (/sp.plan command output)
├── spec.md              # Feature specification
├── research.md          # Phase 0 output - technical decisions
├── data-model.md        # Phase 1 output - data entities
├── quickstart.md        # Phase 1 output - setup/usage guide
├── contracts/           # Phase 1 output - API contracts (N/A for batch script)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
backend/
└── embedding-pipeline/
    ├── main.py              # Single-file pipeline implementation
    ├── pyproject.toml       # UV project configuration
    ├── .env.example         # Environment variable template
    ├── README.md            # Setup and usage instructions
    └── tests/
        ├── test_crawler.py      # Unit tests for URL crawling
        ├── test_extraction.py   # Unit tests for text extraction
        ├── test_chunking.py     # Unit tests for chunking logic
        ├── test_embedding.py    # Unit tests for Cohere integration
        └── test_qdrant.py       # Unit tests for Qdrant operations
```

**Structure Decision**: Single project structure (Option 1) is appropriate. This is a standalone batch script, not a web application. The embedding pipeline is separate from the existing FastAPI RAG backend (backend/src/). Keeping it in a dedicated directory (`backend/embedding-pipeline/`) maintains separation of concerns.

## Complexity Tracking

> This section documents deviations from constitution that require justification.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Not using FastAPI | Pipeline is one-time batch process, not API service | FastAPI adds unnecessary complexity for non-HTTP workflows. Query backend (separate) uses FastAPI. |
| Using Cohere instead of OpenAI | User explicitly requested Cohere embeddings | OpenAI embeddings work equivalently but user preference specified Cohere for this pipeline. |

---

## Phase 0: Research & Technical Decisions

**Output**: `research.md` - See detailed research document

**Key Decisions Made**:

1. **Crawling Strategy**: Use `httpx` + `BeautifulSoup4`
   - **Rationale**: Docusaurus sites have predictable HTML structure; no JavaScript rendering needed
   - **Alternative Considered**: Scrapy (rejected: overkill for single-domain crawl)
   - **Alternative Considered**: Selenium (rejected: overhead of browser automation unnecessary)

2. **Text Chunking Strategy**: Recursive character-based splitting with overlap
   - **Rationale**: Preserves sentence boundaries while fitting Cohere's token limits
   - **Chunk Size**: 500 tokens (~2000 characters) with 100 token overlap
   - **Alternative Considered**: Sentence-based (rejected: harder to control token counts)

3. **Cohere Model Selection**: `embed-english-v3.0`
   - **Rationale**: Latest Cohere embedding model, 1024 dimensions, good quality/speed tradeoff
   - **Alternative Considered**: `embed-english-light-v3.0` (rejected: lower quality)

4. **Qdrant Collection Strategy**: Single collection `rag_embedding` with chapter metadata
   - **Rationale**: Enables filtering by chapter while maintaining unified search
   - **Alternative Considered**: Separate collections per chapter (rejected: harder to manage cross-chapter queries)

5. **Idempotency Strategy**: URL-based vector IDs
   - **Rationale**: Re-running pipeline on same URL replaces old embedding (update-in-place)
   - **Implementation**: MD5 hash of URL as Qdrant point ID

---

## Phase 1: Design & Contracts

### Data Model

**Output**: `data-model.md` - See detailed data model document

**Core Entities**:

1. **DocumentPage**
   - Attributes: url (str), html_content (str), clean_text (str), metadata (dict)
   - Purpose: Represents a single Docusaurus page during extraction

2. **TextChunk**
   - Attributes: text (str), chunk_index (int), metadata (ChunkMetadata)
   - Purpose: Represents a chunk of text ready for embedding

3. **ChunkMetadata**
   - Attributes: chapter (str), section (str), page_title (str), source_url (str), chunk_id (str)
   - Purpose: Metadata attached to each vector in Qdrant

4. **QdrantPoint**
   - Attributes: id (str), vector (List[float]), payload (ChunkMetadata)
   - Purpose: Final format for Qdrant upsert

### API Contracts

**Note**: This is a batch script, not an API service. No REST endpoints.

**Internal Function Contracts**:

```python
# Function signatures for main.py

async def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl Docusaurus site and return all documentation page URLs.

    Args:
        base_url: Root URL (e.g., "https://physical-ai-robotics-book.vercel.app/")

    Returns:
        List of absolute URLs to documentation pages

    Raises:
        httpx.HTTPError: If base URL is unreachable
    """

async def extract_text_from_url(url: str) -> DocumentPage:
    """
    Fetch HTML and extract clean text with metadata.

    Args:
        url: Absolute URL to documentation page

    Returns:
        DocumentPage with clean_text and metadata extracted

    Raises:
        httpx.HTTPError: If URL fetch fails
        ValueError: If HTML parsing fails
    """

def chunk_text(document: DocumentPage, chunk_size: int = 500, overlap: int = 100) -> List[TextChunk]:
    """
    Split document text into overlapping chunks.

    Args:
        document: DocumentPage with clean_text
        chunk_size: Target chunk size in tokens
        overlap: Number of overlapping tokens between chunks

    Returns:
        List of TextChunk objects with metadata
    """

async def embed(chunks: List[TextChunk]) -> List[Tuple[TextChunk, List[float]]]:
    """
    Generate embeddings for text chunks using Cohere.

    Args:
        chunks: List of TextChunk objects

    Returns:
        List of (chunk, embedding_vector) tuples

    Raises:
        cohere.CohereError: If API call fails
    """

async def create_collection(client: QdrantClient, collection_name: str = "rag_embedding") -> None:
    """
    Create or verify Qdrant collection exists with correct config.

    Args:
        client: Initialized QdrantClient
        collection_name: Name of collection (default: "rag_embedding")

    Raises:
        QdrantException: If collection creation fails
    """

async def save_chunk_to_qdrant(
    client: QdrantClient,
    chunks_with_embeddings: List[Tuple[TextChunk, List[float]]],
    collection_name: str = "rag_embedding"
) -> int:
    """
    Batch upsert chunks with embeddings to Qdrant.

    Args:
        client: Initialized QdrantClient
        chunks_with_embeddings: List of (chunk, vector) tuples
        collection_name: Target collection name

    Returns:
        Number of vectors successfully upserted

    Raises:
        QdrantException: If upsert fails
    """

async def main() -> None:
    """
    Orchestrate entire pipeline: crawl → extract → chunk → embed → store.
    """
```

### Quickstart Guide

**Output**: `quickstart.md` - See detailed setup guide

**Quick Setup Steps**:

```bash
# 1. Initialize project with UV
cd backend/embedding-pipeline
uv init
uv add cohere qdrant-client beautifulsoup4 httpx python-dotenv

# 2. Configure environment
cp .env.example .env
# Edit .env with COHERE_API_KEY and QDRANT credentials

# 3. Run pipeline
uv run python main.py

# 4. Verify in Qdrant
# Check collection "rag_embedding" has expected vector count
```

---

## Phase 2: Task Breakdown

**Note**: Task breakdown is generated by `/sp.tasks` command (separate from this plan).

**Expected Task Structure** (preview):

1. **Setup Tasks**:
   - Initialize UV project
   - Configure environment variables
   - Set up Cohere and Qdrant clients

2. **Crawling Tasks**:
   - Implement `get_all_urls` with sitemap parsing
   - Add URL deduplication and filtering

3. **Extraction Tasks**:
   - Implement `extract_text_from_url` with BeautifulSoup
   - Extract metadata (chapter, section, title)

4. **Chunking Tasks**:
   - Implement `chunk_text` with token-aware splitting
   - Preserve chunk overlap for context

5. **Embedding Tasks**:
   - Implement `embed` with Cohere API
   - Add retry logic and rate limiting

6. **Storage Tasks**:
   - Implement `create_collection` with vector config
   - Implement `save_chunk_to_qdrant` with batch upsert

7. **Orchestration Tasks**:
   - Implement `main` function with error handling
   - Add progress logging

8. **Testing Tasks**:
   - Unit tests for all functions
   - Integration test end-to-end

---

## Success Criteria Mapping

Mapping functional requirements from spec.md to implementation plan:

| Spec Requirement | Implementation Component |
|------------------|-------------------------|
| FR-001: Crawl Docusaurus site | `get_all_urls()` |
| FR-002: Extract clean text | `extract_text_from_url()` with BeautifulSoup |
| FR-003: Preserve code blocks | HTML parsing preserves `<code>` tags |
| FR-004: Extract metadata | Metadata extraction in `extract_text_from_url()` |
| FR-005: Chunk within token limits | `chunk_text()` with 500 token chunks |
| FR-006: Generate embeddings | `embed()` with Cohere API |
| FR-007: Handle rate limits | Exponential backoff in `embed()` |
| FR-008: Store in Qdrant | `save_chunk_to_qdrant()` |
| FR-009: Deduplication | URL-based vector IDs |
| FR-010: Progress logging | Logging throughout `main()` |
| FR-011: Incremental updates | Upsert operation replaces old vectors |
| FR-012: Validate dimensions | Check vector length before upsert |
| FR-013: Create collection | `create_collection()` |

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown in `tasks.md`
2. Implement `main.py` following function signatures above
3. Write unit tests for each function
4. Run end-to-end test with sample URLs
5. Execute pipeline on full Docusaurus site
6. Verify Qdrant collection has expected vectors

---

**Post-Design Constitution Re-Check**: ✅ PASS

All constitution requirements met with documented deviations (FastAPI N/A for batch script, Cohere per user request). Ready to proceed to task generation.
