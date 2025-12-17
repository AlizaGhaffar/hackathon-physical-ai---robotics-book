# Research & Technical Decisions: Embedding Pipeline

**Feature**: 002-embedding-pipeline
**Date**: 2025-12-11
**Phase**: Phase 0 - Research

## Purpose

This document captures technical research and decisions made during the planning phase for the embedding pipeline. All decisions are evaluated based on the specific requirements of crawling https://physical-ai-robotics-book.vercel.app/, generating embeddings with Cohere, and storing in Qdrant.

---

## Decision 1: Crawling Strategy

### Question
How should we crawl the Docusaurus site to discover all documentation pages?

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **Scrapy Framework** | Robust, handles complex sites, built-in middleware | Heavyweight for single-domain crawl, steep learning curve | ❌ Rejected |
| **Selenium/Playwright** | Handles JavaScript rendering, full browser control | Slow, resource-intensive, unnecessary for Docusaurus | ❌ Rejected |
| **httpx + BeautifulSoup** | Lightweight, fast, sufficient for static HTML | Requires manual link extraction logic | ✅ **Selected** |
| **Sitemap parsing** | Fast, reliable if sitemap exists | Depends on sitemap.xml being up-to-date | 🔄 Backup option |

### Decision: httpx + BeautifulSoup

**Rationale**:
- Docusaurus generates static HTML with predictable structure
- No JavaScript rendering needed (content is in HTML)
- Lightweight and fast for ~100-500 pages
- Easy to extract links from navigation elements
- Fallback to sitemap.xml if manual crawling fails

**Implementation Details**:
```python
async def get_all_urls(base_url: str) -> List[str]:
    # 1. Try fetching sitemap.xml first (fast path)
    # 2. If no sitemap, crawl from homepage
    # 3. Extract links from <nav> and <aside> elements
    # 4. Filter for documentation pages only (exclude /blog, /api-reference)
    # 5. Deduplicate and return absolute URLs
```

**Alternatives Rejected**:
- **Scrapy**: Overkill for single-domain, adds complexity
- **Selenium**: Unnecessary browser overhead for static site

---

## Decision 2: Text Extraction Strategy

### Question
How should we extract clean text from Docusaurus HTML while preserving structure and code blocks?

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **BeautifulSoup .get_text()** | Simple, removes all HTML | Loses structure, hard to extract metadata | ❌ Rejected |
| **html2text library** | Converts to Markdown | May not preserve Docusaurus structure | ❌ Rejected |
| **Custom BeautifulSoup parsing** | Full control, can extract metadata | Requires understanding Docusaurus HTML structure | ✅ **Selected** |
| **Newspaper3k** | Designed for article extraction | Not optimized for documentation sites | ❌ Rejected |

### Decision: Custom BeautifulSoup Parsing

**Rationale**:
- Docusaurus has consistent HTML structure:
  - Main content in `<article>` or `<main>` tags
  - Headings use `<h1>`-`<h6>` with IDs
  - Code blocks in `<pre><code>` tags
  - Navigation in `<nav>` elements (exclude)
- Need to extract metadata (chapter, section) from breadcrumbs and headings
- Must preserve code blocks with language identifiers

**Implementation Details**:
```python
async def extract_text_from_url(url: str) -> DocumentPage:
    # 1. Fetch HTML with httpx
    # 2. Parse with BeautifulSoup
    # 3. Find main content container (article, main, or .markdown)
    # 4. Extract metadata:
    #    - Page title from <h1> or <title>
    #    - Chapter from breadcrumbs or URL path
    #    - Section from first <h2>
    # 5. Remove navigation, footer, sidebar elements
    # 6. Preserve code blocks with language
    # 7. Clean text while maintaining paragraph boundaries
    # 8. Return DocumentPage object
```

**Elements to Extract**:
- Headings (h1-h6) with hierarchy
- Paragraphs
- Lists (ul, ol)
- Code blocks with language identifier
- Tables (convert to text representation)

**Elements to Remove**:
- Navigation (`<nav>`)
- Footer (`<footer>`)
- Sidebar (`<aside>`)
- Edit links ("Edit this page")
- Table of contents (duplicate of headings)

**Alternatives Rejected**:
- **Simple .get_text()**: Loses structure and metadata
- **html2text**: Markdown conversion may not match Docusaurus format

---

## Decision 3: Text Chunking Strategy

### Question
How should we split extracted text into chunks that fit within Cohere's token limits while preserving semantic coherence?

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **Fixed character count** | Simple, predictable size | May split mid-sentence, no token awareness | ❌ Rejected |
| **Sentence-based splitting** | Preserves sentence boundaries | Hard to control exact token count | ❌ Rejected |
| **Recursive character splitting with overlap** | Balances size control and coherence | Requires token estimation | ✅ **Selected** |
| **Semantic splitting (LangChain)** | Intelligent chunking | Adds dependency, slower, overkill | ❌ Rejected |

### Decision: Recursive Character Splitting with Overlap

**Rationale**:
- Cohere's `embed-english-v3.0` has token limit (typically 512 tokens)
- Target chunk size: 500 tokens (~2000 characters assuming 4 chars/token)
- Overlap: 100 tokens (~400 characters) to preserve context across chunks
- Recursive splitting tries to split on paragraph boundaries first, then sentences, then words

**Implementation Details**:
```python
def chunk_text(document: DocumentPage, chunk_size: int = 500, overlap: int = 100) -> List[TextChunk]:
    # 1. Estimate tokens (simple heuristic: text_length / 4)
    # 2. If document < chunk_size, return single chunk
    # 3. Otherwise, split recursively:
    #    - Try splitting on "\n\n" (paragraphs)
    #    - If chunks still too large, split on "." (sentences)
    #    - If still too large, split on " " (words)
    # 4. Add overlap: last 100 tokens of chunk N appear in chunk N+1
    # 5. Attach metadata (chapter, section, URL) to each chunk
    # 6. Generate chunk_id: MD5(url + chunk_index)
```

**Chunk Metadata**:
```python
class ChunkMetadata:
    chapter: str          # e.g., "Chapter 1: ROS 2 Basics"
    section: str          # e.g., "1.2 Publishers and Subscribers"
    page_title: str       # e.g., "Creating Your First Publisher"
    source_url: str       # Full URL
    chunk_id: str         # Unique identifier for deduplication
    chunk_index: int      # Position within document (0, 1, 2, ...)
```

**Alternatives Rejected**:
- **Fixed character count**: Splits mid-sentence, poor quality
- **LangChain splitter**: Adds dependency, unnecessary complexity

---

## Decision 4: Cohere Embedding Model Selection

### Question
Which Cohere embedding model should we use for generating vectors?

### Options Considered

| Model | Dimensions | Quality | Speed | Cost | Decision |
|-------|-----------|---------|-------|------|----------|
| **embed-english-v3.0** | 1024 | High | Fast | Standard | ✅ **Selected** |
| **embed-english-light-v3.0** | 384 | Medium | Very Fast | Low | ❌ Rejected |
| **embed-multilingual-v3.0** | 1024 | High | Medium | Standard | ❌ Rejected |

### Decision: embed-english-v3.0

**Rationale**:
- Latest Cohere model with best quality for English text
- 1024 dimensions balances quality and storage size
- Fast enough for batch processing (~1000 pages in 2 hours)
- English-only documentation (no multilingual needed)
- Standard pricing acceptable for one-time embedding generation

**API Configuration**:
```python
import cohere

co = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))

response = co.embed(
    texts=chunk_texts,
    model="embed-english-v3.0",
    input_type="search_document",  # For indexing documents
    truncate="END"  # Truncate if exceeds limit
)
```

**Rate Limiting**:
- Cohere API rate limits: Check current tier (typically 100 req/min for free tier)
- Batch embeddings: Send up to 96 texts per API call
- Retry logic: Exponential backoff (1s, 2s, 4s delays) with max 3 retries

**Alternatives Rejected**:
- **embed-english-light-v3.0**: Lower quality embeddings hurt retrieval accuracy
- **embed-multilingual-v3.0**: Unnecessary overhead for English-only content

---

## Decision 5: Qdrant Collection Strategy

### Question
Should we use a single collection with metadata filtering or separate collections per chapter?

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **Single collection + metadata filter** | Unified search, easy cross-chapter queries | Slightly slower filtering | ✅ **Selected** |
| **Separate collections per chapter** | Fast chapter-scoped search | Hard to do cross-chapter search, more management | ❌ Rejected |
| **Hybrid: collection per book section** | Balances performance and flexibility | Complex to manage, unclear boundaries | ❌ Rejected |

### Decision: Single Collection with Metadata Filtering

**Rationale**:
- Qdrant supports efficient filtering on payload metadata
- Enables both chapter-scoped and cross-chapter queries
- Simpler management (single collection to maintain)
- Scale: ~1000-5000 vectors is well within single collection capabilities
- Future-proof: Easy to add new chapters without infrastructure changes

**Collection Configuration**:
```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

client.create_collection(
    collection_name="rag_embedding",
    vectors_config=VectorParams(
        size=1024,  # Cohere embed-english-v3.0 dimension
        distance=Distance.COSINE  # Cosine similarity
    )
)
```

**Payload Structure**:
```python
{
    "id": "md5_hash_of_url_and_index",  # Unique identifier
    "vector": [0.123, 0.456, ...],      # 1024 dimensions
    "payload": {
        "chapter": "Chapter 1: ROS 2 Basics",
        "section": "1.2 Publishers and Subscribers",
        "page_title": "Creating Your First Publisher",
        "source_url": "https://physical-ai-robotics-book.vercel.app/chapter1/publishers",
        "text": "Original chunk text...",
        "chunk_index": 0,
        "created_at": "2025-12-11T10:30:00Z"
    }
}
```

**Filtering Example**:
```python
# Chapter-scoped query
results = client.search(
    collection_name="rag_embedding",
    query_vector=query_embedding,
    limit=5,
    query_filter={
        "must": [
            {"key": "chapter", "match": {"value": "Chapter 1: ROS 2 Basics"}}
        ]
    }
)
```

**Alternatives Rejected**:
- **Separate collections**: Complicates cross-chapter search
- **Hybrid approach**: Unnecessary complexity for current scale

---

## Decision 6: Idempotency Strategy

### Question
How should we handle re-running the pipeline on the same URL without creating duplicates?

### Options Considered

| Option | Pros | Cons | Decision |
|--------|------|------|----------|
| **URL-based vector IDs** | Deterministic, enables upsert | MD5 collision risk (negligible) | ✅ **Selected** |
| **Check before insert** | Avoids overwriting | Slower, requires extra query | ❌ Rejected |
| **Timestamp-based versions** | Keeps history | Storage bloat, harder to query | ❌ Rejected |

### Decision: URL-based Vector IDs with Upsert

**Rationale**:
- Generate deterministic ID from URL + chunk_index
- Qdrant's upsert operation replaces existing vectors with same ID
- Re-running pipeline updates embeddings automatically
- No extra queries needed before insert

**ID Generation**:
```python
import hashlib

def generate_vector_id(url: str, chunk_index: int) -> str:
    unique_string = f"{url}::{chunk_index}"
    return hashlib.md5(unique_string.encode()).hexdigest()
```

**Upsert Operation**:
```python
from qdrant_client.models import PointStruct

points = [
    PointStruct(
        id=generate_vector_id(chunk.source_url, chunk.chunk_index),
        vector=embedding,
        payload=chunk.metadata
    )
    for chunk, embedding in chunks_with_embeddings
]

client.upsert(
    collection_name="rag_embedding",
    points=points
)
```

**Benefits**:
- Idempotent: Running twice produces same result
- Incremental updates: Re-crawl single page updates only its vectors
- No duplicate detection logic needed

**Alternatives Rejected**:
- **Check before insert**: Adds latency, extra API calls
- **Versioning**: Unnecessary complexity for batch pipeline

---

## Decision 7: Error Handling and Retry Logic

### Question
How should we handle failures during crawling, embedding, and storage?

### Strategy

**Crawling Failures**:
- Retry HTTP errors with exponential backoff (max 3 retries)
- Log failed URLs to `failed_urls.txt` for manual review
- Continue processing remaining URLs (don't fail entire pipeline)

**Embedding Failures**:
- Retry Cohere API errors with exponential backoff
- If chunk fails after 3 retries, skip and log
- Batch failures: Retry individual chunks from failed batch

**Storage Failures**:
- Retry Qdrant upsert errors with exponential backoff
- If batch fails, retry with smaller batch size (halve)
- Log failed batches to `failed_batches.json` for manual retry

**Implementation**:
```python
from tenacity import retry, stop_after_attempt, wait_exponential

@retry(
    stop=stop_after_attempt(3),
    wait=wait_exponential(multiplier=1, min=1, max=10)
)
async def embed_with_retry(chunks: List[TextChunk]):
    return await embed(chunks)
```

---

## Technology Stack Summary

| Component | Technology | Version | Justification |
|-----------|-----------|---------|---------------|
| **Language** | Python | 3.11+ | Async/await support, modern type hints |
| **Package Manager** | UV | Latest | Fast, modern Python package management |
| **HTTP Client** | httpx | Latest | Async HTTP with connection pooling |
| **HTML Parser** | BeautifulSoup4 | Latest | Robust HTML parsing, Docusaurus compatible |
| **Embedding Service** | Cohere | SDK latest | User-requested, high-quality embeddings |
| **Vector Database** | Qdrant Cloud | Client latest | Free tier, efficient vector search |
| **Environment Config** | python-dotenv | Latest | Secure API key management |
| **Testing** | pytest + pytest-asyncio | Latest | Async test support |
| **Retry Logic** | tenacity | Latest | Exponential backoff for API calls |

---

## Environment Variables

Required environment variables for pipeline execution:

```bash
# .env file structure

# Cohere API
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Pipeline Configuration
BASE_URL=https://physical-ai-robotics-book.vercel.app/
COLLECTION_NAME=rag_embedding
CHUNK_SIZE=500
CHUNK_OVERLAP=100
BATCH_SIZE=100
```

---

## Performance Estimates

Based on technical decisions above:

| Metric | Estimate | Calculation |
|--------|----------|-------------|
| **Total Pages** | 100-500 | Typical Docusaurus book size |
| **Total Chunks** | 1000-5000 | ~10 chunks per page average |
| **Crawling Time** | 5-10 minutes | ~1 page/second with rate limiting |
| **Embedding Time** | 30-60 minutes | Cohere API: ~5s per 1000 tokens |
| **Storage Time** | 5-10 minutes | Qdrant batch upsert: ~2s per 100 vectors |
| **Total Pipeline Time** | 40-80 minutes | End-to-end execution |

**Bottleneck**: Cohere API embedding generation (respecting rate limits)

---

## Next Steps

1. ✅ Research complete - all technical decisions documented
2. → Proceed to Phase 1: Create `data-model.md` with entity definitions
3. → Create `quickstart.md` with setup instructions
4. → Update agent context with new technologies (UV, Cohere, Qdrant)
5. → Generate tasks.md with `/sp.tasks` command

---

**Research Phase Status**: ✅ COMPLETE

All technical unknowns resolved. Ready for Phase 1 design work.
