# Data Model: Embedding Pipeline

**Feature**: 002-embedding-pipeline
**Date**: 2025-12-11
**Phase**: Phase 1 - Design

## Purpose

This document defines the data entities, their attributes, relationships, and validation rules for the embedding pipeline. These entities represent the data flow from URL crawling through embedding generation to vector storage.

---

## Entity Overview

The embedding pipeline processes data through four primary entity types:

```
DocumentPage → TextChunk → Embedding → QdrantPoint
```

1. **DocumentPage**: Represents a single crawled page with raw HTML and extracted text
2. **TextChunk**: Represents a segment of text with metadata, ready for embedding
3. **Embedding**: Represents a vector embedding paired with its source chunk
4. **QdrantPoint**: Represents the final stored vector in Qdrant database

---

## Entity Definitions

### 1. DocumentPage

Represents a single Docusaurus page after crawling and text extraction.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `url` | str | Yes | Absolute URL of the page | Must be valid HTTPS URL |
| `html_content` | str | Yes | Raw HTML content | Non-empty string |
| `clean_text` | str | Yes | Extracted clean text | Non-empty after extraction |
| `metadata` | PageMetadata | Yes | Extracted page metadata | See PageMetadata schema |
| `crawled_at` | datetime | Yes | Timestamp of crawl | ISO 8601 UTC |
| `content_hash` | str | Yes | MD5 hash of clean_text | For change detection |

**Relationships**:
- One DocumentPage → Many TextChunks (1:N)

**Lifecycle**:
1. Created during URL crawling (`get_all_urls`)
2. Populated during text extraction (`extract_text_from_url`)
3. Consumed during chunking (`chunk_text`)
4. Discarded after chunking (not persisted)

**Example**:
```python
from dataclasses import dataclass
from datetime import datetime

@dataclass
class DocumentPage:
    url: str
    html_content: str
    clean_text: str
    metadata: PageMetadata
    crawled_at: datetime
    content_hash: str

    def __post_init__(self):
        # Validation
        assert self.url.startswith('https://'), "URL must be HTTPS"
        assert len(self.clean_text) > 0, "Extracted text cannot be empty"
```

---

### 2. PageMetadata

Metadata extracted from a Docusaurus page.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `chapter` | str | Yes | Chapter name/number | e.g., "Chapter 1: ROS 2 Basics" |
| `section` | str | No | Section name | e.g., "1.2 Publishers" |
| `page_title` | str | Yes | Page title from `<h1>` or `<title>` | Non-empty string |
| `breadcrumbs` | List[str] | No | Navigation breadcrumbs | e.g., ["Home", "Ch1", "Publishers"] |
| `heading_hierarchy` | List[str] | No | All heading tags (h1-h6) in order | For context preservation |

**Extraction Rules**:
- **Chapter**: Extract from breadcrumbs, URL path, or first `<h1>`
- **Section**: Extract from first `<h2>` or URL path segment
- **Page Title**: Prefer `<h1>`, fallback to `<title>` tag
- **Breadcrumbs**: Extract from `<nav aria-label="breadcrumb">` or similar
- **Heading Hierarchy**: Extract all `<h1>`-`<h6>` tags in document order

**Example**:
```python
from dataclasses import dataclass
from typing import List, Optional

@dataclass
class PageMetadata:
    chapter: str
    page_title: str
    section: Optional[str] = None
    breadcrumbs: Optional[List[str]] = None
    heading_hierarchy: Optional[List[str]] = None
```

---

### 3. TextChunk

Represents a segment of text ready for embedding, with associated metadata.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `text` | str | Yes | Chunk text content | 10 ≤ length ≤ 2000 chars |
| `chunk_index` | int | Yes | Position within document | ≥ 0 |
| `metadata` | ChunkMetadata | Yes | Chunk-specific metadata | See ChunkMetadata schema |
| `token_count` | int | Yes | Estimated token count | Calculated via heuristic |

**Relationships**:
- Many TextChunks → One DocumentPage (N:1, via source_url)
- One TextChunk → One Embedding (1:1)

**Lifecycle**:
1. Created during text chunking (`chunk_text`)
2. Consumed during embedding generation (`embed`)
3. Paired with embedding vector
4. Transformed into QdrantPoint for storage

**Validation Rules**:
- Text length: 10-2000 characters (prevent empty or oversized chunks)
- Token count: ≤500 tokens (Cohere limit)
- Chunk index: Non-negative, sequential

**Example**:
```python
from dataclasses import dataclass

@dataclass
class TextChunk:
    text: str
    chunk_index: int
    metadata: 'ChunkMetadata'
    token_count: int

    def __post_init__(self):
        assert 10 <= len(self.text) <= 2000, "Text length out of range"
        assert self.chunk_index >= 0, "Chunk index must be non-negative"
        assert self.token_count <= 500, "Token count exceeds limit"
```

---

### 4. ChunkMetadata

Metadata attached to each text chunk for retrieval and filtering.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `chapter` | str | Yes | Chapter name | From PageMetadata |
| `section` | str | No | Section name | From PageMetadata |
| `page_title` | str | Yes | Page title | From PageMetadata |
| `source_url` | str | Yes | Original page URL | Valid HTTPS URL |
| `chunk_id` | str | Yes | Unique identifier | MD5(url + chunk_index) |
| `created_at` | str | Yes | ISO 8601 timestamp | UTC timezone |

**ID Generation**:
```python
import hashlib
from datetime import datetime, timezone

def generate_chunk_id(url: str, chunk_index: int) -> str:
    unique_string = f"{url}::{chunk_index}"
    return hashlib.md5(unique_string.encode()).hexdigest()

def generate_timestamp() -> str:
    return datetime.now(timezone.utc).isoformat()
```

**Purpose**:
- **chapter/section**: Enable filtered retrieval (chapter-scoped queries)
- **page_title**: Context for user when viewing sources
- **source_url**: Link back to original documentation
- **chunk_id**: Deterministic ID for deduplication
- **created_at**: Track when embedding was generated

**Example**:
```python
from dataclasses import dataclass
from typing import Optional

@dataclass
class ChunkMetadata:
    chapter: str
    page_title: str
    source_url: str
    chunk_id: str
    created_at: str
    section: Optional[str] = None
```

---

### 5. Embedding

Represents a vector embedding paired with its source chunk.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `chunk` | TextChunk | Yes | Source text chunk | See TextChunk validation |
| `vector` | List[float] | Yes | Embedding vector | Length = 1024 (Cohere v3) |
| `model` | str | Yes | Cohere model name | "embed-english-v3.0" |
| `generated_at` | datetime | Yes | Embedding generation time | UTC timezone |

**Relationships**:
- One Embedding → One TextChunk (1:1)
- One Embedding → One QdrantPoint (1:1, transformed)

**Lifecycle**:
1. Created during embedding generation (`embed`)
2. Validated (vector dimensions)
3. Transformed into QdrantPoint
4. Discarded after storage

**Validation Rules**:
- Vector length must equal 1024 (Cohere embed-english-v3.0)
- All vector values must be finite floats (no NaN, Inf)
- Model name must match expected ("embed-english-v3.0")

**Example**:
```python
from dataclasses import dataclass
from datetime import datetime
from typing import List

@dataclass
class Embedding:
    chunk: TextChunk
    vector: List[float]
    model: str
    generated_at: datetime

    def __post_init__(self):
        assert len(self.vector) == 1024, f"Expected 1024 dims, got {len(self.vector)}"
        assert all(isinstance(v, float) for v in self.vector), "All vector values must be floats"
        assert self.model == "embed-english-v3.0", f"Unexpected model: {self.model}"
```

---

### 6. QdrantPoint

Represents the final stored vector in Qdrant database.

**Attributes**:

| Field | Type | Required | Description | Validation |
|-------|------|----------|-------------|------------|
| `id` | str | Yes | Unique point ID | MD5 hash from chunk_id |
| `vector` | List[float] | Yes | Embedding vector | Length = 1024 |
| `payload` | dict | Yes | Chunk metadata as JSON | See payload schema |

**Payload Schema**:
```python
{
    "chapter": str,          # e.g., "Chapter 1: ROS 2 Basics"
    "section": str | None,   # e.g., "1.2 Publishers"
    "page_title": str,       # e.g., "Creating Publishers"
    "source_url": str,       # Full HTTPS URL
    "text": str,             # Original chunk text
    "chunk_index": int,      # Position in document
    "created_at": str        # ISO 8601 UTC timestamp
}
```

**Relationships**:
- One QdrantPoint → One Embedding (1:1, transformed from)
- Stored in Qdrant collection `"rag_embedding"`

**Lifecycle**:
1. Created from Embedding + ChunkMetadata
2. Upserted to Qdrant collection
3. Persisted indefinitely (until collection deleted)
4. Retrieved during RAG queries

**Storage Format**:
```python
from qdrant_client.models import PointStruct

def to_qdrant_point(embedding: Embedding) -> PointStruct:
    return PointStruct(
        id=embedding.chunk.metadata.chunk_id,
        vector=embedding.vector,
        payload={
            "chapter": embedding.chunk.metadata.chapter,
            "section": embedding.chunk.metadata.section,
            "page_title": embedding.chunk.metadata.page_title,
            "source_url": embedding.chunk.metadata.source_url,
            "text": embedding.chunk.text,
            "chunk_index": embedding.chunk.chunk_index,
            "created_at": embedding.chunk.metadata.created_at
        }
    )
```

---

## Data Flow Diagram

```
┌─────────────────┐
│   get_all_urls  │
│  (URL Crawling) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ extract_text_   │
│   from_url      │
└────────┬────────┘
         │
         ▼
   DocumentPage
   ┌──────────────┐
   │ url          │
   │ html_content │
   │ clean_text   │
   │ metadata     │
   └──────┬───────┘
          │
          ▼
   ┌──────────────┐
   │  chunk_text  │
   └──────┬───────┘
          │
          ▼
     TextChunk (List)
     ┌──────────────┐
     │ text         │
     │ chunk_index  │
     │ metadata     │
     └──────┬───────┘
            │
            ▼
     ┌──────────────┐
     │    embed     │
     │  (Cohere)    │
     └──────┬───────┘
            │
            ▼
      Embedding (List)
      ┌──────────────┐
      │ chunk        │
      │ vector       │
      └──────┬───────┘
             │
             ▼
      ┌──────────────┐
      │ to_qdrant_   │
      │    point     │
      └──────┬───────┘
             │
             ▼
       QdrantPoint (List)
       ┌──────────────┐
       │ id           │
       │ vector       │
       │ payload      │
       └──────┬───────┘
              │
              ▼
       ┌──────────────┐
       │ Qdrant Cloud │
       │  Collection  │
       │ "rag_embedding"
       └──────────────┘
```

---

## Validation Rules Summary

| Entity | Key Validations |
|--------|----------------|
| **DocumentPage** | URL must be HTTPS, clean_text non-empty, valid PageMetadata |
| **TextChunk** | Text length 10-2000 chars, token_count ≤500, chunk_index ≥0 |
| **Embedding** | Vector length = 1024, all floats finite, model = "embed-english-v3.0" |
| **QdrantPoint** | Valid chunk_id (MD5 format), payload contains all required fields |

---

## State Transitions

### DocumentPage States
1. **Discovered**: URL found during crawling
2. **Fetched**: HTML content retrieved
3. **Extracted**: Clean text and metadata extracted
4. **Chunked**: Split into TextChunks (terminal state)

### TextChunk States
1. **Created**: Generated from DocumentPage
2. **Embedded**: Vector generated via Cohere
3. **Stored**: Upserted to Qdrant (terminal state)

---

## Error Handling

### Invalid Data Scenarios

| Scenario | Detection | Handling |
|----------|-----------|----------|
| Empty extracted text | `len(clean_text) == 0` | Skip page, log warning |
| Oversized chunk | `len(text) > 2000` | Re-chunk with smaller size |
| Invalid embedding dimension | `len(vector) != 1024` | Retry embedding, skip if persistent |
| Missing metadata field | `chapter is None` | Use default "Unknown Chapter" |
| Duplicate chunk_id | Qdrant upsert with same ID | Replace old vector (idempotent) |

---

## Performance Considerations

### Memory Management
- **DocumentPage**: ~50KB per page (HTML + text)
- **TextChunk**: ~2KB per chunk
- **Embedding**: ~4KB per vector (1024 floats)
- **Batch Size**: Process 100 chunks at a time to limit memory usage

### Indexing Strategy
- Qdrant uses HNSW index for fast vector search
- Metadata filtering via payload index
- Expected search time: <500ms for top-5 results

---

## Next Steps

1. ✅ Data model defined
2. → Create `quickstart.md` with setup instructions
3. → Create function contracts in plan.md (already done)
4. → Update agent context with new technologies
5. → Generate `tasks.md` with `/sp.tasks` command

---

**Data Model Status**: ✅ COMPLETE

All entities, attributes, relationships, and validation rules defined. Ready for implementation.
