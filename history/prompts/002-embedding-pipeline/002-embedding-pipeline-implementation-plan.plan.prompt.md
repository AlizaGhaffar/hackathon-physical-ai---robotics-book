---
id: 002
title: Embedding Pipeline Implementation Plan
stage: plan
date: 2025-12-11
surface: agent
model: claude-sonnet-4-5
feature: 002-embedding-pipeline
branch: 002-embedding-pipeline
user: AlizaGhaffar
command: /sp.plan
labels: ["plan", "embedding-pipeline", "cohere", "qdrant", "architecture", "design"]
links:
  spec: specs/002-embedding-pipeline/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-embedding-pipeline/plan.md
  - specs/002-embedding-pipeline/research.md
  - specs/002-embedding-pipeline/data-model.md
  - specs/002-embedding-pipeline/quickstart.md
  - CLAUDE.md
tests:
  - Constitution Check validation (PASS with documented deviations)
  - Technical decisions documented in research.md
  - Data model entity definitions validated
---

## Prompt

```
/sp.plan initialize backend folder and initialize project with UV package

-Setup Cohere and Qdrant clients

-Fetch, clean, and chunk text from deployed URLs

-Generate embeddings and upsert into Qdrant with metadata

-Only in the one file name main.py system design (get_all_urls, extract_text_from_url, chunk_text,embed,
create_collection name rag_embedding,
save_chunk_to_qdrant and execute in last main function) here is deployed link :
https://physical-ai-robotics-book.vercel.app/
```

## Response snapshot

Created comprehensive implementation plan for Embedding Pipeline Setup (002-embedding-pipeline) including:

### 1. Plan.md - Main Implementation Plan

**Technical Context**:
- Language: Python 3.11+
- Package Manager: UV
- Dependencies: Cohere, Qdrant Client, BeautifulSoup4, httpx, python-dotenv
- Single-file architecture: main.py with 7 core functions

**Constitution Check**: ✅ PASS with 2 documented deviations:
1. Not using FastAPI (justified: batch script, not API service)
2. Using Cohere instead of OpenAI (justified: user request, equivalent quality)

**Function Architecture**:
```
get_all_urls → extract_text_from_url → chunk_text →
embed → create_collection → save_chunk_to_qdrant → main()
```

**Project Structure**: backend/embedding-pipeline/ with main.py, tests/, .env.example

### 2. Research.md - Phase 0 Technical Decisions

**7 Key Decisions Documented**:

1. **Crawling Strategy**: httpx + BeautifulSoup (vs Scrapy, Selenium)
   - Rationale: Lightweight, fast, sufficient for static Docusaurus HTML

2. **Text Extraction**: Custom BeautifulSoup parsing (vs html2text, Newspaper3k)
   - Rationale: Full control over Docusaurus structure, metadata extraction

3. **Chunking Strategy**: Recursive character splitting with overlap (vs fixed, sentence-based)
   - Parameters: 500 tokens, 100 token overlap
   - Preserves sentence boundaries while controlling size

4. **Cohere Model**: embed-english-v3.0 (vs light, multilingual)
   - 1024 dimensions, high quality, English-only

5. **Qdrant Collection**: Single collection with metadata filtering (vs separate collections)
   - Collection name: "rag_embedding"
   - Enables both chapter-scoped and cross-chapter queries

6. **Idempotency**: URL-based vector IDs with MD5 hash (vs check-before-insert)
   - Upsert operation replaces old embeddings automatically

7. **Error Handling**: Exponential backoff retry logic for all API calls
   - Max 3 retries, graceful degradation

**Performance Estimates**:
- Total pages: 100-500
- Total chunks: 1000-5000
- Pipeline time: 40-80 minutes
- Bottleneck: Cohere API rate limits

### 3. Data-Model.md - Entity Definitions

**6 Core Entities**:

1. **DocumentPage**: URL, html_content, clean_text, metadata (PageMetadata), crawled_at
2. **PageMetadata**: chapter, section, page_title, breadcrumbs, heading_hierarchy
3. **TextChunk**: text, chunk_index, metadata (ChunkMetadata), token_count
4. **ChunkMetadata**: chapter, section, page_title, source_url, chunk_id, created_at
5. **Embedding**: chunk (TextChunk), vector (List[float]), model, generated_at
6. **QdrantPoint**: id, vector, payload (metadata dict)

**Data Flow**:
```
DocumentPage → TextChunk (list) → Embedding (list) → QdrantPoint (list) → Qdrant Cloud
```

**Validation Rules**:
- Text length: 10-2000 characters
- Token count: ≤500 tokens
- Vector dimensions: 1024 (Cohere v3)
- Chunk ID: MD5(url + chunk_index)

### 4. Quickstart.md - Setup Guide

**Step-by-Step Instructions**:
1. Install UV package manager
2. Initialize project: `uv init`
3. Install dependencies: `uv add cohere qdrant-client beautifulsoup4 httpx python-dotenv`
4. Configure .env with COHERE_API_KEY, QDRANT_URL, QDRANT_API_KEY
5. Run pipeline: `uv run python main.py`
6. Verify in Qdrant Cloud dashboard

**Environment Variables**:
```bash
COHERE_API_KEY=...
QDRANT_URL=https://xyz.qdrant.io
QDRANT_API_KEY=...
BASE_URL=https://physical-ai-robotics-book.vercel.app/
COLLECTION_NAME=rag_embedding
CHUNK_SIZE=500
CHUNK_OVERLAP=100
```

**Troubleshooting Section**: Common errors and solutions documented

### 5. Agent Context Update (CLAUDE.md)

Added Embedding Pipeline section with:
- Technology stack (UV, Cohere, Qdrant, BeautifulSoup4, httpx)
- Project structure
- Function signatures
- Code style guidelines
- Recent feature summary

## Outcome

- ✅ Impact: Complete implementation plan with technical context, research, data model, and setup guide
- 🧪 Tests: Constitution Check passed with justified deviations (FastAPI N/A, Cohere per user request)
- 📁 Files: Created plan.md, research.md, data-model.md, quickstart.md; updated CLAUDE.md
- 🔁 Next prompts: `/sp.tasks` to generate task breakdown, then implement main.py
- 🧠 Reflection: Single-file architecture (main.py) simplifies batch pipeline. Constitution deviations justified (not an API service). All technical unknowns resolved through research phase.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully
- Graders run and results (PASS/FAIL): Constitution Check - PASS (with documented deviations)
- Prompt variant (if applicable): Standard planning workflow with research → design → quickstart phases
- Next experiment (smallest change to try): Proceed to task generation (`/sp.tasks`) to break down implementation into testable units

## Architectural Decisions

**Significant architectural decisions made** (ADR candidates):

1. **Single-file vs Multi-module Architecture**
   - Decision: Single-file main.py
   - Alternatives: Multi-module with services/ directory
   - Rationale: Batch script simplicity, no runtime API needs, <500 LOC expected
   - Impact: Long-term (affects maintainability, testability)
   - Scope: Embedding pipeline architecture

2. **Cohere vs OpenAI Embeddings**
   - Decision: Cohere embed-english-v3.0
   - Alternatives: OpenAI text-embedding-3-small
   - Rationale: User request, equivalent quality, 1024 dimensions
   - Impact: Long-term (affects vector compatibility, migration)
   - Scope: Embedding service choice

3. **Single Collection vs Separate Collections per Chapter**
   - Decision: Single "rag_embedding" collection with metadata filtering
   - Alternatives: Separate collections (chapter_1, chapter_2, ...)
   - Rationale: Unified search, cross-chapter queries, simpler management
   - Impact: Long-term (affects query patterns, scaling)
   - Scope: Vector storage strategy

📋 **Architectural decision detected**: Cohere embeddings vs OpenAI embeddings — This decision affects vector compatibility and future migration path. Document reasoning and tradeoffs? Run `/sp.adr cohere-vs-openai-embeddings`

(Note: Not creating ADR automatically per constitution guidelines. Waiting for user consent.)
