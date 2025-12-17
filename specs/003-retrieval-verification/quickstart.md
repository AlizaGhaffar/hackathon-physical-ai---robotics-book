# Quickstart: Retrieval Verification Script

**Feature**: 003-retrieval-verification
**Purpose**: Step-by-step guide to run the verification script and interpret results

---

## Prerequisites

1. **Embedding Pipeline Completed**: Run `backend/embedding-pipeline/main.py` first to populate Qdrant collection
2. **API Keys Configured**: Ensure `.env` file exists with valid credentials
3. **Dependencies Installed**: Run `uv add cohere qdrant-client python-dotenv pytest pytest-asyncio`

---

## Environment Setup

### 1. Configure API Keys

Create or update `.env` file in `backend/embedding-pipeline/`:

```bash
# Cohere API (for embedding generation)
COHERE_API_KEY=your_cohere_api_key_here

# Qdrant Cloud (for vector storage)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Verification Configuration (optional)
VERIFICATION_TOP_K=5              # Number of search results to retrieve
VERIFICATION_MIN_SCORE=0.6        # Minimum similarity threshold
VERIFICATION_TIMEOUT=30           # Max seconds for verification
```

### 2. Verify Collection Exists

```bash
cd backend/embedding-pipeline
uv run python verify.py
```

**Expected Output**:
```
Collection: rag_embedding
Vector count: 42
Vector dimensions: 1024
Distance metric: Cosine

Stored vectors:
1. Vector ID: 12345...
   Chapter: Chapter 1: ROS 2 Basics
   Title: Introduction to ROS 2
   URL: https://physical-ai-robotics-book.vercel.app/chapter1/intro
   Text preview: ROS 2 is the next generation Robot Operating System...
```

If you see errors, run the embedding pipeline first:
```bash
uv run python main.py
```

---

## Running Verification

### Basic Usage

```bash
cd backend/embedding-pipeline
uv run python retrieval_test.py
```

### With Options

```bash
# Verbose output (show all validation details)
uv run python retrieval_test.py --verbose

# JSON output (for CI/CD integration)
uv run python retrieval_test.py --format json

# Custom test queries
uv run python retrieval_test.py --queries "What is ROS 2?" "How to create a publisher?"

# Save report to file
uv run python retrieval_test.py --output verification-report.json
```

---

## Interpreting Results

### Console Output (Human-Readable)

```
============================================================
RETRIEVAL VERIFICATION STARTED
============================================================
Timestamp: 2025-12-11T10:30:00Z

[1/4] Loading collection metadata...
  ✅ Collection exists: rag_embedding
  ✅ Vector count: 42
  ✅ Dimensions: 1024
  ✅ Distance metric: Cosine

[2/4] Running validation checks...

  ✅ [P1-BLOCKER] Collection Exists: PASS
     Details: Collection 'rag_embedding' is accessible

  ✅ [P1-BLOCKER] Non-Empty Collection: PASS
     Details: Collection contains 42 vectors

  ✅ [P1-BLOCKER] Correct Dimensions: PASS
     Details: All 42 vectors have 1024 dimensions

  ✅ [P2-QUALITY] Metadata Completeness: PASS
     Details: 100% of vectors have all required fields

  ✅ [P2-QUALITY] No Duplicates: PASS
     Details: Zero duplicate chunk_ids found

  ✅ [P2-QUALITY] Valid URLs: PASS
     Details: All source_url fields are valid HTTPS URLs

  ✅ [P3-BEST-PRACTICE] Chunk Length Bounds: PASS
     Details: All chunks within 10-2000 character range

[3/4] Running semantic search tests...

  Query 1: "What is ROS 2?"
    [Score: 0.85] Chapter 1: ROS 2 Basics - Introduction to ROS 2
      ROS 2 is the next generation Robot Operating System, designed for production robotics...
      Source: https://physical-ai-robotics-book.vercel.app/chapter1/intro

    [Score: 0.78] Chapter 1: ROS 2 Basics - ROS 2 Architecture
      The ROS 2 architecture is built on DDS middleware, providing real-time communication...
      Source: https://physical-ai-robotics-book.vercel.app/chapter1/architecture

    Status: ✅ PASS (top result score > 0.6)

  Query 2: "How to create a publisher in ROS 2?"
    [Score: 0.82] Chapter 1: ROS 2 Basics - Creating Publishers
      To create a publisher in ROS 2, you first need to import the rclpy library...
      Source: https://physical-ai-robotics-book.vercel.app/chapter1/publishers

    Status: ✅ PASS

[4/4] Generating diagnostic report...

============================================================
VERIFICATION SUMMARY
============================================================
Pipeline Status: PASS
Total Checks: 10
Passed: 10 | Failed: 0 | Warnings: 0

Collection Stats:
  Vectors: 42
  Dimensions: 1024

Search Tests: 5 queries executed
  Average retrieval time: 1.8s
  Average top-1 score: 0.79

Recommendations:
  1. Pipeline is production-ready
  2. Consider adding more test queries for edge cases
  3. Monitor vector count as more content is added

Verification completed in 27.3 seconds.
============================================================
```

### JSON Output (Machine-Readable)

```json
{
  "timestamp": "2025-12-11T10:30:00Z",
  "pipeline_status": "PASS",
  "collection_stats": {
    "name": "rag_embedding",
    "vector_count": 42,
    "dimensions": 1024,
    "distance_metric": "Cosine"
  },
  "validation_results": [
    {
      "check_name": "Collection Exists",
      "check_type": "collection",
      "priority": 1,
      "status": "PASS",
      "details": "Collection 'rag_embedding' is accessible"
    },
    {
      "check_name": "Metadata Completeness",
      "check_type": "metadata",
      "priority": 2,
      "status": "PASS",
      "details": "100% of vectors have all required fields"
    }
  ],
  "search_test_results": [
    {
      "query": "What is ROS 2?",
      "top_results": [
        {
          "score": 0.85,
          "chapter": "Chapter 1: ROS 2 Basics",
          "title": "Introduction to ROS 2",
          "source_url": "https://physical-ai-robotics-book.vercel.app/chapter1/intro"
        }
      ],
      "retrieval_time_ms": 1850,
      "status": "PASS"
    }
  ],
  "summary": {
    "total_checks": 10,
    "passed": 10,
    "failed": 0,
    "warnings": 0,
    "total_vectors_checked": 42,
    "total_search_queries": 5
  },
  "recommendations": [
    "Pipeline is production-ready",
    "Consider adding more test queries for edge cases"
  ]
}
```

---

## Understanding Status Codes

### Pipeline Status

- **PASS** ✅: All checks passed, pipeline is production-ready
- **WARNING** ⚠️: Quality issues detected, but not blocking (e.g., some metadata missing)
- **FAIL** ❌: Blocking issues found (e.g., wrong dimensions, empty collection, duplicates)

### Validation Check Priorities

- **P1 (BLOCKER)**: Must pass for pipeline to be usable
  - Collection Exists
  - Non-Empty Collection
  - Correct Dimensions

- **P2 (QUALITY)**: Should pass for production readiness
  - Metadata Completeness
  - No Duplicates
  - Valid URLs

- **P3 (BEST PRACTICE)**: Nice to have for optimal performance
  - Chunk Length Bounds

---

## Troubleshooting

### Error: "Collection 'rag_embedding' not found"

**Cause**: Embedding pipeline has not been run yet.

**Solution**:
```bash
cd backend/embedding-pipeline
uv run python main.py
```

### Error: "Invalid API key"

**Cause**: Missing or incorrect API keys in `.env` file.

**Solution**:
1. Check `.env` file exists in `backend/embedding-pipeline/`
2. Verify API keys are valid and not expired
3. Ensure no extra spaces around `=` in `.env` file

### Warning: "Low similarity scores for test queries"

**Cause**: Query embeddings may not match document embeddings well.

**Possible reasons**:
- Embedding model mismatch (ensure both use `embed-english-v3.0`)
- Test query too vague or out of scope for book content
- Not enough relevant content in collection

**Solution**:
- Review test queries in `fixtures/sample_queries.json`
- Add more specific queries related to book topics
- Re-run embedding pipeline if model was changed

### Error: "Timeout after 30 seconds"

**Cause**: Verification taking too long (collection too large or API slow).

**Solution**:
- Increase timeout: `uv run python retrieval_test.py --timeout 60`
- Check network connection to Qdrant Cloud
- Review Qdrant Cloud plan limits (free tier may have rate limits)

---

## Running Tests

### Unit Tests

```bash
cd backend/embedding-pipeline
uv run pytest tests/test_validation.py -v
```

**Expected Output**:
```
tests/test_validation.py::test_check_dimensions PASSED
tests/test_validation.py::test_detect_duplicates PASSED
tests/test_validation.py::test_metadata_completeness PASSED
tests/test_validation.py::test_url_validation PASSED
```

### Integration Tests

```bash
uv run pytest tests/test_search.py -v
```

**Expected Output**:
```
tests/test_search.py::test_cohere_embedding PASSED
tests/test_search.py::test_qdrant_search PASSED
tests/test_search.py::test_end_to_end PASSED
```

### Full Test Suite

```bash
uv run pytest tests/ -v --cov=. --cov-report=html
```

---

## CI/CD Integration

### GitHub Actions Example

```yaml
name: Verify Embedding Pipeline

on:
  push:
    branches: [main, 002-embedding-pipeline, 003-retrieval-verification]

jobs:
  verify:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.11'

      - name: Install UV
        run: pip install uv

      - name: Install dependencies
        run: |
          cd backend/embedding-pipeline
          uv sync

      - name: Run verification
        env:
          COHERE_API_KEY: ${{ secrets.COHERE_API_KEY }}
          QDRANT_URL: ${{ secrets.QDRANT_URL }}
          QDRANT_API_KEY: ${{ secrets.QDRANT_API_KEY }}
        run: |
          cd backend/embedding-pipeline
          uv run python retrieval_test.py --format json --output report.json

      - name: Upload report
        if: always()
        uses: actions/upload-artifact@v3
        with:
          name: verification-report
          path: backend/embedding-pipeline/report.json

      - name: Check status
        run: |
          STATUS=$(jq -r '.pipeline_status' backend/embedding-pipeline/report.json)
          if [ "$STATUS" = "FAIL" ]; then
            echo "Pipeline verification failed!"
            exit 1
          fi
```

---

## Next Steps

1. **Production Integration**: Once verification passes, integrate RAG chatbot with frontend
2. **Monitoring**: Set up periodic verification runs to catch embedding drift
3. **Expansion**: Add verification for additional chapters as content grows

For implementation details, see [plan.md](./plan.md) and [data-model.md](./data-model.md).
