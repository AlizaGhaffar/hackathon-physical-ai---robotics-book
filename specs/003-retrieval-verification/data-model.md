# Data Model: Embedding Retrieval & Pipeline Verification

**Feature**: 003-retrieval-verification
**Date**: 2025-12-11
**Purpose**: Define entities, relationships, and data structures for verification testing

## Entity Definitions

### 1. QdrantCollection

Represents the Qdrant vector database collection being verified.

**Attributes**:
- `name: str` - Collection name (e.g., "rag_embedding")
- `vector_count: int` - Total number of vectors stored
- `dimensions: int` - Vector dimensionality (expected: 1024)
- `distance_metric: str` - Similarity metric (expected: "Cosine")
- `creation_timestamp: datetime` - When collection was created

**Relationships**:
- Has many `StoredVector` entries

**Validation Rules**:
- `name` must match expected collection name
- `vector_count` must be > 0
- `dimensions` must equal 1024 (Cohere embed-english-v3.0)
- `distance_metric` must be "Cosine"

**Python Representation**:
```python
@dataclass
class QdrantCollection:
    name: str
    vector_count: int
    dimensions: int
    distance_metric: str
    creation_timestamp: datetime

    def validate(self) -> ValidationResult:
        """Validate collection configuration"""
        if self.vector_count == 0:
            return ValidationResult(status="FAIL", details="Collection is empty")
        if self.dimensions != 1024:
            return ValidationResult(status="FAIL", details=f"Invalid dimensions: {self.dimensions}")
        if self.distance_metric != "Cosine":
            return ValidationResult(status="FAIL", details=f"Invalid metric: {self.distance_metric}")
        return ValidationResult(status="PASS", details="Collection valid")
```

---

### 2. StoredVector

Represents a single vector point stored in Qdrant with associated metadata.

**Attributes**:
- `id: UUID` - Unique vector identifier
- `vector: List[float]` - 1024-dimensional embedding
- `payload: ChunkMetadata` - Associated metadata

**Relationships**:
- Belongs to `QdrantCollection`
- Has one `ChunkMetadata` payload

**Validation Rules**:
- `vector` length must be exactly 1024
- `vector` must contain only finite float values (no NaN, Inf)
- `payload` must have all required fields

**Python Representation**:
```python
@dataclass
class StoredVector:
    id: UUID
    vector: List[float]
    payload: 'ChunkMetadata'

    def validate_dimensions(self) -> bool:
        """Check vector has correct dimensionality"""
        return len(self.vector) == 1024

    def validate_values(self) -> bool:
        """Check vector contains valid float values"""
        return all(math.isfinite(v) for v in self.vector)

    def validate_payload(self) -> ValidationResult:
        """Check payload completeness"""
        return self.payload.validate()
```

---

### 3. ChunkMetadata

Represents metadata attached to each vector, describing the source document chunk.

**Attributes**:
- `chapter: str` - Chapter name (e.g., "Chapter 1: ROS 2 Basics")
- `section: Optional[str]` - Section name within chapter
- `page_title: str` - Page or heading title
- `source_url: str` - Original documentation URL
- `text: str` - Full text content of the chunk
- `chunk_index: int` - Index of chunk within source document
- `created_at: datetime` - Timestamp when chunk was embedded

**Relationships**:
- Belongs to `StoredVector`

**Validation Rules**:
- `chapter` must not be null or empty
- `page_title` must not be null or empty
- `source_url` must be valid HTTPS URL
- `text` length must be 10-2000 characters
- `chunk_index` must be >= 0
- `created_at` must be valid ISO 8601 datetime

**Python Representation**:
```python
@dataclass
class ChunkMetadata:
    chapter: str
    section: Optional[str]
    page_title: str
    source_url: str
    text: str
    chunk_index: int
    created_at: datetime

    def validate(self) -> ValidationResult:
        """Validate metadata completeness and format"""
        errors = []

        if not self.chapter or not self.page_title:
            errors.append("Missing required fields")

        if not self.source_url.startswith("https://"):
            errors.append(f"Invalid URL: {self.source_url}")

        if not (10 <= len(self.text) <= 2000):
            errors.append(f"Text length out of bounds: {len(self.text)}")

        if self.chunk_index < 0:
            errors.append(f"Invalid chunk_index: {self.chunk_index}")

        if errors:
            return ValidationResult(status="FAIL", details="; ".join(errors))
        return ValidationResult(status="PASS", details="Metadata valid")
```

---

### 4. SearchQuery

Represents a semantic search request to validate retrieval quality.

**Attributes**:
- `query_text: str` - Natural language query
- `query_embedding: List[float]` - 1024-dim embedding of query (generated via Cohere)
- `filter_criteria: Optional[Dict]` - Metadata filters (e.g., {"chapter": "Chapter 1"})
- `top_k: int` - Number of results to retrieve (default: 5)
- `min_score: float` - Minimum similarity score threshold (default: 0.6)

**Relationships**:
- Produces multiple `SearchResult` entries

**Validation Rules**:
- `query_text` must not be empty
- `query_embedding` must be 1024-dimensional
- `top_k` must be between 1 and 100
- `min_score` must be between 0.0 and 1.0

**Python Representation**:
```python
@dataclass
class SearchQuery:
    query_text: str
    query_embedding: List[float]
    filter_criteria: Optional[Dict[str, str]] = None
    top_k: int = 5
    min_score: float = 0.6

    def validate(self) -> ValidationResult:
        """Validate query parameters"""
        if not self.query_text:
            return ValidationResult(status="FAIL", details="Empty query text")

        if len(self.query_embedding) != 1024:
            return ValidationResult(status="FAIL", details="Invalid embedding dimensions")

        if not (1 <= self.top_k <= 100):
            return ValidationResult(status="FAIL", details=f"Invalid top_k: {self.top_k}")

        if not (0.0 <= self.min_score <= 1.0):
            return ValidationResult(status="FAIL", details=f"Invalid min_score: {self.min_score}")

        return ValidationResult(status="PASS", details="Query valid")
```

---

### 5. SearchResult

Represents a single result from semantic search, with score and metadata.

**Attributes**:
- `vector_id: UUID` - ID of matched vector
- `similarity_score: float` - Cosine similarity (0.0-1.0)
- `metadata: ChunkMetadata` - Metadata of matched chunk
- `retrieval_time_ms: float` - Time taken for retrieval

**Relationships**:
- Belongs to `SearchQuery`
- References `StoredVector` via `vector_id`

**Validation Rules**:
- `similarity_score` must be between 0.0 and 1.0
- `retrieval_time_ms` must be positive
- `metadata` must be valid `ChunkMetadata`

**Python Representation**:
```python
@dataclass
class SearchResult:
    vector_id: UUID
    similarity_score: float
    metadata: ChunkMetadata
    retrieval_time_ms: float

    def is_relevant(self, threshold: float = 0.6) -> bool:
        """Check if result meets relevance threshold"""
        return self.similarity_score >= threshold

    def format_for_display(self) -> str:
        """Format result for console output"""
        return (
            f"[Score: {self.similarity_score:.2f}] "
            f"{self.metadata.chapter} - {self.metadata.page_title}\n"
            f"  {self.metadata.text[:100]}...\n"
            f"  Source: {self.metadata.source_url}"
        )
```

---

### 6. ValidationCheck

Represents a single validation test with pass/fail status.

**Attributes**:
- `check_name: str` - Descriptive name (e.g., "Metadata Completeness")
- `check_type: str` - Category (e.g., "dimensionality", "duplicates", "metadata")
- `priority: int` - Priority level (1=blocker, 2=quality, 3=best-practice)
- `status: str` - Result status ("PASS", "FAIL", "WARNING")
- `details: str` - Human-readable explanation of result
- `affected_vectors: Optional[List[UUID]]` - IDs of vectors that failed check

**Relationships**:
- Belongs to `DiagnosticReport`

**Validation Rules**:
- `check_name` must not be empty
- `priority` must be 1, 2, or 3
- `status` must be one of ["PASS", "FAIL", "WARNING"]

**Python Representation**:
```python
@dataclass
class ValidationCheck:
    check_name: str
    check_type: str
    priority: int
    status: str
    details: str
    affected_vectors: Optional[List[UUID]] = None

    def is_blocker(self) -> bool:
        """Check if this is a blocking failure"""
        return self.priority == 1 and self.status == "FAIL"

    def format_for_display(self) -> str:
        """Format check result for console output"""
        emoji = {"PASS": "✅", "FAIL": "❌", "WARNING": "⚠️"}[self.status]
        priority_label = {1: "P1-BLOCKER", 2: "P2-QUALITY", 3: "P3-BEST-PRACTICE"}[self.priority]

        result = f"{emoji} [{priority_label}] {self.check_name}: {self.status}\n"
        result += f"   Details: {self.details}\n"

        if self.affected_vectors:
            result += f"   Affected vectors: {len(self.affected_vectors)}\n"

        return result
```

---

### 7. DiagnosticReport

Represents the final output of the verification process with all results.

**Attributes**:
- `timestamp: datetime` - When verification was run
- `pipeline_status: str` - Overall status ("PASS", "FAIL", "WARNING")
- `collection_stats: QdrantCollection` - Collection metadata
- `validation_results: List[ValidationCheck]` - All validation check results
- `search_test_results: List[Tuple[SearchQuery, List[SearchResult]]]` - Semantic search test results
- `recommendations: List[str]` - Actionable next steps

**Relationships**:
- Has one `QdrantCollection`
- Has many `ValidationCheck` entries
- Has many `SearchQuery` + `SearchResult` pairs

**Validation Rules**:
- `pipeline_status` must be determined by validation results
- If any P1 check fails → pipeline_status = "FAIL"
- If all P1 pass but P2 fails → pipeline_status = "WARNING"
- If all checks pass → pipeline_status = "PASS"

**Python Representation**:
```python
@dataclass
class DiagnosticReport:
    timestamp: datetime
    pipeline_status: str
    collection_stats: QdrantCollection
    validation_results: List[ValidationCheck]
    search_test_results: List[Tuple[SearchQuery, List[SearchResult]]]
    recommendations: List[str]

    def determine_status(self) -> str:
        """Compute overall pipeline status from validation results"""
        has_blocker = any(check.is_blocker() for check in self.validation_results)
        if has_blocker:
            return "FAIL"

        has_quality_issue = any(
            check.priority == 2 and check.status == "FAIL"
            for check in self.validation_results
        )
        if has_quality_issue:
            return "WARNING"

        return "PASS"

    def format_summary(self) -> str:
        """Format summary section for console output"""
        total = len(self.validation_results)
        passed = sum(1 for c in self.validation_results if c.status == "PASS")
        failed = sum(1 for c in self.validation_results if c.status == "FAIL")
        warnings = sum(1 for c in self.validation_results if c.status == "WARNING")

        summary = f"\n{'='*60}\n"
        summary += f"VERIFICATION SUMMARY\n"
        summary += f"{'='*60}\n"
        summary += f"Pipeline Status: {self.pipeline_status}\n"
        summary += f"Total Checks: {total}\n"
        summary += f"Passed: {passed} | Failed: {failed} | Warnings: {warnings}\n"
        summary += f"\nCollection Stats:\n"
        summary += f"  Vectors: {self.collection_stats.vector_count}\n"
        summary += f"  Dimensions: {self.collection_stats.dimensions}\n"
        summary += f"\nSearch Tests: {len(self.search_test_results)} queries executed\n"

        if self.recommendations:
            summary += f"\nRecommendations:\n"
            for i, rec in enumerate(self.recommendations, 1):
                summary += f"  {i}. {rec}\n"

        return summary

    def to_json(self) -> Dict:
        """Export report as JSON for machine parsing"""
        return {
            "timestamp": self.timestamp.isoformat(),
            "pipeline_status": self.pipeline_status,
            "collection_stats": {
                "name": self.collection_stats.name,
                "vector_count": self.collection_stats.vector_count,
                "dimensions": self.collection_stats.dimensions,
                "distance_metric": self.collection_stats.distance_metric,
            },
            "validation_results": [
                {
                    "check_name": check.check_name,
                    "priority": check.priority,
                    "status": check.status,
                    "details": check.details,
                }
                for check in self.validation_results
            ],
            "search_test_results": [
                {
                    "query": query.query_text,
                    "top_results": [
                        {
                            "score": result.similarity_score,
                            "chapter": result.metadata.chapter,
                            "title": result.metadata.page_title,
                        }
                        for result in results[:3]
                    ],
                }
                for query, results in self.search_test_results
            ],
            "summary": {
                "total_checks": len(self.validation_results),
                "passed": sum(1 for c in self.validation_results if c.status == "PASS"),
                "failed": sum(1 for c in self.validation_results if c.status == "FAIL"),
                "warnings": sum(1 for c in self.validation_results if c.status == "WARNING"),
            },
            "recommendations": self.recommendations,
        }
```

---

## Entity Relationships

```text
QdrantCollection (1) ──────< (N) StoredVector
                                     │
                                     │ has payload
                                     ▼
                                ChunkMetadata

SearchQuery (1) ──────< (N) SearchResult
                                 │
                                 │ references
                                 ▼
                            StoredVector

DiagnosticReport (1) ──────< (N) ValidationCheck
                 │
                 └──────< (N) (SearchQuery, SearchResult[])
                 │
                 └──────> (1) QdrantCollection
```

---

## State Transitions

### ValidationCheck State Machine

```text
PENDING ──[run check]──> PASS
                     └──> FAIL
                     └──> WARNING
```

No state transitions after completion (immutable result).

### DiagnosticReport Status Determination

```text
All checks run
    │
    ├──[Any P1 FAIL]──────────> FAIL
    │
    ├──[All P1 PASS]
    │   └──[Any P2 FAIL]──────> WARNING
    │
    └──[All checks PASS]──────> PASS
```

---

## Data Flow

```text
1. Load QdrantCollection metadata
   ↓
2. Retrieve all StoredVector entries
   ↓
3. Run ValidationCheck for each validation rule
   ↓
4. Generate SearchQuery embeddings
   ↓
5. Execute semantic search → SearchResult[]
   ↓
6. Aggregate all results into DiagnosticReport
   ↓
7. Output to console (formatted) and JSON (structured)
```

---

## Summary

This data model defines:
- **7 core entities** with clear relationships
- **Validation rules** for each entity
- **State transitions** for verification workflow
- **Data flow** from collection to diagnostic report

All entities are designed as Python dataclasses with type hints for type safety and clarity. The model supports both human-readable console output and machine-readable JSON export for CI/CD integration.
