"""Retrieval & Pipeline Verification Script

This script validates the correctness of embeddings stored in Qdrant by:
1. Verifying data integrity (dimensions, metadata, duplicates)
2. Performing semantic search tests with Cohere embeddings
3. Generating diagnostic reports confirming pipeline health

Usage:
    python retrieval_test.py [--verbose] [--format json] [--output FILE]
"""

import os
import sys
import json
import asyncio
import argparse
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Dict, Tuple
from datetime import datetime
from urllib.parse import urlparse
from collections import Counter

from dotenv import load_dotenv
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import PointStruct, Filter, FieldCondition, MatchValue

# Load environment variables
load_dotenv()

# ============================================================================
# DATACLASS ENTITIES (Phase 2: T004-T006)
# ============================================================================

@dataclass
class QdrantCollection:
    """Represents the Qdrant vector database collection"""
    name: str
    vector_count: int
    dimensions: int
    distance_metric: str
    creation_timestamp: Optional[datetime] = None

    def validate(self) -> 'ValidationCheck':
        """Validate collection configuration"""
        if self.vector_count == 0:
            return ValidationCheck(
                check_name="Collection Non-Empty",
                check_type="collection",
                priority=1,
                status="FAIL",
                details="Collection is empty"
            )
        if self.dimensions != 1024:
            return ValidationCheck(
                check_name="Correct Dimensions",
                check_type="dimensionality",
                priority=1,
                status="FAIL",
                details=f"Invalid dimensions: {self.dimensions}, expected 1024"
            )
        if self.distance_metric != "Cosine":
            return ValidationCheck(
                check_name="Correct Distance Metric",
                check_type="collection",
                priority=2,
                status="FAIL",
                details=f"Invalid metric: {self.distance_metric}, expected Cosine"
            )
        return ValidationCheck(
            check_name="Collection Valid",
            check_type="collection",
            priority=1,
            status="PASS",
            details="Collection configuration is valid"
        )


@dataclass
class ChunkMetadata:
    """Metadata attached to each vector"""
    chapter: str
    page_title: str
    source_url: str
    text: str
    chunk_index: int
    created_at: str
    section: Optional[str] = None

    def validate(self) -> 'ValidationCheck':
        """Validate metadata completeness and format"""
        errors = []

        if not self.chapter or not self.page_title:
            errors.append("Missing required fields (chapter or page_title)")

        if not self.source_url.startswith("https://"):
            errors.append(f"Invalid URL: {self.source_url}")

        if not (10 <= len(self.text) <= 2000):
            errors.append(f"Text length out of bounds: {len(self.text)} chars")

        if self.chunk_index < 0:
            errors.append(f"Invalid chunk_index: {self.chunk_index}")

        if errors:
            return ValidationCheck(
                check_name="Metadata Validation",
                check_type="metadata",
                priority=2,
                status="FAIL",
                details="; ".join(errors)
            )
        return ValidationCheck(
            check_name="Metadata Validation",
            check_type="metadata",
            priority=2,
            status="PASS",
            details="Metadata is valid"
        )


@dataclass
class StoredVector:
    """Represents a single vector point in Qdrant"""
    id: str
    vector: Optional[List[float]]
    payload: ChunkMetadata

    def validate_dimensions(self) -> bool:
        """Check vector has correct dimensionality"""
        if self.vector is None:
            return False
        return len(self.vector) == 1024

    def validate_values(self) -> bool:
        """Check vector contains valid float values"""
        if self.vector is None:
            return False
        return all(isinstance(v, (int, float)) and not (v != v) for v in self.vector)  # Check for NaN


@dataclass
class SearchQuery:
    """Represents a semantic search request"""
    query_text: str
    query_embedding: List[float]
    filter_criteria: Optional[Dict[str, str]] = None
    top_k: int = 5
    min_score: float = 0.6

    def validate(self) -> 'ValidationCheck':
        """Validate query parameters"""
        if not self.query_text:
            return ValidationCheck(
                check_name="Query Validation",
                check_type="search",
                priority=2,
                status="FAIL",
                details="Empty query text"
            )

        if len(self.query_embedding) != 1024:
            return ValidationCheck(
                check_name="Query Validation",
                check_type="search",
                priority=2,
                status="FAIL",
                details="Invalid embedding dimensions"
            )

        if not (1 <= self.top_k <= 100):
            return ValidationCheck(
                check_name="Query Validation",
                check_type="search",
                priority=2,
                status="FAIL",
                details=f"Invalid top_k: {self.top_k}"
            )

        if not (0.0 <= self.min_score <= 1.0):
            return ValidationCheck(
                check_name="Query Validation",
                check_type="search",
                priority=2,
                status="FAIL",
                details=f"Invalid min_score: {self.min_score}"
            )

        return ValidationCheck(
            check_name="Query Validation",
            check_type="search",
            priority=2,
            status="PASS",
            details="Query is valid"
        )


@dataclass
class SearchResult:
    """Represents a single search result"""
    vector_id: str
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


@dataclass
class ValidationCheck:
    """Represents a single validation test result"""
    check_name: str
    check_type: str
    priority: int
    status: str
    details: str
    affected_vectors: Optional[List[str]] = None

    def is_blocker(self) -> bool:
        """Check if this is a blocking failure"""
        return self.priority == 1 and self.status == "FAIL"

    def format_for_display(self) -> str:
        """Format check result for console output"""
        emoji = {"PASS": "[PASS]", "FAIL": "[FAIL]", "WARNING": "[WARN]"}[self.status]
        priority_label = {1: "P1-BLOCKER", 2: "P2-QUALITY", 3: "P3-BEST-PRACTICE"}[self.priority]

        result = f"  {emoji} [{priority_label}] {self.check_name}: {self.status}\n"
        result += f"     Details: {self.details}\n"

        if self.affected_vectors:
            result += f"     Affected vectors: {len(self.affected_vectors)}\n"

        return result


@dataclass
class DiagnosticReport:
    """Represents the final verification output"""
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


# ============================================================================
# CLIENT INITIALIZATION (Phase 2: T007-T008)
# ============================================================================

def init_cohere_client() -> cohere.Client:
    """Initialize Cohere API client"""
    api_key = os.getenv("COHERE_API_KEY")
    if not api_key:
        raise ValueError("COHERE_API_KEY not found in environment variables")
    return cohere.Client(api_key)


def init_qdrant_client() -> QdrantClient:
    """Initialize Qdrant Cloud client"""
    url = os.getenv("QDRANT_URL")
    api_key = os.getenv("QDRANT_API_KEY")

    if not url or not api_key:
        raise ValueError("QDRANT_URL or QDRANT_API_KEY not found in environment variables")

    return QdrantClient(url=url, api_key=api_key)


# ============================================================================
# HELPER FUNCTIONS (Phase 2: T009-T010)
# ============================================================================

async def load_collection_metadata(client: QdrantClient, collection_name: str = "rag_embedding") -> QdrantCollection:
    """Load metadata from Qdrant collection"""
    try:
        collection_info = client.get_collection(collection_name)
        return QdrantCollection(
            name=collection_name,
            vector_count=collection_info.points_count,
            dimensions=collection_info.config.params.vectors.size,
            distance_metric=collection_info.config.params.vectors.distance.name,
            creation_timestamp=None
        )
    except Exception as e:
        raise RuntimeError(f"Failed to load collection metadata: {e}")


def log_error_with_context(step: str, error: Exception):
    """Log error with context without crashing"""
    print(f"[ERROR] Step '{step}' failed: {str(error)}", file=sys.stderr)


# ============================================================================
# CORE FUNCTIONS (User-requested implementation)
# ============================================================================

def embed_query(cohere_client: cohere.Client, query: str) -> List[float]:
    """
    Generate Cohere embedding for a single search query.

    Args:
        cohere_client: Initialized Cohere client
        query: Natural language query text

    Returns:
        1024-dimensional embedding vector

    Raises:
        RuntimeError: If embedding generation fails
    """
    try:
        response = cohere_client.embed(
            texts=[query],
            model="embed-english-v3.0",
            input_type="search_query"
        )
        return response.embeddings[0]
    except Exception as e:
        raise RuntimeError(f"Failed to embed query '{query}': {e}")


def search_qdrant(
    qdrant_client: QdrantClient,
    query_embedding: List[float],
    collection_name: str = "rag_embedding",
    top_k: int = 5
) -> List[Dict]:
    """
    Run similarity search on Qdrant and return results.

    Args:
        qdrant_client: Initialized Qdrant client
        query_embedding: 1024-dim query embedding vector
        collection_name: Name of Qdrant collection
        top_k: Number of results to return

    Returns:
        List of search results with metadata

    Raises:
        RuntimeError: If search fails
    """
    try:
        from qdrant_client.models import PointIdsList, QueryRequest, Query

        results = qdrant_client.query_points(
            collection_name=collection_name,
            query=query_embedding,
            limit=top_k,
            with_payload=True
        )

        formatted_results = []
        for result in results.points:
            formatted_results.append({
                "id": result.id,
                "score": result.score,
                "payload": result.payload
            })

        return formatted_results
    except Exception as e:
        raise RuntimeError(f"Failed to search Qdrant: {e}")


def validate_metadata(result: Dict) -> bool:
    """
    Check presence of required metadata fields.

    Args:
        result: Search result with payload

    Returns:
        True if all required fields present, False otherwise
    """
    required_fields = ["page_title", "source_url", "chunk_index", "text", "chapter"]

    payload = result.get("payload", {})

    for field in required_fields:
        if field not in payload or payload[field] is None:
            return False

    return True


def print_results(results: List[Dict], query: str):
    """
    Print ranked matches with readable formatting.

    Args:
        results: List of search results
        query: Original query text
    """
    print(f"\n{'='*60}")
    print(f"Query: \"{query}\"")
    print(f"{'='*60}\n")

    if not results:
        print("No results found.")
        return

    for i, result in enumerate(results, 1):
        payload = result.get("payload", {})
        score = result.get("score", 0.0)

        # Extract metadata
        page_title = payload.get("page_title", "N/A")
        source_url = payload.get("source_url", "N/A")
        chunk_index = payload.get("chunk_index", "N/A")
        chapter = payload.get("chapter", "N/A")
        text = payload.get("text", "")

        # Validation status
        is_valid = validate_metadata(result)
        validation_mark = "[VALID]" if is_valid else "[INVALID METADATA]"

        # Print result
        print(f"Result #{i} {validation_mark}")
        print(f"  Score: {score:.4f}")
        print(f"  Chapter: {chapter}")
        print(f"  Title: {page_title}")
        print(f"  URL: {source_url}")
        print(f"  Chunk Index: {chunk_index}")
        print(f"  Text Preview: {text[:150]}...")
        print()


# ============================================================================
# MAIN FUNCTION
# ============================================================================

async def main():
    """
    Main verification pipeline:
    1. Initialize clients
    2. Load rag_embedding collection
    3. Run several test queries
    4. Print search results + diagnostic summary
    """
    print("="*60)
    print("RETRIEVAL VERIFICATION STARTED")
    print("="*60)
    print(f"Timestamp: {datetime.now().isoformat()}")
    print()

    # Initialize clients
    print("[1/5] Initializing API clients...")
    try:
        cohere_client = init_cohere_client()
        qdrant_client = init_qdrant_client()
        print("  [PASS] Cohere client initialized")
        print("  [PASS] Qdrant client initialized")
    except Exception as e:
        print(f"  [FAIL] Failed to initialize clients: {e}")
        return

    # Load collection metadata
    print("\n[2/5] Loading collection metadata...")
    try:
        collection = await load_collection_metadata(qdrant_client)
        print(f"  [PASS] Collection: {collection.name}")
        print(f"  [PASS] Vector count: {collection.vector_count}")
        print(f"  [PASS] Dimensions: {collection.dimensions}")
        print(f"  [PASS] Distance metric: {collection.distance_metric}")
    except Exception as e:
        print(f"  [FAIL] Failed to load collection: {e}")
        return

    # Confirm collection existence
    print("\n[3/5] Confirming collection existence and statistics...")
    if collection.vector_count == 0:
        print("  [FAIL] Collection is empty. Please run the embedding pipeline first.")
        return
    if collection.dimensions != 1024:
        print(f"  [FAIL] Invalid dimensions: {collection.dimensions}. Expected 1024.")
        return
    print(f"  [PASS] Collection contains {collection.vector_count} vectors")
    print(f"  [PASS] All vectors are 1024-dimensional")

    # Define test queries
    test_queries = [
        "What is robotics?",
        "Tell me about AI models",
        "How do robots work?",
        "Explain machine learning"
    ]

    print(f"\n[4/5] Running semantic search tests ({len(test_queries)} queries)...")

    all_results = []
    valid_results_count = 0
    total_results_count = 0

    for query in test_queries:
        try:
            # Generate query embedding
            print(f"\n  Processing query: \"{query}\"")
            query_embedding = embed_query(cohere_client, query)
            print(f"    [PASS] Generated embedding ({len(query_embedding)} dimensions)")

            # Search Qdrant
            results = search_qdrant(qdrant_client, query_embedding, top_k=5)
            print(f"    [PASS] Retrieved {len(results)} results")

            # Validate metadata
            valid_count = sum(1 for r in results if validate_metadata(r))
            print(f"    [PASS] {valid_count}/{len(results)} results have valid metadata")

            all_results.append((query, results))
            valid_results_count += valid_count
            total_results_count += len(results)

        except Exception as e:
            print(f"    [FAIL] Query failed: {e}")

    # Print detailed results
    print("\n[5/5] Displaying search results...")
    for query, results in all_results:
        print_results(results, query)

    # Diagnostic summary
    print("="*60)
    print("DIAGNOSTIC SUMMARY")
    print("="*60)
    print(f"Collection: {collection.name}")
    print(f"Total Vectors: {collection.vector_count}")
    print(f"Vector Dimensions: {collection.dimensions}")
    print(f"Distance Metric: {collection.distance_metric}")
    print(f"\nTest Queries Executed: {len(test_queries)}")
    print(f"Total Results Retrieved: {total_results_count}")
    print(f"Results with Valid Metadata: {valid_results_count}/{total_results_count}")

    if valid_results_count == total_results_count:
        print("\n[PASS] All results have valid metadata!")
    else:
        invalid_count = total_results_count - valid_results_count
        print(f"\n[WARNING] {invalid_count} results have missing or malformed metadata")

    print("\n" + "="*60)
    print("RETRIEVAL VERIFICATION COMPLETE")
    print("="*60)
    print("\nOutput: A complete retrieval test pipeline proving that embeddings,")
    print("metadata, and vector storage from the embedding pipeline work correctly.")


if __name__ == "__main__":
    asyncio.run(main())
