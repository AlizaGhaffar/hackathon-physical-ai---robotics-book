from typing import List, Dict, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue, SearchRequest
from ..vector_store import get_qdrant_client
from ..ai_client import generate_embedding

def search_similar_content(
    query: str,
    collection_name: str = "chapter_1_embeddings",
    limit: int = 5,
    score_threshold: float = 0.7
) -> List[Dict]:
    """
    Search for similar content in Qdrant using semantic search

    Args:
        query: User's query text
        collection_name: Name of the Qdrant collection
        limit: Maximum number of results to return
        score_threshold: Minimum similarity score (0-1)

    Returns:
        List of relevant content chunks with metadata
    """
    # Generate embedding for the query
    query_embedding = generate_embedding(query)

    # Get Qdrant client
    client = get_qdrant_client()

    # Search for similar vectors (qdrant-client v1.16+)
    try:
        # Use the correct search API
        from qdrant_client.models import ScoredPoint
        search_results = client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=limit,
            score_threshold=score_threshold
        )
    except Exception as e:
        # If search fails, try with minimal parameters
        try:
            search_results = client.search(
                collection_name=collection_name,
                query_vector=query_embedding,
                limit=limit
            )
            # Filter by score manually
            search_results = [r for r in search_results if r.score >= score_threshold]
        except Exception as e2:
            print(f"Qdrant search error: {e2}")
            # Return empty results if both fail
            return []

    # Format results
    results = []
    for result in search_results:
        results.append({
            "content": result.payload.get("content", ""),
            "section": result.payload.get("section", ""),
            "chapter": result.payload.get("chapter", ""),
            "metadata": result.payload.get("metadata", {}),
            "score": result.score
        })

    return results

def build_context_from_results(results: List[Dict], max_tokens: int = 2000) -> str:
    """
    Build a context string from search results

    Args:
        results: List of search results
        max_tokens: Maximum tokens to include (approximate)

    Returns:
        Formatted context string
    """
    if not results:
        return ""

    context_parts = []
    current_length = 0

    for result in results:
        section = result.get("section", "Unknown Section")
        content = result.get("content", "")
        score = result.get("score", 0)

        # Approximate token count (4 chars ≈ 1 token)
        content_tokens = len(content) // 4

        if current_length + content_tokens > max_tokens:
            break

        context_parts.append(f"[{section}] (Relevance: {score:.2f})\n{content}")
        current_length += content_tokens

    return "\n\n---\n\n".join(context_parts)

def rag_query(
    user_query: str,
    chapter: str = "chapter_1",
    max_results: int = 5,
    score_threshold: float = 0.7
) -> Dict[str, any]:
    """
    Perform RAG query: retrieve relevant context and prepare for LLM

    Args:
        user_query: User's question
        chapter: Chapter identifier
        max_results: Maximum search results
        score_threshold: Minimum similarity score

    Returns:
        Dictionary with context and metadata
    """
    collection_name = f"{chapter}_embeddings"

    # Search for relevant content
    results = search_similar_content(
        query=user_query,
        collection_name=collection_name,
        limit=max_results,
        score_threshold=score_threshold
    )

    # Build context
    context = build_context_from_results(results)

    return {
        "context": context,
        "sources": results,
        "num_sources": len(results)
    }

def personalized_rag_query(
    user_query: str,
    user_skill_level: str,
    chapter: str = "chapter_1",
    max_results: int = 5
) -> Dict[str, any]:
    """
    Perform personalized RAG query based on user's skill level

    Args:
        user_query: User's question
        user_skill_level: Beginner, Intermediate, or Advanced
        chapter: Chapter identifier
        max_results: Maximum search results

    Returns:
        Dictionary with context and metadata
    """
    # Adjust score threshold based on skill level
    score_thresholds = {
        "Beginner": 0.65,      # More lenient, broader context
        "Intermediate": 0.70,   # Balanced
        "Advanced": 0.75        # Stricter, more focused
    }

    score_threshold = score_thresholds.get(user_skill_level, 0.70)

    # Adjust max results based on skill level
    results_limit = {
        "Beginner": max_results + 2,      # More context for beginners
        "Intermediate": max_results,       # Standard
        "Advanced": max_results - 1        # Focused context for advanced
    }

    limit = results_limit.get(user_skill_level, max_results)

    return rag_query(
        user_query=user_query,
        chapter=chapter,
        max_results=limit,
        score_threshold=score_threshold
    )
