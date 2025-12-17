"""Final debug - test backend embedding and search"""
import asyncio
import os
from dotenv import load_dotenv
from src.services.cohere_embedding_service import cohere_embedding_service
from src.services.vector_service import vector_service

load_dotenv()

async def main():
    query = "robotics"
    print(f"Query: {query}")

    # Step 1: Embed query using backend service
    print("\n1. Embedding query with backend service...")
    query_vector = await cohere_embedding_service.embed_text(query)
    print(f"   Vector dimension: {len(query_vector)}")
    print(f"   First 5 values: {query_vector[:5]}")

    # Step 2: Search Qdrant with NO chapter filter and LOW threshold
    print("\n2. Searching Qdrant (no filter, threshold=0.1)...")
    results = await vector_service.search_chunks(
        query_vector=query_vector,
        chapter_id=None,
        top_k=5,
        score_threshold=0.1
    )

    print(f"\n   Found {len(results)} results:")
    for idx, result in enumerate(results, 1):
        print(f"\n   [{idx}] Score: {result['score']:.3f}")
        chapter = str(result.get('chapter_id', 'N/A')).encode('ascii', 'ignore').decode()
        print(f"       Chapter: {chapter}")
        content = str(result.get('content', ''))[:80].encode('ascii', 'ignore').decode()
        print(f"       Content: {content}...")

if __name__ == "__main__":
    asyncio.run(main())
