"""Debug script to test Qdrant search"""
import asyncio
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient
import cohere

load_dotenv()

async def main():
    # Initialize clients
    qdrant = QdrantClient(
        url=os.getenv('QDRANT_URL'),
        api_key=os.getenv('QDRANT_API_KEY')
    )
    co = cohere.Client(os.getenv('COHERE_API_KEY'))

    collection_name = os.getenv('QDRANT_COLLECTION_NAME', 'book_embeddings')

    # Check collection
    print(f"Collection: {collection_name}")
    info = qdrant.get_collection(collection_name)
    print(f"Total points: {info.points_count}\n")

    # Get all points
    points = qdrant.scroll(
        collection_name=collection_name,
        limit=10,
        with_payload=True,
        with_vectors=False
    )

    print("=== ALL VECTORS IN DATABASE ===")
    for idx, point in enumerate(points[0], 1):
        print(f"\n[{idx}] ID: {str(point.id)}")
        payload = point.payload
        # Safe print - remove non-ASCII
        chapter = str(payload.get('chapter', '')).encode('ascii', 'ignore').decode()
        section = str(payload.get('section', '')).encode('ascii', 'ignore').decode()
        title = str(payload.get('page_title', '')).encode('ascii', 'ignore').decode()
        text = str(payload.get('text', ''))[:150].encode('ascii', 'ignore').decode()
        print(f"  chapter: {chapter}")
        print(f"  section: {section}")
        print(f"  page_title: {title}")
        print(f"  text: {text}...")

    # Try search without filter
    print("\n\n=== TESTING SEARCH (NO FILTER) ===")
    query = "What is in chapter 1?"
    print(f"Query: {query}")

    # Embed query
    response = co.embed(
        texts=[query],
        model='embed-english-v3.0',
        input_type='search_query'
    )
    query_vector = response.embeddings[0]

    # Search
    results = qdrant.query_points(
        collection_name=collection_name,
        query=query_vector,
        limit=3,
    ).points

    print(f"\nFound {len(results)} results:")
    for idx, result in enumerate(results, 1):
        print(f"\n[{idx}] Score: {result.score:.3f}")
        chapter = str(result.payload.get('chapter', '')).encode('ascii', 'ignore').decode()
        title = str(result.payload.get('page_title', '')).encode('ascii', 'ignore').decode()
        text = str(result.payload.get('text', ''))[:100].encode('ascii', 'ignore').decode()
        print(f"  chapter: {chapter}")
        print(f"  page_title: {title}")
        print(f"  text: {text}...")

if __name__ == "__main__":
    asyncio.run(main())
