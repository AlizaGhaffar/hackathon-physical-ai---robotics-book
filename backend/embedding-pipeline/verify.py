"""Verify vectors in Qdrant collection"""
import os
from dotenv import load_dotenv
from qdrant_client import QdrantClient

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Get collection info
collection_info = client.get_collection("rag_embedding")
print(f"Collection: rag_embedding")
print(f"Vector count: {collection_info.points_count}")
print(f"Vector dimensions: {collection_info.config.params.vectors.size}")
print(f"Distance metric: {collection_info.config.params.vectors.distance}")

# Get first few vectors
scroll_result = client.scroll(
    collection_name="rag_embedding",
    limit=5,
    with_payload=True,
    with_vectors=False
)

print(f"\nStored vectors:")
for i, point in enumerate(scroll_result[0], 1):
    print(f"\n{i}. Vector ID: {point.id}")
    if point.payload:
        print(f"   Chapter: {point.payload.get('chapter')}")
        print(f"   Title: {point.payload.get('page_title')}")
        print(f"   URL: {point.payload.get('source_url')}")
        print(f"   Text preview: {point.payload.get('text', '')[:100]}...")
