from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from .config import settings

def get_qdrant_client():
    """Get Qdrant client"""
    return QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )

def setup_qdrant_collection():
    """Create Qdrant collection for Chapter 1 embeddings"""
    client = get_qdrant_client()

    collection_name = "chapter_1_embeddings"

    # Check if collection exists
    collections = client.get_collections().collections
    if collection_name not in [c.name for c in collections]:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1536,  # OpenAI text-embedding-ada-002
                distance=Distance.COSINE
            )
        )
        print(f"[SUCCESS] Created Qdrant collection: {collection_name}")
    else:
        print(f"[INFO] Qdrant collection already exists: {collection_name}")

def create_chapter_2_collection():
    """Create Qdrant collection for Chapter 2 embeddings"""
    client = get_qdrant_client()

    collection_name = "chapter_2_embeddings"

    # Check if collection exists
    collections = client.get_collections().collections
    if collection_name not in [c.name for c in collections]:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1536,  # OpenAI text-embedding-ada-002 (same as Chapter 1)
                distance=Distance.COSINE
            )
        )
        print(f"[SUCCESS] Created Qdrant collection: {collection_name}")
    else:
        print(f"[INFO] Qdrant collection already exists: {collection_name}")

if __name__ == "__main__":
    setup_qdrant_collection()
    create_chapter_2_collection()
