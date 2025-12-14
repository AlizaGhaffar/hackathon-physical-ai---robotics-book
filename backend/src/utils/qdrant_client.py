"""Qdrant vector database client wrapper."""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition
from src.config import settings
from src.utils.logger import logger


class QdrantManager:
    """Manages Qdrant client connection and operations."""

    def __init__(self):
        """Initialize Qdrant client with settings."""
        self.client: Optional[QdrantClient] = None
        self.collection_name = settings.qdrant_collection_name
        self.vector_size = 1024  # Cohere embed-english-v3.0 dimension

    def connect(self) -> QdrantClient:
        """
        Establish connection to Qdrant Cloud.

        Returns:
            QdrantClient instance

        Raises:
            ConnectionError: If connection fails
        """
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                timeout=30,
            )
            logger.info(f"[OK] Connected to Qdrant at {settings.qdrant_url}")
            return self.client
        except Exception as e:
            logger.error(f"[ERROR] Failed to connect to Qdrant: {str(e)}")
            raise ConnectionError(f"Qdrant connection failed: {str(e)}")

    async def get_client(self) -> QdrantClient:
        """
        Get Qdrant client, connecting if necessary.

        Returns:
            QdrantClient instance
        """
        if not self.client:
            self.connect()
        return self.client

    async def health_check(self) -> bool:
        """
        Check Qdrant service health.

        Returns:
            True if healthy, False otherwise
        """
        try:
            if not self.client:
                self.connect()
            # Try to list collections as health check
            self.client.get_collections()
            return True
        except Exception as e:
            logger.warning(f"Qdrant health check failed: {str(e)}")
            return False

    def create_collection_if_not_exists(self) -> bool:
        """
        Create Qdrant collection if it doesn't exist.

        Returns:
            True if created or already exists
        """
        try:
            if not self.client:
                self.connect()

            # Check if collection exists
            collections = self.client.get_collections().collections
            exists = any(col.name == self.collection_name for col in collections)

            if not exists:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE,
                    ),
                )
                logger.info(f"[OK] Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"[OK] Qdrant collection already exists: {self.collection_name}")

            return True
        except Exception as e:
            logger.error(f"[ERROR] Failed to create collection: {str(e)}")
            return False

    def upsert_chunks(
        self, chunks: List[Dict[str, Any]], vectors: List[List[float]]
    ) -> bool:
        """
        Insert or update document chunks with embeddings.

        Args:
            chunks: List of chunk metadata dicts with 'chunk_id', 'content', etc.
            vectors: Corresponding embedding vectors

        Returns:
            True if successful
        """
        try:
            if not self.client:
                self.connect()

            points = [
                PointStruct(
                    id=chunk["chunk_id"],
                    vector=vector,
                    payload=chunk,
                )
                for chunk, vector in zip(chunks, vectors)
            ]

            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
            )

            logger.info(f"[OK] Upserted {len(points)} chunks to Qdrant")
            return True
        except Exception as e:
            logger.error(f"[ERROR] Failed to upsert chunks: {str(e)}")
            return False

    def search(
        self,
        query_vector: List[float],
        chapter_id: Optional[str] = None,
        limit: int = 5,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks by vector.

        Args:
            query_vector: Query embedding vector
            chapter_id: Optional chapter filter
            limit: Number of results to return

        Returns:
            List of scored chunks with metadata
        """
        try:
            if not self.client:
                self.connect()

            # Build filter if chapter_id provided
            search_filter = None
            if chapter_id:
                search_filter = Filter(
                    must=[
                        FieldCondition(
                            key="chapter_id",
                            match={"value": chapter_id},
                        )
                    ]
                )

            results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                query_filter=search_filter,
                limit=limit,
            )

            # Format results
            chunks = [
                {
                    "chunk_id": hit.id,
                    "score": hit.score,
                    **hit.payload,
                }
                for hit in results
            ]

            logger.info(f"[OK] Found {len(chunks)} similar chunks")
            return chunks

        except Exception as e:
            logger.error(f"[ERROR] Qdrant search failed: {str(e)}")
            return []


# Global instance
qdrant_manager = QdrantManager()
