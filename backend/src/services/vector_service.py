"""Vector service for interacting with Qdrant vector database."""
from typing import List, Dict, Any, Optional
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from src.utils.qdrant_client import qdrant_manager
from src.config import settings
from src.utils.logger import logger


class VectorService:
    """Service for vector storage and retrieval operations."""

    def __init__(self):
        self.collection_name = settings.qdrant_collection_name
        self.top_k = settings.top_k_results
        self.similarity_threshold = settings.similarity_threshold

    async def ensure_collection_exists(self, vector_size: int = 1536):
        """
        Ensure the Qdrant collection exists, create if not.

        Args:
            vector_size: Dimension of embedding vectors
        """
        try:
            client = await qdrant_manager.get_client()
            collections = client.get_collections()

            collection_exists = any(
                col.name == self.collection_name for col in collections.collections
            )

            if not collection_exists:
                logger.info(f"Creating collection: {self.collection_name}")
                client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=vector_size,
                        distance=Distance.COSINE,
                    ),
                )
                logger.info(f"Collection {self.collection_name} created successfully")
            else:
                logger.info(f"Collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Failed to ensure collection exists: {str(e)}")
            raise

    async def upsert_chunks(
        self,
        chunks: List[Dict[str, Any]],
        embeddings: List[List[float]],
    ) -> int:
        """
        Insert or update chunks in Qdrant.

        Args:
            chunks: List of chunk metadata dicts
            embeddings: List of embedding vectors

        Returns:
            Number of chunks successfully upserted
        """
        if len(chunks) != len(embeddings):
            raise ValueError("Number of chunks must match number of embeddings")

        try:
            client = await qdrant_manager.get_client()

            # Ensure collection exists
            await self.ensure_collection_exists(vector_size=len(embeddings[0]))

            # Create points for Qdrant
            points = []
            for idx, (chunk, embedding) in enumerate(zip(chunks, embeddings)):
                point = PointStruct(
                    id=chunk.get("chunk_id", f"{chunk['chapter_id']}_{idx}"),
                    vector=embedding,
                    payload={
                        "chunk_id": chunk["chunk_id"],
                        "chapter_id": chunk["chapter_id"],
                        "source_file": chunk["source_file"],
                        "chunk_index": chunk["chunk_index"],
                        "content": chunk["content"],
                        "metadata": chunk.get("metadata", {}),
                    },
                )
                points.append(point)

            # Upsert to Qdrant
            client.upsert(
                collection_name=self.collection_name,
                points=points,
            )

            logger.info(f"Successfully upserted {len(points)} chunks to Qdrant")
            return len(points)

        except Exception as e:
            logger.error(f"Failed to upsert chunks: {str(e)}")
            raise

    async def search_chunks(
        self,
        query_vector: List[float],
        chapter_id: Optional[str] = None,
        top_k: Optional[int] = None,
        score_threshold: Optional[float] = None,
    ) -> List[Dict[str, Any]]:
        """
        Search for similar chunks in Qdrant.

        Args:
            query_vector: Query embedding vector
            chapter_id: Optional chapter filter
            top_k: Number of results to return
            score_threshold: Minimum similarity score

        Returns:
            List of matching chunks with scores
        """
        top_k = top_k or self.top_k
        score_threshold = score_threshold or self.similarity_threshold

        try:
            client = await qdrant_manager.get_client()

            # Build filter if chapter_id provided
            query_filter = None
            if chapter_id:
                # Try both "chapter_id" and "chapter" fields
                query_filter = Filter(
                    should=[
                        FieldCondition(
                            key="chapter_id",
                            match=MatchValue(value=chapter_id),
                        ),
                        FieldCondition(
                            key="chapter",
                            match=MatchValue(value=chapter_id),
                        )
                    ]
                )

            # Search
            results = client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                query_filter=query_filter,
                limit=top_k,
                score_threshold=score_threshold,
            ).points

            # Format results
            chunks = []
            for result in results:
                chunk = {
                    "chunk_id": result.payload.get("chunk_id", result.id),
                    "chapter_id": result.payload.get("chapter_id", result.payload.get("chapter")),
                    "source_file": result.payload.get("source_file", result.payload.get("source_url")),
                    "chunk_index": result.payload.get("chunk_index", 0),
                    "content": result.payload.get("content", result.payload.get("text")),
                    "score": result.score,
                    "metadata": result.payload.get("metadata", {
                        "page_title": result.payload.get("page_title"),
                        "section": result.payload.get("section"),
                    }),
                }
                chunks.append(chunk)

            logger.info(
                f"Found {len(chunks)} chunks above threshold {score_threshold}"
            )
            return chunks

        except Exception as e:
            logger.error(f"Vector search failed: {str(e)}")
            raise

    async def delete_by_chapter(self, chapter_id: str) -> bool:
        """
        Delete all chunks for a specific chapter.

        Args:
            chapter_id: Chapter identifier

        Returns:
            True if successful
        """
        try:
            client = await qdrant_manager.get_client()

            client.delete(
                collection_name=self.collection_name,
                points_selector=Filter(
                    must=[
                        FieldCondition(
                            key="chapter_id",
                            match=MatchValue(value=chapter_id),
                        )
                    ]
                ),
            )

            logger.info(f"Deleted all chunks for chapter: {chapter_id}")
            return True

        except Exception as e:
            logger.error(f"Failed to delete chapter chunks: {str(e)}")
            raise


# Global service instance
vector_service = VectorService()
