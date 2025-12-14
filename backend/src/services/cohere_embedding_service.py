"""Cohere embedding service for generating vector embeddings."""
from typing import List
import cohere
from src.config import settings
from src.utils.logger import logger


class CohereEmbeddingService:
    """Service for generating embeddings using Cohere API."""

    def __init__(self):
        self.client = cohere.Client(settings.cohere_api_key)
        self.model = "embed-english-v3.0"
        self.input_type = "search_query"  # For queries
        self.embedding_types = ["float"]

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text string.

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector
        """
        try:
            response = self.client.embed(
                texts=[text],
                model=self.model,
                input_type=self.input_type,
                embedding_types=self.embedding_types,
            )
            return response.embeddings.float[0]
        except Exception as e:
            logger.error(f"Failed to embed text with Cohere: {str(e)}")
            raise

    async def embed_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts in batch.

        Args:
            texts: List of text strings to embed

        Returns:
            List of embedding vectors
        """
        if not texts:
            return []

        try:
            logger.info(f"Embedding batch of {len(texts)} texts with Cohere")

            response = self.client.embed(
                texts=texts,
                model=self.model,
                input_type=self.input_type,
                embedding_types=self.embedding_types,
            )

            logger.info(f"Successfully embedded {len(texts)} texts")
            return response.embeddings.float

        except Exception as e:
            logger.error(f"Batch embedding failed: {str(e)}")
            raise

    async def get_embedding_dimension(self) -> int:
        """
        Get the dimension size of the embedding model.

        Returns:
            Integer dimension size (1024 for embed-english-v3.0)
        """
        return 1024


# Global service instance
cohere_embedding_service = CohereEmbeddingService()
