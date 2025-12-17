"""Embedding service for generating vector embeddings using OpenAI."""
from typing import List, Dict, Any
import asyncio
from src.utils.openai_client import openai_manager
from src.config import settings
from src.utils.logger import logger


class EmbeddingService:
    """Service for generating embeddings from text."""

    def __init__(self):
        self.model = settings.openai_embedding_model
        self.max_batch_size = 100  # OpenAI allows up to 100 inputs per request

    async def embed_text(self, text: str) -> List[float]:
        """
        Generate embedding for a single text string.

        Args:
            text: Input text to embed

        Returns:
            List of floats representing the embedding vector
        """
        try:
            embeddings = await self.embed_batch([text])
            return embeddings[0]
        except Exception as e:
            logger.error(f"Failed to embed text: {str(e)}")
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
            # Process in batches if needed
            all_embeddings = []

            for i in range(0, len(texts), self.max_batch_size):
                batch = texts[i : i + self.max_batch_size]
                logger.info(
                    f"Embedding batch {i // self.max_batch_size + 1}: "
                    f"{len(batch)} texts"
                )

                response = await openai_manager.client.embeddings.create(
                    model=self.model,
                    input=batch,
                )

                # Extract embeddings from response
                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

                # Small delay to avoid rate limiting
                if i + self.max_batch_size < len(texts):
                    await asyncio.sleep(0.1)

            logger.info(f"Successfully embedded {len(texts)} texts")
            return all_embeddings

        except Exception as e:
            logger.error(f"Batch embedding failed: {str(e)}")
            raise

    async def get_embedding_dimension(self) -> int:
        """
        Get the dimension size of the embedding model.

        Returns:
            Integer dimension size
        """
        # text-embedding-3-small produces 1536-dimensional vectors
        if "text-embedding-3-small" in self.model:
            return 1536
        elif "text-embedding-3-large" in self.model:
            return 3072
        elif "text-embedding-ada-002" in self.model:
            return 1536
        else:
            # Fallback: generate a test embedding to determine dimension
            test_embedding = await self.embed_text("test")
            return len(test_embedding)


# Global service instance
embedding_service = EmbeddingService()
