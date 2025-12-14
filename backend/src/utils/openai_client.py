"""OpenAI API client wrapper with retry logic."""
from typing import List, Dict, Any, Optional
import asyncio
from openai import AsyncOpenAI, OpenAIError, RateLimitError, APITimeoutError
from src.config import settings
from src.utils.logger import logger


class OpenAIManager:
    """Manages OpenAI client with retry and error handling."""

    def __init__(self):
        """Initialize OpenAI client."""
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.embedding_model = settings.openai_embedding_model
        self.chat_model = settings.openai_model
        self.max_retries = 3
        self.retry_delay = 1  # seconds

    async def health_check(self) -> bool:
        """
        Check OpenAI API connectivity.

        Returns:
            True if API is accessible
        """
        try:
            # Make a minimal API call to test connectivity
            await self.client.embeddings.create(
                model=self.embedding_model,
                input="test",
            )
            return True
        except Exception as e:
            logger.warning(f"OpenAI health check failed: {str(e)}")
            return False

    async def create_embedding(
        self, text: str, retry_count: int = 0
    ) -> Optional[List[float]]:
        """
        Create embedding for a single text.

        Args:
            text: Input text to embed
            retry_count: Current retry attempt

        Returns:
            Embedding vector or None on failure
        """
        try:
            response = await self.client.embeddings.create(
                model=self.embedding_model,
                input=text,
            )
            return response.data[0].embedding

        except RateLimitError as e:
            if retry_count < self.max_retries:
                wait_time = self.retry_delay * (2**retry_count)  # Exponential backoff
                logger.warning(
                    f"Rate limited. Retrying in {wait_time}s (attempt {retry_count + 1}/{self.max_retries})"
                )
                await asyncio.sleep(wait_time)
                return await self.create_embedding(text, retry_count + 1)
            else:
                logger.error(f"Max retries exceeded for embedding: {str(e)}")
                return None

        except APITimeoutError as e:
            if retry_count < self.max_retries:
                logger.warning(f"Timeout. Retrying... (attempt {retry_count + 1}/{self.max_retries})")
                await asyncio.sleep(self.retry_delay)
                return await self.create_embedding(text, retry_count + 1)
            else:
                logger.error(f"Max retries exceeded due to timeout: {str(e)}")
                return None

        except OpenAIError as e:
            logger.error(f"OpenAI API error: {str(e)}")
            return None

    async def create_embeddings_batch(
        self, texts: List[str]
    ) -> List[Optional[List[float]]]:
        """
        Create embeddings for multiple texts in batch.

        Args:
            texts: List of input texts

        Returns:
            List of embedding vectors (may contain None for failures)
        """
        try:
            # OpenAI supports batch embedding
            response = await self.client.embeddings.create(
                model=self.embedding_model,
                input=texts,
            )
            return [item.embedding for item in response.data]

        except OpenAIError as e:
            logger.error(f"Batch embedding failed: {str(e)}")
            # Fallback to individual embeddings
            logger.info("Falling back to individual embeddings...")
            return await asyncio.gather(
                *[self.create_embedding(text) for text in texts]
            )

    async def generate_chat_response(
        self,
        messages: List[Dict[str, str]],
        temperature: float = 0.7,
        max_tokens: Optional[int] = None,
    ) -> Optional[Dict[str, Any]]:
        """
        Generate chat completion response.

        Args:
            messages: List of message dicts with 'role' and 'content'
            temperature: Sampling temperature (0.0-2.0)
            max_tokens: Maximum tokens to generate

        Returns:
            Dict with 'content' and 'usage' or None on failure
        """
        try:
            response = await self.client.chat.completions.create(
                model=self.chat_model,
                messages=messages,
                temperature=temperature,
                max_tokens=max_tokens,
            )

            return {
                "content": response.choices[0].message.content,
                "usage": {
                    "prompt_tokens": response.usage.prompt_tokens,
                    "completion_tokens": response.usage.completion_tokens,
                    "total_tokens": response.usage.total_tokens,
                },
                "finish_reason": response.choices[0].finish_reason,
            }

        except RateLimitError as e:
            logger.error(f"Rate limited during chat generation: {str(e)}")
            return None

        except OpenAIError as e:
            logger.error(f"OpenAI chat error: {str(e)}")
            return None

    async def generate_streaming_response(
        self,
        messages: List[Dict[str, str]],
        temperature: float = 0.7,
    ):
        """
        Generate streaming chat completion.

        Args:
            messages: List of message dicts
            temperature: Sampling temperature

        Yields:
            Content chunks as they arrive
        """
        try:
            stream = await self.client.chat.completions.create(
                model=self.chat_model,
                messages=messages,
                temperature=temperature,
                stream=True,
            )

            async for chunk in stream:
                if chunk.choices[0].delta.content:
                    yield chunk.choices[0].delta.content

        except OpenAIError as e:
            logger.error(f"Streaming error: {str(e)}")
            yield None


# Global instance
openai_manager = OpenAIManager()
