"""Query service for orchestrating the RAG pipeline."""
import time
from typing import Dict, Any, List, Optional
from src.services.cohere_embedding_service import cohere_embedding_service
from src.services.vector_service import vector_service
from src.utils.openai_client import openai_manager
from src.config import settings
from src.utils.logger import logger


class QueryService:
    """Service for processing user queries through RAG pipeline."""

    def __init__(self):
        self.model = settings.openai_model
        self.max_tokens = settings.max_response_tokens
        self.temperature = settings.temperature

    async def process_query(
        self,
        query: str,
        chapter_id: str,
        selected_text: Optional[str] = None,
    ) -> Dict[str, Any]:
        """
        Process a user query through the RAG pipeline.

        Pipeline:
        1. Embed query
        2. Search vector database
        3. Generate answer with GPT

        Args:
            query: User's question
            chapter_id: Chapter to search
            selected_text: Optional selected text for focused query

        Returns:
            Dict with answer, sources, and metadata
        """
        start_time = time.time()

        try:
            # Step 1: Embed the query
            logger.info(f"Processing query: {query[:100]}...")
            query_vector = await cohere_embedding_service.embed_text(query)

            # Step 2: Retrieve relevant chunks
            # TEMPORARILY DISABLED chapter filtering for testing
            chunks = await vector_service.search_chunks(
                query_vector=query_vector,
                chapter_id=None,  # Disabled for testing
            )

            # Handle no results
            if not chunks:
                logger.warning(f"No relevant chunks found for query in {chapter_id}")
                return {
                    "answer": "I couldn't find relevant information about that in the specified chapter. Please try rephrasing your question or check a different chapter.",
                    "sources": [],
                    "token_usage": None,
                    "response_time_ms": (time.time() - start_time) * 1000,
                }

            # Step 3: Build context from chunks
            context = self._build_context(chunks, selected_text)

            # Step 4: Generate answer with GPT
            answer, token_usage = await self._generate_answer(query, context)

            response_time_ms = (time.time() - start_time) * 1000

            logger.info(
                f"Query processed in {response_time_ms:.0f}ms "
                f"with {len(chunks)} sources"
            )

            return {
                "answer": answer,
                "sources": self._format_sources(chunks),
                "token_usage": token_usage,
                "response_time_ms": response_time_ms,
            }

        except Exception as e:
            logger.error(f"Query processing failed: {str(e)}")
            raise

    def _build_context(
        self, chunks: List[Dict[str, Any]], selected_text: Optional[str] = None
    ) -> str:
        """
        Build context string from retrieved chunks.

        Args:
            chunks: Retrieved chunks with scores
            selected_text: Optional selected text to prioritize

        Returns:
            Formatted context string
        """
        context_parts = []

        if selected_text:
            context_parts.append(f"Selected Text:\n{selected_text}\n")

        context_parts.append("Relevant Context from Book:\n")

        for idx, chunk in enumerate(chunks, 1):
            source_info = f"[Source {idx}]"
            if chunk.get("metadata", {}).get("heading"):
                source_info += f" {chunk['metadata']['heading']}"

            context_parts.append(f"{source_info}\n{chunk['content']}\n")

        return "\n".join(context_parts)

    async def _generate_answer(
        self, query: str, context: str
    ) -> tuple[str, Dict[str, int]]:
        """
        Generate answer using GPT with retrieved context.

        Args:
            query: User's question
            context: Retrieved context from vector search

        Returns:
            Tuple of (answer text, token usage dict)
        """
        system_prompt = """You are a helpful AI tutor for a Physical AI and Robotics textbook.
Your role is to answer questions accurately based on the provided book content.

Guidelines:
- Answer ONLY based on the provided context
- If the context doesn't contain the answer, say "I don't have enough information about that in this chapter"
- Be concise but thorough
- Use examples from the context when helpful
- Cite sources using [Source N] notation
- For code-related questions, explain the concepts clearly"""

        user_prompt = f"""Context from the book:
{context}

Question: {query}

Please provide a clear, accurate answer based on the context above. Include source citations where appropriate."""

        try:
            response = await openai_manager.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt},
                ],
                max_tokens=self.max_tokens,
                temperature=self.temperature,
            )

            answer = response.choices[0].message.content

            token_usage = {
                "prompt_tokens": response.usage.prompt_tokens,
                "completion_tokens": response.usage.completion_tokens,
                "total_tokens": response.usage.total_tokens,
            }

            return answer, token_usage

        except Exception as e:
            logger.error(f"Answer generation failed: {str(e)}")
            raise

    def _format_sources(self, chunks: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """
        Format chunks into source citations.

        Args:
            chunks: Retrieved chunks

        Returns:
            List of formatted source dicts
        """
        sources = []

        for chunk in chunks:
            source = {
                "chunk_id": chunk["chunk_id"],
                "content": chunk["content"][:200] + "..."
                if len(chunk["content"]) > 200
                else chunk["content"],
                "score": round(chunk["score"], 3),
                "source_file": chunk["source_file"],
                "chapter_id": chunk["chapter_id"],
            }

            # Add heading if available
            if chunk.get("metadata", {}).get("heading"):
                source["heading"] = chunk["metadata"]["heading"]

            sources.append(source)

        return sources


# Global service instance
query_service = QueryService()
