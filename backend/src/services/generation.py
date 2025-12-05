"""
Generation Service
Handles GPT-4 answer generation with streaming support and retry logic.
"""
import os
from typing import List, Dict, Any, Optional, AsyncIterator
from openai import OpenAI, AsyncOpenAI
from tenacity import retry, stop_after_attempt, wait_exponential
from dotenv import load_dotenv
import sys
from pathlib import Path

# Add parent directory to import utils
sys.path.append(str(Path(__file__).parent.parent))
from utils.prompts import format_rag_prompt, format_selected_text_prompt

load_dotenv()

class GenerationService:
    """Service for generating answers using GPT-4."""
    
    def __init__(self):
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.async_client = AsyncOpenAI(api_key=os.getenv("OPENAI_API_KEY"))
        self.model = "gpt-4-turbo-preview"
        self.temperature = 0.3
        self.max_tokens = 500
    
    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=2, max=10)
    )
    async def generate_answer(
        self,
        question: str,
        retrieved_chunks: List[Dict[str, Any]],
        selected_text: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Generate answer using GPT-4 with retrieved context.
        
        Args:
            question: User's question
            retrieved_chunks: List of retrieved context chunks
            selected_text: Optional selected text for focused answers
            
        Returns:
            Dictionary with answer and extracted citations
        """
        # Format context from chunks
        context_texts = [chunk['content'] for chunk in retrieved_chunks]
        
        # Create system prompt
        if selected_text:
            system_prompt = format_selected_text_prompt(
                question, selected_text, context_texts
            )
        else:
            system_prompt = format_rag_prompt(question, context_texts)
        
        # Call GPT-4
        response = await self.async_client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt}
            ],
            temperature=self.temperature,
            max_tokens=self.max_tokens
        )
        
        answer = response.choices[0].message.content
        
        # Extract citations from retrieved chunks
        citations = [
            {
                "chapter": chunk['chapter'],
                "section": chunk['section'],
                "chunk_id": chunk['chunk_id'],
                "similarity_score": chunk['similarity_score']
            }
            for chunk in retrieved_chunks
        ]
        
        return {
            "answer": answer,
            "citations": citations
        }
    
    @retry(
        stop=stop_after_attempt(3),
        wait=wait_exponential(multiplier=1, min=2, max=10)
    )
    async def stream_answer(
        self,
        question: str,
        retrieved_chunks: List[Dict[str, Any]],
        selected_text: Optional[str] = None
    ) -> AsyncIterator[str]:
        """
        Generate answer with streaming (yields tokens progressively).
        
        Args:
            question: User's question
            retrieved_chunks: List of retrieved context chunks
            selected_text: Optional selected text for focused answers
            
        Yields:
            Answer tokens as they are generated
        """
        # Format context from chunks
        context_texts = [chunk['content'] for chunk in retrieved_chunks]
        
        # Create system prompt
        if selected_text:
            system_prompt = format_selected_text_prompt(
                question, selected_text, context_texts
            )
        else:
            system_prompt = format_rag_prompt(question, context_texts)
        
        # Stream GPT-4 response
        stream = await self.async_client.chat.completions.create(
            model=self.model,
            messages=[
                {"role": "system", "content": system_prompt}
            ],
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            stream=True
        )
        
        async for chunk in stream:
            if chunk.choices[0].delta.content:
                yield chunk.choices[0].delta.content
    
    def health_check(self) -> bool:
        """Check OpenAI API connectivity."""
        try:
            self.client.models.list()
            return True
        except Exception:
            return False

# Global generation service instance
generation_service = GenerationService()
