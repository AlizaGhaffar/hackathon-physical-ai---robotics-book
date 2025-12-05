"""
OpenAI Embedding Generation Utility
Generates embeddings using text-embedding-3-small model.
"""
import os
from typing import List
from openai import OpenAI
from dotenv import load_dotenv

load_dotenv()

client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding for a single text using OpenAI text-embedding-3-small.
    
    Args:
        text: Text to embed (max 8191 tokens)
        
    Returns:
        List of 1536 floats representing the embedding vector
        
    Raises:
        ValueError: If text is empty
        Exception: If OpenAI API call fails
    """
    if not text or not text.strip():
        raise ValueError("Text cannot be empty")
    
    try:
        response = client.embeddings.create(
            model="text-embedding-3-small",
            input=text.strip()
        )
        return response.data[0].embedding
    except Exception as e:
        raise Exception(f"Failed to generate embedding: {e}")

def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in a single API call.
    
    Args:
        texts: List of texts to embed (max 100 texts per batch)
        
    Returns:
        List of embedding vectors corresponding to input texts
        
    Raises:
        ValueError: If texts is empty or exceeds batch size
        Exception: If OpenAI API call fails
    """
    if not texts:
        raise ValueError("Texts list cannot be empty")
    
    if len(texts) > 100:
        raise ValueError("Batch size cannot exceed 100 texts")
    
    # Filter out empty texts
    valid_texts = [t.strip() for t in texts if t and t.strip()]
    
    if not valid_texts:
        raise ValueError("No valid texts to embed")
    
    try:
        response = client.embeddings.create(
            model="text-embedding-3-small",
            input=valid_texts
        )
        return [data.embedding for data in response.data]
    except Exception as e:
        raise Exception(f"Failed to generate batch embeddings: {e}")
