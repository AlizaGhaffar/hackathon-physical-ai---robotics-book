"""
Retrieval Service
Handles vector search in Qdrant for RAG context retrieval.
"""
import os
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from dotenv import load_dotenv
import sys
from pathlib import Path

# Add parent directory to import utils
sys.path.append(str(Path(__file__).parent.parent))
from utils.embeddings import generate_embedding

load_dotenv()

class RetrievalService:
    """Service for retrieving relevant content from Qdrant."""
    
    def __init__(self):
        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.collection_name = "physical_ai_textbook"
    
    async def retrieve_context(
        self,
        question: str,
        chapter: Optional[int] = None,
        selected_text: Optional[str] = None,
        limit: int = 5,
        score_threshold: float = 0.7
    ) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context chunks for a question.
        
        Args:
            question: User's question
            chapter: Optional chapter filter (1-8)
            selected_text: Optional selected text to prioritize
            limit: Maximum number of chunks to retrieve
            score_threshold: Minimum similarity score
            
        Returns:
            List of retrieved chunks with metadata
        """
        # Generate query embedding
        if selected_text:
            # Prioritize selected text by combining with question
            query_text = f"{selected_text}\n\n{question}"
        else:
            query_text = question
        
        query_embedding = generate_embedding(query_text)
        
        # Build filter for chapter scoping
        query_filter = None
        if chapter:
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="chapter",
                        match=MatchValue(value=chapter)
                    )
                ]
            )
        
        # Search Qdrant
        search_result = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            query_filter=query_filter,
            limit=limit,
            score_threshold=score_threshold
        )
        
        # Format results
        retrieved_chunks = []
        for scored_point in search_result:
            chunk = {
                "chunk_id": scored_point.payload["chunk_id"],
                "chapter": scored_point.payload["chapter"],
                "section": scored_point.payload["section"],
                "content": scored_point.payload["content"],
                "similarity_score": scored_point.score,
                "type": scored_point.payload.get("type", "text"),
                "language": scored_point.payload.get("language")
            }
            retrieved_chunks.append(chunk)
        
        return retrieved_chunks
    
    def health_check(self) -> bool:
        """Check Qdrant connection health."""
        try:
            self.client.get_collections()
            return True
        except Exception:
            return False

# Global retrieval service instance
retrieval_service = RetrievalService()
