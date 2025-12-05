"""
Database Service
Handles all PostgreSQL database operations using asyncpg.
"""
import os
import asyncpg
from typing import Optional, List, Dict, Any
from uuid import UUID
from datetime import datetime
from dotenv import load_dotenv

load_dotenv()

class DatabaseService:
    """Async database service for Neon Postgres."""
    
    def __init__(self):
        self.connection_string = os.getenv("NEON_CONNECTION_STRING")
        self.pool: Optional[asyncpg.Pool] = None
    
    async def connect(self):
        """Create connection pool."""
        if not self.pool:
            self.pool = await asyncpg.create_pool(
                self.connection_string,
                min_size=1,
                max_size=10
            )
    
    async def disconnect(self):
        """Close connection pool."""
        if self.pool:
            await self.pool.close()
    
    async def create_conversation(
        self,
        user_id: Optional[UUID],
        chapter: int,
        session_metadata: Optional[Dict[str, Any]] = None
    ) -> UUID:
        """
        Create a new conversation.
        
        Args:
            user_id: User UUID (can be None for anonymous)
            chapter: Chapter number (1-8)
            session_metadata: Optional metadata dictionary
            
        Returns:
            UUID of created conversation
        """
        await self.connect()
        
        query = """
            INSERT INTO conversations (user_id, chapter, session_metadata)
            VALUES ($1, $2, $3)
            RETURNING id
        """
        
        metadata = session_metadata or {}
        
        async with self.pool.acquire() as conn:
            conversation_id = await conn.fetchval(
                query, user_id, chapter, metadata
            )
        
        return conversation_id
    
    async def create_message(
        self,
        conversation_id: UUID,
        role: str,
        content: str,
        citations: Optional[List[Dict[str, Any]]] = None,
        selected_text: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None
    ) -> UUID:
        """
        Create a new message in a conversation.
        
        Args:
            conversation_id: Conversation UUID
            role: 'user' or 'assistant'
            content: Message content
            citations: List of citations (for assistant messages)
            selected_text: User-highlighted text (for user messages)
            metadata: Performance metrics
            
        Returns:
            UUID of created message
        """
        await self.connect()
        
        query = """
            INSERT INTO messages (conversation_id, role, content, citations, selected_text, metadata)
            VALUES ($1, $2, $3, $4, $5, $6)
            RETURNING id
        """
        
        async with self.pool.acquire() as conn:
            message_id = await conn.fetchval(
                query,
                conversation_id,
                role,
                content,
                citations or [],
                selected_text,
                metadata or {}
            )
        
        return message_id
    
    async def get_conversation_history(self, conversation_id: UUID) -> Optional[Dict[str, Any]]:
        """
        Retrieve conversation with all messages.
        
        Args:
            conversation_id: Conversation UUID
            
        Returns:
            Dictionary with conversation data and messages
        """
        await self.connect()
        
        conversation_query = """
            SELECT id, chapter, created_at
            FROM conversations
            WHERE id = $1
        """
        
        messages_query = """
            SELECT id, role, content, citations, selected_text, created_at
            FROM messages
            WHERE conversation_id = $1
            ORDER BY created_at ASC
        """
        
        async with self.pool.acquire() as conn:
            conversation = await conn.fetchrow(conversation_query, conversation_id)
            
            if not conversation:
                return None
            
            messages = await conn.fetch(messages_query, conversation_id)
            
            return {
                "conversation_id": conversation['id'],
                "chapter": conversation['chapter'],
                "messages": [dict(msg) for msg in messages]
            }
    
    async def delete_conversation(self, conversation_id: UUID) -> bool:
        """
        Delete a conversation and all its messages (cascade).
        
        Args:
            conversation_id: Conversation UUID
            
        Returns:
            True if deleted, False if not found
        """
        await self.connect()
        
        query = """
            DELETE FROM conversations
            WHERE id = $1
        """
        
        async with self.pool.acquire() as conn:
            result = await conn.execute(query, conversation_id)
        
        return result != "DELETE 0"
    
    async def health_check(self) -> bool:
        """Check database connection health."""
        try:
            await self.connect()
            async with self.pool.acquire() as conn:
                await conn.fetchval("SELECT 1")
            return True
        except Exception:
            return False

# Global database service instance
db_service = DatabaseService()
