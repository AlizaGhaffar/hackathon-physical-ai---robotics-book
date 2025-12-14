"""Chat service for managing conversations and history."""
import uuid
from typing import Optional, List, Dict, Any
from datetime import datetime, timezone
from sqlalchemy import select
from sqlalchemy.ext.asyncio import AsyncSession
from src.models.db_models import ChatSession, ChatMessage
from src.utils.database import AsyncSessionLocal
from src.utils.logger import logger


class ChatService:
    """Service for managing chat sessions and messages."""

    async def create_or_get_session(
        self,
        session_id: Optional[str],
        chapter_id: str,
        user_id: Optional[str] = None,
    ) -> str:
        """
        Create a new session or retrieve existing one.

        Args:
            session_id: Existing session ID or None
            chapter_id: Chapter identifier
            user_id: Optional user identifier

        Returns:
            Session ID (new or existing)
        """
        async with AsyncSessionLocal() as db:
            # If session_id provided, check if it exists
            if session_id:
                result = await db.execute(
                    select(ChatSession).where(ChatSession.session_id == session_id)
                )
                existing_session = result.scalar_one_or_none()

                if existing_session:
                    logger.info(f"Using existing session: {session_id}")
                    return session_id

            # Create new session
            new_session_id = str(uuid.uuid4())
            session = ChatSession(
                session_id=new_session_id,
                chapter_id=chapter_id,
                user_id=user_id,
            )

            db.add(session)
            await db.commit()

            logger.info(f"Created new session: {new_session_id}")
            return new_session_id

    async def save_message(
        self,
        session_id: str,
        role: str,
        content: str,
        token_count: Optional[int] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> int:
        """
        Save a chat message to the database.

        Args:
            session_id: Session identifier
            role: Message role ('user' or 'assistant')
            content: Message content
            token_count: Optional token count
            metadata: Optional metadata dict

        Returns:
            Message ID
        """
        async with AsyncSessionLocal() as db:
            message = ChatMessage(
                session_id=session_id,
                role=role,
                content=content,
                token_count=token_count,
                metadata=metadata or {},
            )

            db.add(message)
            await db.commit()
            await db.refresh(message)

            logger.info(f"Saved {role} message in session {session_id}")
            return message.id

    async def get_session_history(
        self,
        session_id: str,
        limit: Optional[int] = 50,
    ) -> List[Dict[str, Any]]:
        """
        Retrieve chat history for a session.

        Args:
            session_id: Session identifier
            limit: Maximum number of messages to retrieve

        Returns:
            List of message dicts with role, content, timestamp
        """
        async with AsyncSessionLocal() as db:
            query = (
                select(ChatMessage)
                .where(ChatMessage.session_id == session_id)
                .order_by(ChatMessage.created_at.asc())
                .limit(limit)
            )

            result = await db.execute(query)
            messages = result.scalars().all()

            history = []
            for msg in messages:
                history.append({
                    "id": msg.id,
                    "role": msg.role,
                    "content": msg.content,
                    "timestamp": msg.created_at.isoformat(),
                    "token_count": msg.token_count,
                    "metadata": msg.metadata,
                })

            logger.info(f"Retrieved {len(history)} messages for session {session_id}")
            return history

    async def delete_session(self, session_id: str) -> bool:
        """
        Delete a chat session and all its messages.

        Args:
            session_id: Session identifier

        Returns:
            True if successful
        """
        async with AsyncSessionLocal() as db:
            # Delete messages first
            await db.execute(
                select(ChatMessage).where(ChatMessage.session_id == session_id)
            )

            # Delete session
            result = await db.execute(
                select(ChatSession).where(ChatSession.session_id == session_id)
            )
            session = result.scalar_one_or_none()

            if session:
                await db.delete(session)
                await db.commit()
                logger.info(f"Deleted session: {session_id}")
                return True

            return False

    async def get_recent_sessions(
        self,
        user_id: Optional[str] = None,
        limit: int = 10,
    ) -> List[Dict[str, Any]]:
        """
        Get recent chat sessions for a user.

        Args:
            user_id: Optional user identifier
            limit: Maximum number of sessions

        Returns:
            List of session dicts
        """
        async with AsyncSessionLocal() as db:
            query = select(ChatSession).order_by(ChatSession.created_at.desc()).limit(limit)

            if user_id:
                query = query.where(ChatSession.user_id == user_id)

            result = await db.execute(query)
            sessions = result.scalars().all()

            session_list = []
            for session in sessions:
                session_list.append({
                    "session_id": session.session_id,
                    "chapter_id": session.chapter_id,
                    "user_id": session.user_id,
                    "created_at": session.created_at.isoformat(),
                })

            return session_list


# Global service instance
chat_service = ChatService()
