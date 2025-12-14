"""SQLAlchemy database models."""
from datetime import datetime, timezone
from sqlalchemy import Column, Integer, String, DateTime, Text, Float, JSON, Index
from sqlalchemy.orm import declarative_mixin
from src.utils.database import Base


@declarative_mixin
class TimestampMixin:
    """Mixin to add created_at and updated_at timestamps to models."""

    created_at = Column(
        DateTime(timezone=True),
        default=lambda: datetime.now(timezone.utc),
        nullable=False,
    )
    updated_at = Column(
        DateTime(timezone=True),
        default=lambda: datetime.now(timezone.utc),
        onupdate=lambda: datetime.now(timezone.utc),
        nullable=False,
    )


class ChatSession(TimestampMixin, Base):
    """Chat session model for storing conversation context."""

    __tablename__ = "chat_sessions"

    id = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(String(100), unique=True, nullable=False, index=True)
    chapter_id = Column(String(50), nullable=False)
    user_id = Column(String(100), nullable=True)  # Optional user tracking

    __table_args__ = (
        Index("idx_session_chapter", "session_id", "chapter_id"),
    )


class ChatMessage(TimestampMixin, Base):
    """Individual chat messages within a session."""

    __tablename__ = "chat_messages"

    id = Column(Integer, primary_key=True, autoincrement=True)
    session_id = Column(String(100), nullable=False, index=True)
    role = Column(String(20), nullable=False)  # 'user' or 'assistant'
    content = Column(Text, nullable=False)
    token_count = Column(Integer, nullable=True)
    message_metadata = Column(JSON, nullable=True)  # For storing additional context

    __table_args__ = (
        Index("idx_message_session_created", "session_id", "created_at"),
    )


class DocumentChunk(TimestampMixin, Base):
    """Metadata for document chunks stored in Qdrant."""

    __tablename__ = "document_chunks"

    id = Column(Integer, primary_key=True, autoincrement=True)
    chunk_id = Column(String(100), unique=True, nullable=False, index=True)
    chapter_id = Column(String(50), nullable=False, index=True)
    source_file = Column(String(500), nullable=False)
    chunk_index = Column(Integer, nullable=False)
    content_preview = Column(Text, nullable=True)  # First 200 chars
    token_count = Column(Integer, nullable=True)
    embedding_model = Column(String(100), nullable=False)

    __table_args__ = (
        Index("idx_chunk_chapter", "chapter_id", "chunk_index"),
    )


class QueryLog(TimestampMixin, Base):
    """Log of queries for analytics and debugging."""

    __tablename__ = "query_logs"

    id = Column(Integer, primary_key=True, autoincrement=True)
    request_id = Column(String(50), unique=True, nullable=False, index=True)
    session_id = Column(String(100), nullable=True)
    chapter_id = Column(String(50), nullable=False)
    query_text = Column(Text, nullable=False)
    response_text = Column(Text, nullable=True)
    retrieval_count = Column(Integer, nullable=True)
    response_time_ms = Column(Float, nullable=True)
    error_message = Column(Text, nullable=True)

    __table_args__ = (
        Index("idx_query_chapter_created", "chapter_id", "created_at"),
    )
