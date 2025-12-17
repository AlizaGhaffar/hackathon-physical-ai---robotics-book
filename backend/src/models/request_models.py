"""Pydantic request models for API endpoints."""
from typing import Optional, List
from pydantic import BaseModel, Field, validator


class QueryRequest(BaseModel):
    """Request model for chat query endpoint."""

    query: str = Field(
        ...,
        description="User's question about the book content",
        min_length=1,
        max_length=1000,
    )
    session_id: Optional[str] = Field(
        None, description="Session identifier for conversation continuity"
    )
    chapter_id: str = Field(..., description="Chapter to query (e.g., 'chapter-1')")
    selected_text: Optional[str] = Field(
        None,
        description="User-selected text for focused queries",
        max_length=5000,
    )

    @validator("query")
    def validate_query(cls, v):
        """Validate query text."""
        v = v.strip()
        if not v:
            raise ValueError("Query cannot be empty or whitespace only")
        return v

    @validator("chapter_id")
    def validate_chapter_id(cls, v):
        """Validate chapter ID format - accepts both 'chapter-X' and 'Chapter X' formats."""
        v_lower = v.lower()
        if not (v_lower.startswith("chapter-") or v_lower.startswith("chapter ")):
            raise ValueError("Chapter ID must start with 'chapter-' or 'Chapter '")
        return v


class ChunkInput(BaseModel):
    """Single chunk for embedding."""

    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    chapter_id: str = Field(..., description="Chapter identifier")
    content: str = Field(..., description="Text content to embed")
    source_file: str = Field(..., description="Source file path")
    chunk_index: int = Field(..., description="Index of chunk within document")
    metadata: Optional[dict] = Field(
        None, description="Additional metadata (section, heading, etc.)"
    )


class EmbedRequest(BaseModel):
    """Request model for document embedding endpoint."""

    chunks: List[ChunkInput] = Field(
        ..., description="List of chunks to embed", min_items=1, max_items=500
    )
    chapter_id: str = Field(..., description="Chapter identifier for all chunks")
    force_reindex: bool = Field(
        default=False,
        description="Force re-embedding of existing chunks",
    )

    @validator("chapter_id")
    def validate_chapter_id(cls, v):
        """Validate chapter ID format - accepts both 'chapter-X' and 'Chapter X' formats."""
        v_lower = v.lower()
        if not (v_lower.startswith("chapter-") or v_lower.startswith("chapter ")):
            raise ValueError("Chapter ID must start with 'chapter-' or 'Chapter '")
        return v


class FeedbackRequest(BaseModel):
    """Request model for user feedback on responses."""

    message_id: str = Field(..., description="ID of the message being rated")
    session_id: str = Field(..., description="Session identifier")
    rating: str = Field(
        ..., description="User rating: 'thumbs_up' or 'thumbs_down'"
    )
    feedback_text: Optional[str] = Field(
        None, description="Optional text feedback", max_length=2000
    )

    @validator("rating")
    def validate_rating(cls, v):
        """Validate rating value."""
        if v not in ["thumbs_up", "thumbs_down"]:
            raise ValueError("Rating must be 'thumbs_up' or 'thumbs_down'")
        return v
