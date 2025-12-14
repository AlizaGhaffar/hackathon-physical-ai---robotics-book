"""Pydantic response models for API responses."""
from typing import Optional, List, Dict, Any
from datetime import datetime
from pydantic import BaseModel, Field


class BaseResponse(BaseModel):
    """Base response model with request tracing."""

    request_id: str = Field(..., description="Unique request identifier for tracing")


class HealthCheckResponse(BaseModel):
    """Health check endpoint response."""

    status: str = Field(..., description="Overall health status")
    version: str = Field(..., description="API version")
    timestamp: str = Field(..., description="Current server timestamp (ISO 8601)")
    services: Dict[str, str] = Field(
        ..., description="Status of dependent services (openai, qdrant, neon)"
    )


class RetrievedChunk(BaseModel):
    """Single retrieved document chunk with score."""

    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    content: str = Field(..., description="Text content of the chunk")
    score: float = Field(..., description="Relevance score (0.0-1.0)")
    source_file: str = Field(..., description="Source file path")
    chapter_id: str = Field(..., description="Chapter identifier")


class QueryResponse(BaseResponse):
    """Response for chat query endpoint."""

    answer: str = Field(..., description="Generated answer from the RAG system")
    session_id: str = Field(..., description="Chat session identifier")
    sources: List[RetrievedChunk] = Field(
        default_factory=list, description="Retrieved context chunks used for answer"
    )
    token_usage: Optional[Dict[str, int]] = Field(
        None, description="Token counts (prompt, completion, total)"
    )
    response_time_ms: Optional[float] = Field(
        None, description="Response generation time in milliseconds"
    )


class ErrorResponse(BaseModel):
    """Standard error response format."""

    error: str = Field(..., description="Error type or code")
    message: str = Field(..., description="Human-readable error message")
    request_id: str = Field(..., description="Request identifier for debugging")
    details: Optional[Dict[str, Any]] = Field(
        None, description="Additional error context"
    )


class SessionHistoryResponse(BaseResponse):
    """Chat session history response."""

    session_id: str = Field(..., description="Session identifier")
    messages: List[Dict[str, Any]] = Field(
        ..., description="List of messages (role, content, timestamp)"
    )
    total_messages: int = Field(..., description="Total number of messages in session")


class EmbedResponse(BaseResponse):
    """Response from document embedding endpoint."""

    chunks_processed: int = Field(..., description="Number of chunks created")
    chunks_embedded: int = Field(..., description="Number of chunks successfully embedded")
    chapter_id: str = Field(..., description="Chapter identifier")
    processing_time_ms: float = Field(..., description="Total processing time")
    status: str = Field(..., description="Processing status (success, partial, failed)")
