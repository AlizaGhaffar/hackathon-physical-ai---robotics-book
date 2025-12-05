"""
Pydantic Data Transfer Objects (DTOs)
Defines request/response models for the RAG chatbot API.
"""
from pydantic import BaseModel, Field
from typing import Optional, List
from uuid import UUID
from datetime import datetime

class ChatRequest(BaseModel):
    """Request model for chat endpoint."""
    conversation_id: Optional[UUID] = Field(None, description="UUID of existing conversation (null for new)")
    user_id: Optional[UUID] = Field(None, description="UUID of user (null for anonymous)")
    question: str = Field(..., min_length=1, max_length=2000, description="User's question")
    chapter: int = Field(..., ge=1, le=8, description="Current chapter context")
    selected_text: Optional[str] = Field(None, max_length=5000, description="User-highlighted text")

class Citation(BaseModel):
    """Citation model for source references."""
    chapter: int = Field(..., ge=1, le=8)
    section: str
    chunk_id: str
    similarity_score: float = Field(..., ge=0.0, le=1.0)

class ChatResponse(BaseModel):
    """Response model for chat endpoint."""
    conversation_id: UUID
    message_id: UUID
    answer: str
    citations: List[Citation]
    metadata: dict  # Performance metrics

class Message(BaseModel):
    """Message model for conversation history."""
    id: UUID
    role: str = Field(..., pattern="^(user|assistant)$")
    content: str
    citations: Optional[List[Citation]] = None
    selected_text: Optional[str] = None
    created_at: datetime

class ConversationHistory(BaseModel):
    """Conversation history response model."""
    conversation_id: UUID
    chapter: int = Field(..., ge=1, le=8)
    messages: List[Message]

class ErrorResponse(BaseModel):
    """Error response model."""
    error: str
    detail: str

class HealthResponse(BaseModel):
    """Health check response model."""
    status: str = Field(..., pattern="^(healthy|unhealthy)$")
    dependencies: dict
    version: str
    uptime_seconds: int
