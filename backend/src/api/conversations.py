"""
Conversations API Endpoints
Handles GET and DELETE /api/conversations/{id} endpoints.
"""
from uuid import UUID
from fastapi import APIRouter, HTTPException

from src.models.dtos import ConversationHistory, Message, Citation
from src.services.database import db_service

router = APIRouter()

@router.get("/conversations/{conversation_id}", response_model=ConversationHistory)
async def get_conversation_history(conversation_id: UUID):
    """
    Retrieve conversation history with all messages.
    
    Args:
        conversation_id: UUID of the conversation
        
    Returns:
        ConversationHistory with all messages
    """
    try:
        conversation = await db_service.get_conversation_history(conversation_id)
        
        if not conversation:
            raise HTTPException(
                status_code=404,
                detail=f"Conversation with ID {conversation_id} does not exist"
            )
        
        # Format messages
        messages = []
        for msg in conversation['messages']:
            citations = None
            if msg['citations']:
                citations = [Citation(**c) for c in msg['citations']]
            
            messages.append(Message(
                id=msg['id'],
                role=msg['role'],
                content=msg['content'],
                citations=citations,
                selected_text=msg.get('selected_text'),
                created_at=msg['created_at']
            ))
        
        return ConversationHistory(
            conversation_id=conversation['conversation_id'],
            chapter=conversation['chapter'],
            messages=messages
        )
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error retrieving conversation: {str(e)}"
        )

@router.delete("/conversations/{conversation_id}", status_code=204)
async def delete_conversation(conversation_id: UUID):
    """
    Delete a conversation and all its messages.
    
    Args:
        conversation_id: UUID of the conversation to delete
    """
    try:
        deleted = await db_service.delete_conversation(conversation_id)
        
        if not deleted:
            raise HTTPException(
                status_code=404,
                detail=f"Conversation with ID {conversation_id} does not exist"
            )
        
        return None
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Error deleting conversation: {str(e)}"
        )
