"""
Chat API Endpoints
Handles POST /api/chat for question answering.
"""
import time
from typing import Optional
from uuid import UUID
from fastapi import APIRouter, HTTPException
from fastapi.responses import StreamingResponse

from src.models.dtos import ChatRequest, ChatResponse, Citation, ErrorResponse
from src.services.database import db_service
from src.services.retrieval import retrieval_service
from src.services.generation import generation_service

router = APIRouter()

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Submit a question and receive an AI-generated answer.
    
    Supports two modes:
    - Whole-book RAG: Retrieve from all chapters
    - Selected-text RAG: Prioritize user-highlighted text
    """
    try:
        start_time = time.time()
        
        # Step 1: Retrieve context from Qdrant
        retrieval_start = time.time()
        
        retrieved_chunks = await retrieval_service.retrieve_context(
            question=request.question,
            chapter=request.chapter,
            selected_text=request.selected_text,
            limit=5,
            score_threshold=0.7
        )
        
        retrieval_time = int((time.time() - retrieval_start) * 1000)
        
        if not retrieved_chunks:
            raise HTTPException(
                status_code=404,
                detail="I couldn't find relevant content in the textbook for this question. Try rephrasing or asking about topics covered in the chapters."
            )
        
        # Step 2: Generate answer with GPT-4
        generation_start = time.time()
        
        result = await generation_service.generate_answer(
            question=request.question,
            retrieved_chunks=retrieved_chunks,
            selected_text=request.selected_text
        )
        
        generation_time = int((time.time() - generation_start) * 1000)
        
        # Step 3: Save conversation to database
        conversation_id = request.conversation_id
        
        if not conversation_id:
            # Create new conversation
            conversation_id = await db_service.create_conversation(
                user_id=request.user_id,
                chapter=request.chapter
            )
        
        # Save user message
        await db_service.create_message(
            conversation_id=conversation_id,
            role="user",
            content=request.question,
            selected_text=request.selected_text
        )
        
        # Save assistant message
        message_id = await db_service.create_message(
            conversation_id=conversation_id,
            role="assistant",
            content=result['answer'],
            citations=result['citations'],
            metadata={
                "retrieval_time_ms": retrieval_time,
                "generation_time_ms": generation_time
            }
        )
        
        total_time = int((time.time() - start_time) * 1000)
        
        # Format response
        citations = [Citation(**c) for c in result['citations']]
        
        return ChatResponse(
            conversation_id=conversation_id,
            message_id=message_id,
            answer=result['answer'],
            citations=citations,
            metadata={
                "retrieval_time_ms": retrieval_time,
                "generation_time_ms": generation_time,
                "total_time_ms": total_time
            }
        )
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Internal server error: {str(e)}"
        )

@router.post("/chat/stream")
async def chat_stream(request: ChatRequest):
    """
    Submit a question with streaming response (Server-Sent Events).
    
    Event types:
    - stage: 'searching' | 'generating' | 'complete'
    - token: Progressive answer token
    """
    async def event_generator():
        try:
            import json
            
            # Stage 1: Searching
            yield f"data: {json.dumps({'stage': 'searching'})}\n\n"
            
            retrieved_chunks = await retrieval_service.retrieve_context(
                question=request.question,
                chapter=request.chapter,
                selected_text=request.selected_text,
                limit=5,
                score_threshold=0.7
            )
            
            if not retrieved_chunks:
                yield f"data: {json.dumps({'error': 'No relevant content found'})}\n\n"
                return
            
            # Stage 2: Generating
            yield f"data: {json.dumps({'stage': 'generating'})}\n\n"
            
            answer_tokens = []
            
            async for token in generation_service.stream_answer(
                question=request.question,
                retrieved_chunks=retrieved_chunks,
                selected_text=request.selected_text
            ):
                answer_tokens.append(token)
                yield f"data: {json.dumps({'token': token})}\n\n"
            
            # Stage 3: Complete with citations
            citations = [
                {
                    "chapter": chunk['chapter'],
                    "section": chunk['section'],
                    "chunk_id": chunk['chunk_id'],
                    "similarity_score": chunk['similarity_score']
                }
                for chunk in retrieved_chunks
            ]
            
            # Save to database
            conversation_id = request.conversation_id
            
            if not conversation_id:
                conversation_id = await db_service.create_conversation(
                    user_id=request.user_id,
                    chapter=request.chapter
                )
            
            await db_service.create_message(
                conversation_id=conversation_id,
                role="user",
                content=request.question,
                selected_text=request.selected_text
            )
            
            full_answer = ''.join(answer_tokens)
            
            await db_service.create_message(
                conversation_id=conversation_id,
                role="assistant",
                content=full_answer,
                citations=citations
            )
            
            yield f"data: {json.dumps({'stage': 'complete', 'citations': citations, 'conversation_id': str(conversation_id)})}\n\n"
            
        except Exception as e:
            yield f"data: {json.dumps({'error': str(e)})}\n\n"
    
    return StreamingResponse(
        event_generator(),
        media_type="text/event-stream"
    )
