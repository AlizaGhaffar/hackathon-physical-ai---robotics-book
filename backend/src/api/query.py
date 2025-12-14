"""Query endpoint for RAG chatbot."""
import uuid
from fastapi import APIRouter, HTTPException, status
from src.models.request_models import QueryRequest
from src.models.response_models import QueryResponse, RetrievedChunk
from src.services.query_service import query_service
from src.services.chat_service import chat_service
from src.utils.logger import logger

router = APIRouter()


@router.post(
    "/query",
    response_model=QueryResponse,
    status_code=status.HTTP_200_OK,
    summary="Ask a question about the book",
    description="Process a user query using RAG (Retrieval-Augmented Generation)",
)
async def query_endpoint(request: QueryRequest):
    """
    Process a user query through the RAG pipeline.

    The system will:
    1. Embed the query
    2. Search for relevant book content
    3. Generate an answer using GPT
    4. Save the conversation to database

    Args:
        request: QueryRequest with query text, chapter, and optional session

    Returns:
        QueryResponse with answer, sources, and metadata
    """
    request_id = str(uuid.uuid4())

    try:
        # Generate session ID if not provided (skip database for now)
        session_id = request.session_id or str(uuid.uuid4())

        # TODO: Enable database operations after migrations
        # session_id = await chat_service.create_or_get_session(...)
        # await chat_service.save_message(...)

        # Process query through RAG pipeline
        result = await query_service.process_query(
            query=request.query,
            chapter_id=request.chapter_id,
            selected_text=request.selected_text,
        )

        # TODO: Save assistant response to database
        # await chat_service.save_message(...)

        # Format sources for response
        sources = [
            RetrievedChunk(
                chunk_id=source["chunk_id"],
                content=source["content"],
                score=source["score"],
                source_file=source["source_file"],
                chapter_id=source["chapter_id"],
            )
            for source in result["sources"]
        ]

        return QueryResponse(
            request_id=request_id,
            answer=result["answer"],
            session_id=session_id,
            sources=sources,
            token_usage=result["token_usage"],
            response_time_ms=result["response_time_ms"],
        )

    except Exception as e:
        logger.error(f"Query endpoint error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "query_processing_failed",
                "message": "Failed to process query. Please try again.",
                "request_id": request_id,
            },
        )


@router.get(
    "/history/{session_id}",
    summary="Get chat history",
    description="Retrieve conversation history for a session",
)
async def get_history(session_id: str, limit: int = 50):
    """
    Retrieve chat history for a session.

    Args:
        session_id: Session identifier
        limit: Maximum number of messages to retrieve

    Returns:
        List of messages with role, content, and timestamp
    """
    request_id = str(uuid.uuid4())

    try:
        history = await chat_service.get_session_history(
            session_id=session_id,
            limit=limit,
        )

        return {
            "request_id": request_id,
            "session_id": session_id,
            "messages": history,
            "total_messages": len(history),
        }

    except Exception as e:
        logger.error(f"History retrieval error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail={
                "error": "history_retrieval_failed",
                "message": "Failed to retrieve chat history.",
                "request_id": request_id,
            },
        )
