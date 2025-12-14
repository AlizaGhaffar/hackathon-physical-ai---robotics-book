"""
Agent-Based RAG Backend

A FastAPI application that uses OpenAI Agents SDK with Qdrant vector database
for retrieval-augmented generation. Answers questions strictly based on book content.

Feature: 004-agent-rag-backend
"""

import os
import logging
from typing import Optional, List, Dict, Any
from datetime import datetime

from fastapi import FastAPI, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field, validator
import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Filter, FieldCondition, MatchValue
from openai import OpenAI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Initialize FastAPI app
app = FastAPI(
    title="Agent-Based RAG Backend API",
    version="1.0.0",
    description="Retrieval-Augmented Generation API for Physical AI textbook Q&A"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize clients
cohere_client = cohere.Client(api_key=os.getenv("COHERE_API_KEY"))
qdrant_client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)
openai_client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

# Configuration
QDRANT_COLLECTION = os.getenv("QDRANT_COLLECTION", "rag_embedding")
TOP_K_CHUNKS = int(os.getenv("TOP_K_CHUNKS", "5"))
SIMILARITY_THRESHOLD = float(os.getenv("SIMILARITY_THRESHOLD", "0.7"))
OPENAI_MODEL = os.getenv("OPENAI_MODEL", "gpt-4-turbo-preview")
TEMPERATURE = float(os.getenv("OPENAI_TEMPERATURE", "0.2"))

# Pydantic Models
class Citation(BaseModel):
    """Source citation from book content"""
    chapter_id: int = Field(..., description="Chapter number (1-8)")
    section: str = Field(..., description="Section title")
    page: Optional[int] = Field(None, description="Page number if available")
    chunk_id: str = Field(..., description="Unique chunk identifier")
    source_url: str = Field(..., description="URL to original content")
    relevance_score: Optional[float] = Field(None, ge=0.0, le=1.0, description="Similarity score")


class AskRequest(BaseModel):
    """Request model for /ask endpoint"""
    question: str = Field(..., min_length=1, max_length=1000, description="User's question")
    chapter_filter: Optional[int] = Field(None, ge=1, le=8, description="Optional chapter filter (1-8)")
    section_filter: Optional[str] = Field(None, max_length=100, description="Optional section filter")
    session_id: Optional[str] = Field(None, description="Session ID for multi-turn conversations")

    @validator('question')
    def question_not_empty(cls, v):
        if not v.strip():
            raise ValueError('Question cannot be empty or whitespace only')
        return v.strip()

    @validator('section_filter')
    def validate_both_filters(cls, v, values):
        if v is not None and values.get('chapter_filter') is not None:
            raise ValueError('Specify either chapter_filter or section_filter, not both')
        return v


class AskResponse(BaseModel):
    """Response model for /ask endpoint"""
    answer: str = Field(..., description="Generated response text")
    sources: List[Citation] = Field(..., description="Source citations")
    confidence: str = Field(..., description="Confidence level: high, medium, low")
    retrieval_count: int = Field(..., description="Number of chunks retrieved")
    processing_time_ms: int = Field(..., description="Total processing time in milliseconds")


class HealthResponse(BaseModel):
    """Response model for /health endpoint"""
    status: str = Field(..., description="Overall health status")
    qdrant_available: bool = Field(..., description="Qdrant connectivity")
    openai_available: bool = Field(..., description="OpenAI API availability")
    cohere_available: bool = Field(..., description="Cohere API availability")
    timestamp: str = Field(..., description="Health check timestamp")


# Core Functions

def embed_query(text: str) -> List[float]:
    """
    Embed user query using Cohere embed-english-v3.0

    Args:
        text: Query text to embed

    Returns:
        1024-dimensional embedding vector
    """
    try:
        logger.info(f"Embedding query: {text[:100]}...")
        response = cohere_client.embed(
            texts=[text],
            model="embed-english-v3.0",
            input_type="search_query",  # Optimized for queries
            embedding_types=["float"]
        )
        embedding = response.embeddings[0]
        logger.info(f"Query embedded successfully (dim={len(embedding)})")
        return embedding
    except Exception as e:
        logger.error(f"Error embedding query: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Cohere embedding service unavailable: {str(e)}"
        )


def retrieve_chunks(
    query_embedding: List[float],
    top_k: int = TOP_K_CHUNKS,
    chapter_filter: Optional[int] = None,
    section_filter: Optional[str] = None
) -> List[Dict[str, Any]]:
    """
    Retrieve relevant book chunks from Qdrant

    Args:
        query_embedding: 1024-dim query vector
        top_k: Number of chunks to retrieve
        chapter_filter: Optional chapter ID filter (1-8)
        section_filter: Optional section name filter

    Returns:
        List of chunk dicts with text, metadata, and scores
    """
    try:
        logger.info(f"Retrieving top-{top_k} chunks from Qdrant (collection={QDRANT_COLLECTION})")

        # Build metadata filter
        query_filter = None
        if chapter_filter is not None:
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="chapter_id",
                        match=MatchValue(value=chapter_filter)
                    )
                ]
            )
            logger.info(f"Applying chapter filter: chapter_id={chapter_filter}")
        elif section_filter is not None:
            query_filter = Filter(
                must=[
                    FieldCondition(
                        key="section",
                        match=MatchValue(value=section_filter)
                    )
                ]
            )
            logger.info(f"Applying section filter: section='{section_filter}'")

        # Search Qdrant
        search_results = qdrant_client.search(
            collection_name=QDRANT_COLLECTION,
            query_vector=query_embedding,
            limit=top_k,
            query_filter=query_filter,
            with_payload=True,
            score_threshold=SIMILARITY_THRESHOLD
        )

        logger.info(f"Retrieved {len(search_results)} chunks (threshold={SIMILARITY_THRESHOLD})")

        # Format results
        chunks = []
        for result in search_results:
            chunk = {
                "text": result.payload.get("text", ""),
                "chapter_id": result.payload.get("chapter_id"),
                "section": result.payload.get("section", ""),
                "page": result.payload.get("page"),
                "heading": result.payload.get("heading", ""),
                "source_url": result.payload.get("source_url", ""),
                "chunk_id": str(result.id),
                "score": result.score
            }
            chunks.append(chunk)
            logger.debug(f"Chunk {chunk['chunk_id']}: score={chunk['score']:.3f}, chapter={chunk['chapter_id']}, section='{chunk['section']}'")

        return chunks

    except Exception as e:
        logger.error(f"Error retrieving from Qdrant: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_503_SERVICE_UNAVAILABLE,
            detail=f"Qdrant vector database unavailable: {str(e)}"
        )


def format_context(chunks: List[Dict[str, Any]]) -> str:
    """
    Format retrieved chunks into a clean context block for the agent

    Args:
        chunks: List of chunk dicts from retrieve_chunks()

    Returns:
        Formatted context string with chunk separators and metadata
    """
    if not chunks:
        return "No relevant book content found for this query."

    context_parts = []
    for i, chunk in enumerate(chunks, 1):
        context_parts.append(f"--- Chunk {i} (Chapter {chunk['chapter_id']}, Section: {chunk['section']}) ---")
        context_parts.append(chunk['text'])
        context_parts.append(f"Source: {chunk['source_url']}")
        context_parts.append("")  # Blank line separator

    formatted = "\n".join(context_parts)
    logger.info(f"Formatted context: {len(chunks)} chunks, {len(formatted)} chars")
    return formatted


def create_agent() -> Any:
    """
    Create OpenAI Assistant agent with retrieval tool

    Returns:
        OpenAI Assistant object
    """
    try:
        system_message = """You are a helpful assistant that answers questions about a Physical AI and Robotics textbook.

STRICT RULES:
1. Answer ONLY using information from the book content provided by the retrieve_book_content function
2. If the function returns no relevant content, respond: "I couldn't find information about that in the book"
3. Include citations for every claim (chapter, section)
4. Do NOT use external knowledge or make assumptions beyond the book content
5. If asked about topics outside the book scope, politely decline

Format your response as:
Answer: [Your answer based on book content]
Sources: [List of citations: Chapter X, Section Y]"""

        # Note: OpenAI Assistants API with function calling requires async implementation
        # For this simplified version, we'll use direct GPT-4 calls with manual tool simulation
        logger.info("Agent configuration prepared (using direct GPT-4 calls)")
        return {
            "model": OPENAI_MODEL,
            "temperature": TEMPERATURE,
            "system_message": system_message
        }

    except Exception as e:
        logger.error(f"Error creating agent: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create agent: {str(e)}"
        )


def calculate_confidence(chunks: List[Dict[str, Any]]) -> str:
    """
    Calculate confidence level based on similarity scores

    Args:
        chunks: Retrieved chunks with scores

    Returns:
        "high", "medium", or "low"
    """
    if not chunks:
        return "low"

    avg_score = sum(chunk['score'] for chunk in chunks) / len(chunks)

    if avg_score > 0.8:
        return "high"
    elif avg_score > 0.6:
        return "medium"
    else:
        return "low"


# API Endpoints

@app.post("/ask", response_model=AskResponse)
async def ask_question(request: AskRequest):
    """
    Ask a question about book content

    Retrieves relevant chunks from Qdrant and generates a grounded answer
    using OpenAI GPT-4. Supports chapter/section filtering.
    """
    start_time = datetime.now()

    try:
        logger.info(f"Received question: '{request.question}'")
        if request.chapter_filter:
            logger.info(f"  Chapter filter: {request.chapter_filter}")
        if request.section_filter:
            logger.info(f"  Section filter: '{request.section_filter}'")

        # Step 1: Embed the query
        query_embedding = embed_query(request.question)

        # Step 2: Retrieve relevant chunks
        chunks = retrieve_chunks(
            query_embedding,
            top_k=TOP_K_CHUNKS,
            chapter_filter=request.chapter_filter,
            section_filter=request.section_filter
        )

        # Step 3: Format context
        context = format_context(chunks)

        # Step 4: Generate answer using GPT-4
        agent_config = create_agent()

        if not chunks:
            # No relevant content found - deflect
            answer = "I couldn't find information about that in the book."
            sources = []
            confidence = "low"
        else:
            # Generate grounded answer
            messages = [
                {"role": "system", "content": agent_config["system_message"]},
                {"role": "user", "content": f"Context from book:\n\n{context}\n\nUser question: {request.question}\n\nPlease answer based strictly on the context provided above."}
            ]

            logger.info("Calling OpenAI GPT-4 for answer generation...")
            response = openai_client.chat.completions.create(
                model=agent_config["model"],
                messages=messages,
                temperature=agent_config["temperature"],
                max_tokens=500
            )

            answer = response.choices[0].message.content.strip()
            logger.info(f"Generated answer: {answer[:100]}...")

            # Extract sources from chunks
            sources = [
                Citation(
                    chapter_id=chunk['chapter_id'],
                    section=chunk['section'],
                    page=chunk.get('page'),
                    chunk_id=chunk['chunk_id'],
                    source_url=chunk['source_url'],
                    relevance_score=chunk['score']
                )
                for chunk in chunks
            ]

            confidence = calculate_confidence(chunks)

        # Calculate processing time
        processing_time_ms = int((datetime.now() - start_time).total_seconds() * 1000)

        logger.info(f"Request completed: retrieval_count={len(chunks)}, confidence={confidence}, time={processing_time_ms}ms")

        return AskResponse(
            answer=answer,
            sources=sources,
            confidence=confidence,
            retrieval_count=len(chunks),
            processing_time_ms=processing_time_ms
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Unexpected error in /ask endpoint: {str(e)}", exc_info=True)
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Internal server error: {str(e)}"
        )


@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint

    Verifies connectivity to all external services:
    - Qdrant vector database
    - OpenAI API
    - Cohere API
    """
    logger.info("Health check requested")

    # Check Qdrant
    qdrant_available = False
    try:
        collections = qdrant_client.get_collections()
        qdrant_available = any(c.name == QDRANT_COLLECTION for c in collections.collections)
        logger.info(f"Qdrant check: {'OK' if qdrant_available else 'FAIL'} (collection={QDRANT_COLLECTION})")
    except Exception as e:
        logger.warning(f"Qdrant health check failed: {str(e)}")

    # Check OpenAI
    openai_available = False
    try:
        openai_client.models.list()
        openai_available = True
        logger.info("OpenAI check: OK")
    except Exception as e:
        logger.warning(f"OpenAI health check failed: {str(e)}")

    # Check Cohere
    cohere_available = False
    try:
        # Simple check: try to get API key info
        cohere_client.check_api_key()
        cohere_available = True
        logger.info("Cohere check: OK")
    except Exception as e:
        logger.warning(f"Cohere health check failed: {str(e)}")

    # Determine overall status
    if qdrant_available and openai_available and cohere_available:
        overall_status = "healthy"
    elif any([qdrant_available, openai_available, cohere_available]):
        overall_status = "degraded"
    else:
        overall_status = "unhealthy"

    logger.info(f"Health check complete: status={overall_status}")

    return HealthResponse(
        status=overall_status,
        qdrant_available=qdrant_available,
        openai_available=openai_available,
        cohere_available=cohere_available,
        timestamp=datetime.utcnow().isoformat() + "Z"
    )


@app.get("/")
async def root():
    """Root endpoint - API info"""
    return {
        "name": "Agent-Based RAG Backend",
        "version": "1.0.0",
        "description": "Retrieval-Augmented Generation API for Physical AI textbook Q&A",
        "endpoints": {
            "ask": "POST /ask - Ask a question about book content",
            "health": "GET /health - Health check",
            "docs": "GET /docs - OpenAPI documentation"
        }
    }


if __name__ == "__main__":
    import uvicorn

    # Startup validation
    required_env_vars = [
        "OPENAI_API_KEY",
        "COHERE_API_KEY",
        "QDRANT_URL",
        "QDRANT_API_KEY"
    ]

    missing = [var for var in required_env_vars if not os.getenv(var)]
    if missing:
        logger.error(f"Missing required environment variables: {', '.join(missing)}")
        logger.error("Please create a .env file with required credentials")
        exit(1)

    logger.info("Starting Agent-Based RAG Backend...")
    logger.info(f"Qdrant collection: {QDRANT_COLLECTION}")
    logger.info(f"Top-K chunks: {TOP_K_CHUNKS}")
    logger.info(f"Similarity threshold: {SIMILARITY_THRESHOLD}")
    logger.info(f"OpenAI model: {OPENAI_MODEL}")
    logger.info(f"Temperature: {TEMPERATURE}")

    uvicorn.run(app, host="0.0.0.0", port=8000)
