"""
FastAPI Main Application
Entry point for the RAG Chatbot API.
"""
import os
import time
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from dotenv import load_dotenv

from src.services.database import db_service
from src.services.retrieval import retrieval_service
from src.services.generation import generation_service
from src.models.dtos import HealthResponse

load_dotenv()

# Application startup time
START_TIME = time.time()
API_VERSION = "1.0.0"

app = FastAPI(
    title="RAG Chatbot API",
    description="AI-powered chatbot for Physical AI Textbook",
    version=API_VERSION
)

# CORS Configuration
frontend_url = os.getenv("FRONTEND_URL", "http://localhost:3000")
# Allow Vercel domains and localhost
allowed_origins = [
    frontend_url,
    "http://localhost:3000",
    "https://*.vercel.app",  # Vercel preview deployments
]

# In production, allow all Vercel domains
if os.getenv("RAILWAY_ENVIRONMENT") == "production":
    allowed_origins.append("https://*.vercel.app")

app.add_middleware(
    CORSMiddleware,
    allow_origin_regex=r"https://.*\.vercel\.app",  # Allow all Vercel domains
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "DELETE"],
    allow_headers=["*"],
)

# Import routers
from src.api import chat, conversations

app.include_router(chat.router, prefix="/api", tags=["Chat"])
app.include_router(conversations.router, prefix="/api", tags=["Conversations"])

@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "message": "RAG Chatbot API",
        "version": API_VERSION,
        "docs": "/docs"
    }

@app.get("/health", response_model=HealthResponse)
async def health_check():
    """
    Health check endpoint.
    Verifies connectivity to all dependencies.
    """
    # Check dependencies
    db_healthy = await db_service.health_check()
    qdrant_healthy = retrieval_service.health_check()
    openai_healthy = generation_service.health_check()
    
    dependencies = {
        "database": "connected" if db_healthy else "disconnected",
        "qdrant": "connected" if qdrant_healthy else "disconnected",
        "openai": "connected" if openai_healthy else "disconnected"
    }
    
    all_healthy = all([db_healthy, qdrant_healthy, openai_healthy])
    status = "healthy" if all_healthy else "unhealthy"
    
    uptime = int(time.time() - START_TIME)
    
    response = HealthResponse(
        status=status,
        dependencies=dependencies,
        version=API_VERSION,
        uptime_seconds=uptime
    )
    
    status_code = 200 if all_healthy else 503
    
    return JSONResponse(
        status_code=status_code,
        content=response.dict()
    )

@app.on_event("startup")
async def startup_event():
    """Initialize connections on startup."""
    await db_service.connect()
    print(f"[OK] RAG Chatbot API started (version {API_VERSION})")

@app.on_event("shutdown")
async def shutdown_event():
    """Clean up connections on shutdown."""
    await db_service.disconnect()
    print("[OK] RAG Chatbot API shut down")
