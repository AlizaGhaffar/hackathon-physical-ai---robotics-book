"""
FastAPI application entry point for RAG Chatbot Backend.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from src.config import settings, validate_settings
from src.middleware.error_handler import add_error_handlers
from src.middleware.rate_limiter import RateLimiterMiddleware
from src.api import health, query


@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Application lifespan manager.
    Runs on startup and shutdown.
    """
    # Startup
    print("[*] Starting RAG Chatbot Backend...")
    validate_settings()
    print("[OK] Application startup complete")

    yield

    # Shutdown
    print("[!] Shutting down RAG Chatbot Backend...")


# Create FastAPI application
app = FastAPI(
    title="RAG Chatbot Backend",
    description="Retrieval-Augmented Generation chatbot for Physical AI Textbook",
    version="1.0.0",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[origin.strip() for origin in settings.cors_origins.split(",")],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Rate limiting middleware
app.add_middleware(RateLimiterMiddleware)

# Error handling
add_error_handlers(app)

# Include routers
app.include_router(health.router, prefix="/api", tags=["Health"])
app.include_router(query.router, prefix="/api", tags=["Query"])


@app.get("/")
async def root():
    """Root endpoint - API information."""
    return {
        "service": "RAG Chatbot Backend",
        "version": "1.0.0",
        "status": "running",
        "docs": "/docs",
        "health": "/api/health"
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host=settings.api_host,
        port=settings.api_port,
        reload=settings.api_reload,
    )
