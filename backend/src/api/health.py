"""Health check endpoint for monitoring."""
from fastapi import APIRouter, status
from datetime import datetime, timezone
from sqlalchemy import text
from src.models.response_models import HealthCheckResponse
from src.utils.qdrant_client import qdrant_manager
from src.utils.openai_client import openai_manager
from src.utils.database import engine
from src.utils.logger import logger

router = APIRouter()


async def check_neon_health() -> str:
    """Check Neon PostgreSQL connectivity."""
    try:
        async with engine.connect() as conn:
            await conn.execute(text("SELECT 1"))
        return "healthy"
    except Exception as e:
        logger.error(f"Neon health check failed: {str(e)}")
        return "unhealthy"


@router.get(
    "/health",
    status_code=status.HTTP_200_OK,
    response_model=HealthCheckResponse,
    summary="Health Check",
    description="Check health status of API and dependent services",
)
async def health_check():
    """
    Comprehensive health check endpoint.

    Returns the status of the API and all dependent services:
    - OpenAI API
    - Qdrant Cloud
    - Neon Postgres
    """
    # Check all services
    openai_healthy = await openai_manager.health_check()
    qdrant_healthy = await qdrant_manager.health_check()
    neon_status = await check_neon_health()

    # Determine overall status
    all_healthy = openai_healthy and qdrant_healthy and (neon_status == "healthy")

    return {
        "status": "healthy" if all_healthy else "degraded",
        "version": "1.0.0",
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "services": {
            "openai": "healthy" if openai_healthy else "unhealthy",
            "qdrant": "healthy" if qdrant_healthy else "unhealthy",
            "neon": neon_status,
        },
    }
