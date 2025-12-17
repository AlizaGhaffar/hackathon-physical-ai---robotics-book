"""Database connection and session management."""
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import declarative_base
from src.config import settings
import ssl

# Prepare database URL for asyncpg
# asyncpg doesn't support sslmode query parameter, need to use connect_args
database_url = settings.database_url.replace("postgresql://", "postgresql+asyncpg://")
# Remove sslmode and channel_binding from URL as they're not supported by asyncpg
database_url = database_url.split('?')[0] if '?' in database_url else database_url

# Configure SSL context for asyncpg
ssl_context = ssl.create_default_context()
ssl_context.check_hostname = False
ssl_context.verify_mode = ssl.CERT_NONE

# Create async engine
engine = create_async_engine(
    database_url,
    echo=False,
    pool_pre_ping=True,
    pool_size=10,
    max_overflow=20,
    connect_args={
        "ssl": ssl_context,
    }
)

# Create async session factory
AsyncSessionLocal = async_sessionmaker(
    engine,
    class_=AsyncSession,
    expire_on_commit=False,
    autocommit=False,
    autoflush=False,
)

# Base class for SQLAlchemy models
Base = declarative_base()


async def get_db():
    """
    Dependency for getting database session.

    Usage in FastAPI:
        @app.get("/endpoint")
        async def endpoint(db: AsyncSession = Depends(get_db)):
            ...
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()
