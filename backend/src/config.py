"""
Configuration management for RAG Chatbot Backend.
Loads and validates environment variables from .env file.
"""
import os
from typing import Optional, List
from pydantic_settings import BaseSettings
from pydantic import Field, field_validator
from dotenv import load_dotenv

# Load .env file explicitly
load_dotenv()


class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Qdrant Vector Database
    qdrant_api_key: str = Field(..., env="QDRANT_API_KEY")
    qdrant_cluster_id: str = Field(..., env="QDRANT_CLUSTER_ID")
    qdrant_url: str = Field(..., env="QDRANT_URL")
    qdrant_collection_name: str = Field(default="book_embeddings", env="QDRANT_COLLECTION_NAME")

    # Neon Serverless Postgres
    database_url: str = Field(..., env="DATABASE_URL")
    db_host: Optional[str] = Field(None, env="DB_HOST")
    db_name: Optional[str] = Field(None, env="DB_NAME")
    db_user: Optional[str] = Field(None, env="DB_USER")
    db_password: Optional[str] = Field(None, env="DB_PASSWORD")
    db_port: int = Field(default=5432, env="DB_PORT")
    db_ssl_mode: str = Field(default="require", env="DB_SSL_MODE")

    # OpenAI API
    openai_api_key: str = Field(..., env="OPENAI_API_KEY")
    openai_model: str = Field(default="gpt-4-turbo-preview", env="OPENAI_MODEL")
    openai_embedding_model: str = Field(default="text-embedding-3-small", env="OPENAI_EMBEDDING_MODEL")

    # Cohere API (for embedding pipeline)
    cohere_api_key: Optional[str] = Field(None, env="COHERE_API_KEY")

    # Application Settings
    environment: str = Field(default="development", env="ENVIRONMENT")
    api_host: str = Field(default="0.0.0.0", env="API_HOST")
    api_port: int = Field(default=8000, env="API_PORT")
    api_reload: bool = Field(default=True, env="API_RELOAD")

    # CORS Settings
    cors_origins: str = Field(default="http://localhost:3000,http://localhost:8000", env="CORS_ORIGINS")

    # Security
    rate_limit_per_minute: int = Field(default=100, env="RATE_LIMIT_PER_MINUTE")
    admin_api_key: str = Field(default="change-this-in-production", env="ADMIN_API_KEY")

    # Logging
    log_level: str = Field(default="INFO", env="LOG_LEVEL")

    # RAG Pipeline Settings
    chunk_size: int = Field(default=1000, env="CHUNK_SIZE")
    chunk_overlap: int = Field(default=200, env="CHUNK_OVERLAP")
    top_k_results: int = Field(default=5, env="TOP_K_RESULTS")
    similarity_threshold: float = Field(default=0.7, env="SIMILARITY_THRESHOLD")
    max_response_tokens: int = Field(default=500, env="MAX_RESPONSE_TOKENS")
    temperature: float = Field(default=0.7, env="TEMPERATURE")

    # Feature Flags
    enable_streaming: bool = Field(default=True, env="ENABLE_STREAMING")
    enable_chat_history: bool = Field(default=True, env="ENABLE_CHAT_HISTORY")
    enable_feedback: bool = Field(default=True, env="ENABLE_FEEDBACK")
    enable_selected_text_query: bool = Field(default=True, env="ENABLE_SELECTED_TEXT_QUERY")
    enable_chapter_scoping: bool = Field(default=True, env="ENABLE_CHAPTER_SCOPING")

    class Config:
        env_file = "../.env"  # Look for .env in parent directory
        env_file_encoding = "utf-8"
        case_sensitive = False


# Global settings instance
settings = Settings()


def validate_settings():
    """
    Validate that all required settings are present.
    Called at application startup to fail fast if configuration is invalid.
    """
    required_vars = [
        "qdrant_api_key",
        "qdrant_url",
        "database_url",
        "openai_api_key",
    ]

    missing = []
    for var in required_vars:
        if not getattr(settings, var, None):
            missing.append(var.upper())

    if missing:
        raise ValueError(
            f"Missing required environment variables: {', '.join(missing)}. "
            f"Please check your .env file."
        )

    print("[OK] Configuration validated successfully")
    print(f"   - Environment: {settings.environment}")
    print(f"   - Qdrant URL: {settings.qdrant_url}")
    print(f"   - Database: Connected")
    print(f"   - OpenAI Model: {settings.openai_model}")
