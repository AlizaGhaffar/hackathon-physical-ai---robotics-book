from pydantic_settings import BaseSettings
from typing import Optional

class Settings(BaseSettings):
    # OpenAI
    openai_api_key: str

    # Neon Postgres
    neon_database_url: str

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str

    # Better Auth
    better_auth_secret: str

    # API
    api_url: str = "http://localhost:8000"

    # Environment
    environment: str = "development"

    class Config:
        env_file = ".env"
        case_sensitive = False
        extra = "ignore"  # Ignore extra fields in .env

settings = Settings()
