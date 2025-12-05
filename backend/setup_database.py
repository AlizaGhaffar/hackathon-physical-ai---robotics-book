"""
Database Setup Script

Run this script to create database tables and Qdrant collections
"""

import sys
from pathlib import Path

# Add src to path
sys.path.append(str(Path(__file__).parent / "src"))

from src.database import create_tables
from src.vector_store import setup_qdrant_collection, create_chapter_2_collection

def main():
    print("Setting up database and vector store...\n")

    # Create database tables
    print("Creating database tables...")
    try:
        create_tables()
    except Exception as e:
        print(f"Database error: {e}")
        print("\nMake sure your NEON_DATABASE_URL is correct in .env file")
        return

    # Create Qdrant collections
    print("\nCreating Qdrant collections...")
    try:
        setup_qdrant_collection()
        create_chapter_2_collection()
    except Exception as e:
        print(f"Qdrant error: {e}")
        print("\nMake sure your QDRANT_URL and QDRANT_API_KEY are correct in .env file")
        return

    print("\n[SUCCESS] Setup complete!")
    print("\nNext steps:")
    print("1. Run content ingestion: python -m src.scripts.ingest_content")
    print("2. Start the server: uvicorn src.main:app --reload --port 8000")

if __name__ == "__main__":
    main()
