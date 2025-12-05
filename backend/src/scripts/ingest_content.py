"""
Content Ingestion Script for Qdrant Vector Database

This script ingests documentation content into Qdrant for RAG (Retrieval-Augmented Generation).
It reads markdown files, splits them into chunks, generates embeddings, and stores them in Qdrant.
"""

import os
import sys
from pathlib import Path
from typing import List, Dict
import hashlib
from uuid import uuid4

# Add parent directory to path
sys.path.append(str(Path(__file__).parent.parent.parent))

from src.ai_client import generate_embedding
from src.vector_store import get_qdrant_client, setup_qdrant_collection
from qdrant_client.models import PointStruct

def chunk_text(text: str, chunk_size: int = 500, overlap: int = 50) -> List[str]:
    """
    Split text into overlapping chunks

    Args:
        text: Text to chunk
        chunk_size: Size of each chunk in characters
        overlap: Number of overlapping characters between chunks

    Returns:
        List of text chunks
    """
    chunks = []
    start = 0

    while start < len(text):
        end = start + chunk_size
        chunk = text[start:end]

        # Try to break at sentence boundary
        if end < len(text):
            last_period = chunk.rfind('.')
            last_newline = chunk.rfind('\n')
            break_point = max(last_period, last_newline)

            if break_point > chunk_size // 2:  # Only break if it's not too early
                chunk = chunk[:break_point + 1]
                end = start + break_point + 1

        chunks.append(chunk.strip())
        start = end - overlap

    return chunks

def extract_sections_from_markdown(file_path: str) -> List[Dict]:
    """
    Extract sections from a markdown file

    Args:
        file_path: Path to markdown file

    Returns:
        List of sections with metadata
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    sections = []
    current_section = {"title": "Introduction", "content": "", "level": 1}
    lines = content.split('\n')

    for line in lines:
        # Check if line is a heading
        if line.startswith('#'):
            # Save previous section if it has content
            if current_section["content"].strip():
                sections.append(current_section.copy())

            # Start new section
            level = len(line) - len(line.lstrip('#'))
            title = line.lstrip('#').strip()
            current_section = {"title": title, "content": "", "level": level}
        else:
            current_section["content"] += line + "\n"

    # Add last section
    if current_section["content"].strip():
        sections.append(current_section)

    return sections

def ingest_markdown_file(
    file_path: str,
    chapter_id: str,
    collection_name: str = "chapter_1_embeddings"
):
    """
    Ingest a markdown file into Qdrant

    Args:
        file_path: Path to markdown file
        chapter_id: Chapter identifier (e.g., "chapter_1")
        collection_name: Name of Qdrant collection
    """
    print(f"\n📄 Processing: {file_path}")

    # Extract sections
    sections = extract_sections_from_markdown(file_path)
    print(f"   Found {len(sections)} sections")

    client = get_qdrant_client()
    points = []

    for idx, section in enumerate(sections):
        # Chunk the section content
        chunks = chunk_text(section["content"], chunk_size=600, overlap=100)

        print(f"   Section '{section['title']}': {len(chunks)} chunks")

        for chunk_idx, chunk in enumerate(chunks):
            if len(chunk.strip()) < 50:  # Skip very small chunks
                continue

            # Generate embedding
            try:
                embedding = generate_embedding(chunk)
            except Exception as e:
                print(f"   ⚠️  Error generating embedding: {e}")
                continue

            # Create unique ID
            content_hash = hashlib.md5(chunk.encode()).hexdigest()
            point_id = str(uuid4())

            # Create point
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    "content": chunk,
                    "section": section["title"],
                    "section_level": section["level"],
                    "chapter": chapter_id,
                    "file_path": file_path,
                    "chunk_index": chunk_idx,
                    "total_chunks": len(chunks),
                    "content_hash": content_hash,
                    "metadata": {
                        "source": os.path.basename(file_path),
                        "section_index": idx
                    }
                }
            )

            points.append(point)

    # Upload to Qdrant in batches
    batch_size = 100
    for i in range(0, len(points), batch_size):
        batch = points[i:i + batch_size]
        client.upsert(
            collection_name=collection_name,
            points=batch
        )

    print(f"✅ Ingested {len(points)} chunks from {file_path}")

def ingest_directory(
    docs_dir: str,
    chapter_id: str = "chapter_1",
    collection_name: str = "chapter_1_embeddings"
):
    """
    Ingest all markdown files from a directory

    Args:
        docs_dir: Directory containing markdown files
        chapter_id: Chapter identifier
        collection_name: Name of Qdrant collection
    """
    print(f"\n🚀 Starting content ingestion for {chapter_id}")
    print(f"   Directory: {docs_dir}")
    print(f"   Collection: {collection_name}")

    # Find all markdown files
    docs_path = Path(docs_dir)
    md_files = list(docs_path.glob("**/*.md"))

    if not md_files:
        print(f"⚠️  No markdown files found in {docs_dir}")
        return

    print(f"\n📚 Found {len(md_files)} markdown files")

    for md_file in md_files:
        ingest_markdown_file(
            file_path=str(md_file),
            chapter_id=chapter_id,
            collection_name=collection_name
        )

    print(f"\n✅ Ingestion complete! Processed {len(md_files)} files")

def main():
    """Main ingestion function"""
    # Ensure Qdrant collection exists
    print("🔧 Setting up Qdrant collection...")
    setup_qdrant_collection()

    # Get docs directory
    project_root = Path(__file__).parent.parent.parent.parent
    docs_dir = project_root / "docs"

    if not docs_dir.exists():
        print(f"❌ Docs directory not found: {docs_dir}")
        print("   Please create a 'docs' directory with markdown files")
        return

    # Ingest content
    ingest_directory(
        docs_dir=str(docs_dir),
        chapter_id="chapter_1",
        collection_name="chapter_1_embeddings"
    )

if __name__ == "__main__":
    main()
