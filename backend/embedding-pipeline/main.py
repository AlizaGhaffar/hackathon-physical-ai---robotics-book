"""
Embedding Pipeline for Physical AI Robotics Book

This script crawls the deployed Docusaurus site, extracts text,
generates embeddings using Cohere, and stores them in Qdrant.
"""

import asyncio
import hashlib
import logging
import os
import re
import time
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import List, Optional, Tuple
from urllib.parse import urljoin, urlparse

import cohere
import httpx
from bs4 import BeautifulSoup
from dotenv import load_dotenv
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, PointStruct, VectorParams

# Load environment variables
load_dotenv()

# Configuration
BASE_URL = os.getenv("BASE_URL", "https://physical-ai-robotics-book.vercel.app/")
COLLECTION_NAME = os.getenv("COLLECTION_NAME", "book_embeddings")
CHUNK_SIZE = int(os.getenv("CHUNK_SIZE", "500"))
CHUNK_OVERLAP = int(os.getenv("CHUNK_OVERLAP", "100"))
BATCH_SIZE = int(os.getenv("BATCH_SIZE", "100"))
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")

# Configure logging
logging.basicConfig(
    level=getattr(logging, LOG_LEVEL),
    format="%(asctime)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


# ============================================================================
# Phase 2: Foundational - Data Models
# ============================================================================


@dataclass
class PageMetadata:
    """Metadata extracted from a Docusaurus page."""

    chapter: str
    page_title: str
    section: Optional[str] = None
    breadcrumbs: Optional[List[str]] = None
    heading_hierarchy: Optional[List[str]] = None


@dataclass
class DocumentPage:
    """Represents a single crawled page with raw HTML and extracted text."""

    url: str
    html_content: str
    clean_text: str
    metadata: PageMetadata
    crawled_at: datetime
    content_hash: str

    def __post_init__(self):
        assert self.url.startswith("http"), "URL must be HTTP/HTTPS"
        assert len(self.clean_text) > 0, "Extracted text cannot be empty"


@dataclass
class ChunkMetadata:
    """Metadata attached to each text chunk for retrieval and filtering."""

    chapter: str
    page_title: str
    source_url: str
    chunk_id: str
    created_at: str
    section: Optional[str] = None
    chunk_index: int = 0


@dataclass
class TextChunk:
    """Represents a segment of text with metadata, ready for embedding."""

    text: str
    chunk_index: int
    metadata: ChunkMetadata
    token_count: int

    def __post_init__(self):
        assert 10 <= len(self.text) <= 2000, f"Text length {len(self.text)} out of range (10-2000)"
        assert self.chunk_index >= 0, "Chunk index must be non-negative"
        assert self.token_count <= 600, f"Token count {self.token_count} exceeds limit (600)"


# ============================================================================
# Phase 2: Foundational - Client Initialization
# ============================================================================


def initialize_clients() -> Tuple[cohere.Client, QdrantClient]:
    """Initialize Cohere and Qdrant clients with API keys."""
    cohere_api_key = os.getenv("COHERE_API_KEY")
    qdrant_url = os.getenv("QDRANT_URL")
    qdrant_api_key = os.getenv("QDRANT_API_KEY")

    if not cohere_api_key:
        raise ValueError("COHERE_API_KEY not found in environment variables")
    if not qdrant_url:
        raise ValueError("QDRANT_URL not found in environment variables")
    if not qdrant_api_key:
        raise ValueError("QDRANT_API_KEY not found in environment variables")

    cohere_client = cohere.Client(api_key=cohere_api_key)
    qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

    logger.info("Initialized Cohere and Qdrant clients")
    return cohere_client, qdrant_client


# ============================================================================
# Phase 2: Foundational - Retry Logic
# ============================================================================


async def retry_with_backoff(func, *args, max_retries=3, **kwargs):
    """Retry function with exponential backoff."""
    for attempt in range(max_retries):
        try:
            if asyncio.iscoroutinefunction(func):
                return await func(*args, **kwargs)
            else:
                return func(*args, **kwargs)
        except Exception as e:
            if attempt == max_retries - 1:
                raise
            wait_time = 2**attempt
            logger.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {wait_time}s...")
            await asyncio.sleep(wait_time)


# ============================================================================
# Phase 3: User Story 1 - URL Crawling
# ============================================================================


async def get_all_urls(base_url: str) -> List[str]:
    """
    Crawl Docusaurus site and return all documentation page URLs.

    Args:
        base_url: Root URL (e.g., "https://physical-ai-robotics-book.vercel.app/")

    Returns:
        List of absolute URLs to documentation pages
    """
    logger.info(f"Step 1/6: Crawling URLs from {base_url}...")

    urls = set()
    sitemap_url = urljoin(base_url, "sitemap.xml")

    async with httpx.AsyncClient(timeout=30.0) as client:
        try:
            # Try fetching sitemap.xml first
            response = await retry_with_backoff(client.get, sitemap_url)
            if response.status_code == 200:
                soup = BeautifulSoup(response.text, "xml")
                loc_tags = soup.find_all("loc")
                for loc in loc_tags:
                    url = loc.text.strip()
                    # Filter: exclude /blog, /tags, etc.
                    if not any(
                        excluded in url for excluded in ["/blog", "/tags", "/api-reference"]
                    ):
                        urls.add(url)
                logger.info(f"   Found {len(urls)} URLs from sitemap.xml")
            else:
                logger.warning(f"Sitemap not found ({response.status_code}), using fallback crawling")
                # Fallback: crawl from homepage
                urls = await _crawl_from_homepage(base_url, client)
        except Exception as e:
            logger.error(f"Error fetching sitemap: {e}")
            urls = await _crawl_from_homepage(base_url, client)

    return sorted(list(urls))


async def _crawl_from_homepage(base_url: str, client: httpx.AsyncClient) -> set:
    """Fallback: Crawl navigation links from homepage."""
    urls = set()
    try:
        response = await client.get(base_url)
        soup = BeautifulSoup(response.text, "html.parser")

        # Find all internal links
        for link in soup.find_all("a", href=True):
            href = link["href"]
            absolute_url = urljoin(base_url, href)
            parsed = urlparse(absolute_url)

            # Only include same-domain links
            if parsed.netloc == urlparse(base_url).netloc:
                # Exclude /blog, /tags
                if not any(excluded in absolute_url for excluded in ["/blog", "/tags"]):
                    urls.add(absolute_url)

        logger.info(f"   Found {len(urls)} URLs via homepage crawling")
    except Exception as e:
        logger.error(f"Fallback crawling failed: {e}")

    return urls


# ============================================================================
# Phase 3: User Story 1 - Text Extraction
# ============================================================================


async def extract_text_from_url(url: str) -> Optional[DocumentPage]:
    """
    Fetch HTML and extract clean text with metadata.

    Args:
        url: Absolute URL to documentation page

    Returns:
        DocumentPage with clean_text and metadata extracted, or None if failed
    """
    async with httpx.AsyncClient(timeout=30.0) as client:
        try:
            response = await retry_with_backoff(client.get, url)
            if response.status_code != 200:
                logger.warning(f"Failed to fetch {url}: HTTP {response.status_code}")
                return None

            html_content = response.text
            soup = BeautifulSoup(html_content, "html.parser")

            # Remove unwanted elements
            for element in soup.find_all(["nav", "footer", "aside", "script", "style"]):
                element.decompose()

            # Find main content
            main_content = soup.find("article") or soup.find("main") or soup.find(class_="markdown")
            if not main_content:
                main_content = soup.body

            # Extract metadata
            page_title = ""
            h1 = soup.find("h1")
            if h1:
                page_title = h1.get_text(strip=True)
            else:
                title_tag = soup.find("title")
                if title_tag:
                    page_title = title_tag.get_text(strip=True)

            # Extract chapter from URL or breadcrumbs
            chapter = _extract_chapter(url, soup)
            section = None
            h2 = soup.find("h2")
            if h2:
                section = h2.get_text(strip=True)

            # Extract clean text
            clean_text = main_content.get_text(separator="\n", strip=True)
            clean_text = re.sub(r"\n{3,}", "\n\n", clean_text)  # Normalize whitespace

            if len(clean_text) < 10:
                logger.warning(f"Skipping {url}: extracted text too short")
                return None

            content_hash = hashlib.md5(clean_text.encode()).hexdigest()

            metadata = PageMetadata(
                chapter=chapter,
                page_title=page_title,
                section=section,
            )

            return DocumentPage(
                url=url,
                html_content=html_content,
                clean_text=clean_text,
                metadata=metadata,
                crawled_at=datetime.now(timezone.utc),
                content_hash=content_hash,
            )

        except Exception as e:
            logger.error(f"Error extracting text from {url}: {e}")
            return None


def _extract_chapter(url: str, soup: BeautifulSoup) -> str:
    """Extract chapter name from URL path or breadcrumbs."""
    # Try breadcrumbs first
    breadcrumb = soup.find("nav", {"aria-label": "breadcrumb"})
    if breadcrumb:
        items = breadcrumb.find_all("a")
        if items:
            return items[-1].get_text(strip=True)

    # Fallback: extract from URL path
    path = urlparse(url).path.strip("/")
    segments = path.split("/")
    if segments:
        return segments[0].replace("-", " ").title()

    return "Unknown Chapter"


# ============================================================================
# Phase 3: User Story 1 - Text Chunking
# ============================================================================


def chunk_text(document: DocumentPage, chunk_size: int = 500, overlap: int = 100) -> List[TextChunk]:
    """
    Split document text into overlapping chunks.

    Args:
        document: DocumentPage with clean_text
        chunk_size: Target chunk size in tokens (default: 500)
        overlap: Number of overlapping tokens between chunks (default: 100)

    Returns:
        List of TextChunk objects with metadata
    """
    text = document.clean_text
    # Estimate tokens: ~4 characters per token
    char_per_token = 4
    chunk_size_chars = chunk_size * char_per_token
    overlap_chars = overlap * char_per_token

    chunks = []
    chunk_index = 0

    # Split on paragraphs first
    paragraphs = text.split("\n\n")
    current_chunk = ""

    for para in paragraphs:
        if len(current_chunk) + len(para) < chunk_size_chars:
            current_chunk += para + "\n\n"
        else:
            # Save current chunk
            if current_chunk.strip():
                chunks.append(_create_chunk(document, current_chunk.strip(), chunk_index))
                chunk_index += 1

            # Start new chunk with overlap
            overlap_text = current_chunk[-overlap_chars:] if len(current_chunk) > overlap_chars else ""
            current_chunk = overlap_text + para + "\n\n"

    # Add last chunk
    if current_chunk.strip():
        chunks.append(_create_chunk(document, current_chunk.strip(), chunk_index))

    return chunks


# def _create_chunk(document: DocumentPage, text: str, chunk_index: int) -> TextChunk:
#     """Create a TextChunk with metadata."""
#     token_count = len(text) // 4  # Estimate

#     chunk_id = hashlib.md5(f"{document.url}::{chunk_index}".encode()).hexdigest()

#     metadata = ChunkMetadata(
#         chapter=document.metadata.chapter,
#         page_title=document.metadata.page_title,
#         source_url=document.url,
#         chunk_id=chunk_id,
#         created_at=datetime.now(timezone.utc).isoformat(),
#         section=document.metadata.section,
#         chunk_index=chunk_index,
#     )

#     return TextChunk(
#         text=text, chunk_index=chunk_index, metadata=metadata, token_count=token_count
#     )
def _create_chunk(document: DocumentPage, text: str, chunk_index: int) -> TextChunk:
    # HARD SAFETY TRIM
    if len(text) > 2000:
        text = text[:2000]

    token_count = len(text) // 4

    chunk_id = hashlib.md5(f"{document.url}::{chunk_index}".encode()).hexdigest()

    metadata = ChunkMetadata(
        chapter=document.metadata.chapter,
        page_title=document.metadata.page_title,
        source_url=document.url,
        chunk_id=chunk_id,
        created_at=datetime.now(timezone.utc).isoformat(),
        section=document.metadata.section,
        chunk_index=chunk_index,
    )

    return TextChunk(
        text=text,
        chunk_index=chunk_index,
        metadata=metadata,
        token_count=token_count,
    )


# ============================================================================
# Phase 4: User Story 2 - Embedding Generation
# ============================================================================


async def embed(cohere_client: cohere.Client, chunks: List[TextChunk]) -> List[Tuple[TextChunk, List[float]]]:
    """
    Generate embeddings for text chunks using Cohere.

    Args:
        cohere_client: Initialized Cohere client
        chunks: List of TextChunk objects

    Returns:
        List of (chunk, embedding_vector) tuples
    """
    if not chunks:
        return []

    chunk_texts = [chunk.text for chunk in chunks]

    try:
        # Batch embed with Cohere
        response = await retry_with_backoff(
            cohere_client.embed,
            texts=chunk_texts,
            model="embed-english-v3.0",
            input_type="search_document",
            truncate="END",
        )

        embeddings = response.embeddings

        # Validate dimensions
        for emb in embeddings:
            if len(emb) != 1024:
                raise ValueError(f"Expected 1024 dimensions, got {len(emb)}")

        result = list(zip(chunks, embeddings))
        logger.debug(f"   Embedded {len(chunks)} chunks")
        return result

    except Exception as e:
        logger.error(f"Embedding failed: {e}")
        raise


# ============================================================================
# Phase 5: User Story 3 - Qdrant Storage
# ============================================================================


async def create_collection(qdrant_client: QdrantClient, collection_name: str = "rag_embedding") -> None:
    """
    Create or verify Qdrant collection exists with correct config.

    Args:
        qdrant_client: Initialized QdrantClient
        collection_name: Name of collection (default: "rag_embedding")
    """
    try:
        collections = qdrant_client.get_collections()
        collection_names = [col.name for col in collections.collections]

        if collection_name in collection_names:
            logger.info(f"   Collection '{collection_name}' already exists")
        else:
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=VectorParams(
                    size=1024,  # Cohere embed-english-v3.0 dimension
                    distance=Distance.COSINE,
                ),
            )
            logger.info(f"   Created collection '{collection_name}'")

    except Exception as e:
        logger.error(f"Failed to create collection: {e}")
        raise


async def save_chunk_to_qdrant(
    qdrant_client: QdrantClient,
    chunks_with_embeddings: List[Tuple[TextChunk, List[float]]],
    collection_name: str = "rag_embedding",
) -> int:
    """
    Batch upsert chunks with embeddings to Qdrant.

    Args:
        qdrant_client: Initialized QdrantClient
        chunks_with_embeddings: List of (chunk, vector) tuples
        collection_name: Target collection name

    Returns:
        Number of vectors successfully upserted
    """
    if not chunks_with_embeddings:
        return 0

    points = []
    for chunk, embedding in chunks_with_embeddings:
        point = PointStruct(
            id=chunk.metadata.chunk_id,
            vector=embedding,
            payload={
                "chapter": chunk.metadata.chapter,
                "section": chunk.metadata.section,
                "page_title": chunk.metadata.page_title,
                "source_url": chunk.metadata.source_url,
                "text": chunk.text,
                "chunk_index": chunk.chunk_index,
                "created_at": chunk.metadata.created_at,
            },
        )
        points.append(point)

    try:
        qdrant_client.upsert(collection_name=collection_name, points=points)
        logger.debug(f"   Upserted {len(points)} vectors")
        return len(points)
    except Exception as e:
        logger.error(f"Qdrant upsert failed: {e}")
        raise


# ============================================================================
# Phase 6: Pipeline Orchestration
# ============================================================================


async def main():
    """Orchestrate entire pipeline: crawl → extract → chunk → embed → store."""
    start_time = time.time()

    print("Starting Embedding Pipeline...")
    print(f"Target URL: {BASE_URL}")
    print(f"Collection: {COLLECTION_NAME}")
    print()

    try:
        # Initialize clients
        cohere_client, qdrant_client = initialize_clients()

        # Step 1: Crawl URLs
        urls = await get_all_urls(BASE_URL)
        print(f"   Found {len(urls)} documentation pages\n")

        # Step 2: Extract text from pages
        print(f"Step 2/6: Extracting text from pages...")
        documents = []
        for i, url in enumerate(urls, 1):
            doc = await extract_text_from_url(url)
            if doc:
                documents.append(doc)
            if i % 10 == 0:
                print(f"   Progress: {i}/{len(urls)} pages processed")

        print(f"   Processed {len(documents)} pages (100%)\n")

        # Step 3: Chunk text
        print(f"Step 3/6: Chunking text...")
        all_chunks = []
        for doc in documents:
            chunks = chunk_text(doc, CHUNK_SIZE, CHUNK_OVERLAP)
            all_chunks.extend(chunks)

        print(f"   Generated {len(all_chunks)} chunks\n")

        # Step 4: Generate embeddings
        print(f"Step 4/6: Generating embeddings...")
        all_embeddings = []
        for i in range(0, len(all_chunks), BATCH_SIZE):
            batch = all_chunks[i : i + BATCH_SIZE]
            embeddings = await embed(cohere_client, batch)
            all_embeddings.extend(embeddings)
            print(f"   Progress: {min(i + BATCH_SIZE, len(all_chunks))}/{len(all_chunks)} chunks embedded")

        print(f"   Embedded {len(all_embeddings)} chunks (batched)\n")

        # Step 5: Create Qdrant collection
        print(f"Step 5/6: Creating Qdrant collection...")
        await create_collection(qdrant_client, COLLECTION_NAME)
        print()

        # Step 6: Store vectors in Qdrant
        print(f"Step 6/6: Storing vectors in Qdrant...")
        total_stored = 0
        for i in range(0, len(all_embeddings), BATCH_SIZE):
            batch = all_embeddings[i : i + BATCH_SIZE]
            stored = await save_chunk_to_qdrant(qdrant_client, batch, COLLECTION_NAME)
            total_stored += stored
            print(f"   Progress: {min(i + BATCH_SIZE, len(all_embeddings))}/{len(all_embeddings)} vectors stored")

        print(f"   Upserted {total_stored} vectors\n")

        # Summary
        elapsed = time.time() - start_time
        minutes = int(elapsed // 60)
        seconds = int(elapsed % 60)

        print("Pipeline execution complete!")
        print(f"   Total time: {minutes}m {seconds}s")
        print(f"   Stats: {len(documents)} pages, {len(all_chunks)} chunks, {total_stored} vectors")

    except Exception as e:
        logger.error(f"Pipeline failed: {e}", exc_info=True)
        print(f"\nPipeline failed: {e}")
        raise


if __name__ == "__main__":
    asyncio.run(main())
