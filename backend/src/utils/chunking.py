"""Text chunking utilities for RAG system."""
import re
from typing import List, Tuple
from src.config import settings
from src.utils.logger import logger


def estimate_tokens(text: str) -> int:
    """
    Estimate token count for text.
    Rough approximation: 1 token ≈ 4 characters for English text.
    """
    return len(text) // 4


def chunk_text(
    text: str,
    chunk_size: int = None,
    chunk_overlap: int = None,
    preserve_sentences: bool = True,
) -> List[Tuple[str, int, int]]:
    """
    Split text into overlapping chunks.

    Args:
        text: Input text to chunk
        chunk_size: Maximum tokens per chunk (default from settings)
        chunk_overlap: Overlapping tokens between chunks (default from settings)
        preserve_sentences: Try to break at sentence boundaries

    Returns:
        List of tuples: (chunk_text, start_char, end_char)
    """
    chunk_size = chunk_size or settings.chunk_size
    chunk_overlap = chunk_overlap or settings.chunk_overlap

    # Convert token limits to character estimates
    max_chars = chunk_size * 4
    overlap_chars = chunk_overlap * 4

    if preserve_sentences:
        # Split into sentences first
        sentences = split_into_sentences(text)
        chunks = []
        current_chunk = []
        current_length = 0
        start_pos = 0

        for sentence in sentences:
            sentence_length = len(sentence)

            # If adding this sentence exceeds chunk size
            if current_length + sentence_length > max_chars and current_chunk:
                # Finalize current chunk
                chunk_text = " ".join(current_chunk)
                end_pos = start_pos + len(chunk_text)
                chunks.append((chunk_text, start_pos, end_pos))

                # Start new chunk with overlap
                overlap_text = ""
                overlap_length = 0
                for s in reversed(current_chunk):
                    if overlap_length + len(s) <= overlap_chars:
                        overlap_text = s + " " + overlap_text
                        overlap_length += len(s)
                    else:
                        break

                current_chunk = [overlap_text.strip()] if overlap_text.strip() else []
                current_length = len(overlap_text)
                start_pos = end_pos - current_length

            current_chunk.append(sentence)
            current_length += sentence_length + 1  # +1 for space

        # Add final chunk
        if current_chunk:
            chunk_text = " ".join(current_chunk)
            end_pos = start_pos + len(chunk_text)
            chunks.append((chunk_text, start_pos, end_pos))

    else:
        # Simple sliding window without sentence awareness
        chunks = []
        start = 0
        text_length = len(text)

        while start < text_length:
            end = min(start + max_chars, text_length)
            chunk = text[start:end]
            chunks.append((chunk, start, end))
            start += max_chars - overlap_chars

    logger.info(
        f"Created {len(chunks)} chunks from {len(text)} characters "
        f"(~{estimate_tokens(text)} tokens)"
    )

    return chunks


def split_into_sentences(text: str) -> List[str]:
    """
    Split text into sentences.
    Handles common sentence boundaries while avoiding false splits.
    """
    # Handle common abbreviations that shouldn't trigger splits
    text = text.replace("Dr.", "Dr<prd>")
    text = text.replace("Mr.", "Mr<prd>")
    text = text.replace("Mrs.", "Mrs<prd>")
    text = text.replace("Ms.", "Ms<prd>")
    text = text.replace("Jr.", "Jr<prd>")
    text = text.replace("Sr.", "Sr<prd>")
    text = text.replace("e.g.", "e<prd>g<prd>")
    text = text.replace("i.e.", "i<prd>e<prd>")
    text = text.replace("etc.", "etc<prd>")

    # Split on sentence boundaries
    sentences = re.split(r"(?<=[.!?])\s+", text)

    # Restore abbreviations
    sentences = [s.replace("<prd>", ".") for s in sentences]

    return [s.strip() for s in sentences if s.strip()]


def extract_markdown_sections(markdown_text: str) -> List[dict]:
    """
    Extract sections from markdown text based on headings.

    Args:
        markdown_text: Markdown formatted text

    Returns:
        List of dicts with 'heading', 'level', 'content', 'start', 'end'
    """
    sections = []
    lines = markdown_text.split("\n")
    current_section = None
    start_pos = 0
    current_pos = 0

    for line in lines:
        line_length = len(line) + 1  # +1 for newline

        # Check if line is a heading
        heading_match = re.match(r"^(#{1,6})\s+(.+)$", line)

        if heading_match:
            # Save previous section
            if current_section:
                current_section["end"] = current_pos
                sections.append(current_section)

            # Start new section
            level = len(heading_match.group(1))
            heading = heading_match.group(2).strip()
            current_section = {
                "heading": heading,
                "level": level,
                "content": "",
                "start": current_pos,
                "end": None,
            }
            start_pos = current_pos + line_length
        elif current_section:
            # Add line to current section
            current_section["content"] += line + "\n"

        current_pos += line_length

    # Add final section
    if current_section:
        current_section["end"] = current_pos
        sections.append(current_section)

    logger.info(f"Extracted {len(sections)} sections from markdown")
    return sections


def chunk_by_section(
    markdown_text: str,
    max_chunk_size: int = None,
) -> List[dict]:
    """
    Chunk markdown text respecting section boundaries.

    Args:
        markdown_text: Markdown formatted text
        max_chunk_size: Maximum tokens per chunk

    Returns:
        List of dicts with 'text', 'heading', 'level', 'chunk_index'
    """
    max_chunk_size = max_chunk_size or settings.chunk_size
    max_chars = max_chunk_size * 4

    sections = extract_markdown_sections(markdown_text)
    chunks = []
    chunk_index = 0

    for section in sections:
        section_content = section["content"].strip()

        if not section_content:
            continue

        # If section is small enough, keep as single chunk
        if len(section_content) <= max_chars:
            chunks.append({
                "text": section_content,
                "heading": section["heading"],
                "level": section["level"],
                "chunk_index": chunk_index,
            })
            chunk_index += 1
        else:
            # Split large sections into sub-chunks
            sub_chunks = chunk_text(section_content, preserve_sentences=True)
            for sub_chunk, _, _ in sub_chunks:
                chunks.append({
                    "text": sub_chunk,
                    "heading": section["heading"],
                    "level": section["level"],
                    "chunk_index": chunk_index,
                })
                chunk_index += 1

    logger.info(f"Created {len(chunks)} chunks from {len(sections)} sections")
    return chunks
