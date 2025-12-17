"""Input validation and sanitization utilities."""
import re
import uuid
from typing import Optional


def generate_request_id() -> str:
    """Generate a unique request ID for tracing."""
    return f"req_{str(uuid.uuid4())[:8]}"


def sanitize_input(text: str) -> str:
    """
    Sanitize user input to prevent injection attacks.

    Args:
        text: Input text to sanitize

    Returns:
        Sanitized text
    """
    if not text:
        return ""

    # Remove null bytes
    text = text.replace("\x00", "")

    # Remove control characters except newlines and tabs
    text = "".join(char for char in text if char.isprintable() or char in "\n\t")

    return text.strip()


def validate_chapter_id(chapter_id: str) -> bool:
    """
    Validate chapter ID format.

    Args:
        chapter_id: Chapter identifier (e.g., "chapter-1")

    Returns:
        True if valid format
    """
    pattern = r"^chapter-\d+$"
    return bool(re.match(pattern, chapter_id))


def detect_prompt_injection(text: str) -> bool:
    """
    Detect potential prompt injection attacks.

    Args:
        text: Input text to check

    Returns:
        True if suspicious patterns detected
    """
    # List of suspicious patterns
    suspicious_patterns = [
        r"ignore\s+(previous|above|all)\s+instructions",
        r"system\s*:\s*you\s+are",
        r"<\s*script",
        r"javascript\s*:",
        r"on(load|error|click)\s*=",
    ]

    text_lower = text.lower()
    for pattern in suspicious_patterns:
        if re.search(pattern, text_lower):
            return True

    return False
