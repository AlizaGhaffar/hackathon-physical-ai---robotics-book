"""Logging configuration for the application."""
import logging
import sys
from src.config import settings


def setup_logger(name: str = "rag_backend") -> logging.Logger:
    """
    Setup structured logger for the application.

    Args:
        name: Logger name

    Returns:
        Configured logger instance
    """
    logger = logging.getLogger(name)
    logger.setLevel(getattr(logging, settings.log_level.upper()))

    # Console handler
    handler = logging.StreamHandler(sys.stdout)
    handler.setLevel(getattr(logging, settings.log_level.upper()))

    # Formatter
    formatter = logging.Formatter(
        fmt="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )
    handler.setFormatter(formatter)

    # Add handler if not already added
    if not logger.handlers:
        logger.addHandler(handler)

    return logger


# Global logger instance
logger = setup_logger()
