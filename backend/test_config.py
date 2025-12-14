"""Test config loading"""
from src.config import settings

print(f"SIMILARITY_THRESHOLD from settings: {settings.similarity_threshold}")
print(f"TOP_K_RESULTS: {settings.top_k_results}")
print(f"CHUNK_SIZE: {settings.chunk_size}")
print(f"Environment: {settings.environment}")
print(f"\nAll RAG settings:")
print(f"  similarity_threshold: {settings.similarity_threshold}")
print(f"  top_k_results: {settings.top_k_results}")
print(f"  max_response_tokens: {settings.max_response_tokens}")
print(f"  temperature: {settings.temperature}")
