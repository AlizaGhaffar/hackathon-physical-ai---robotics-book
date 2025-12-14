from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv('QDRANT_URL'),
    api_key=os.getenv('QDRANT_API_KEY')
)

collection_name = os.getenv('QDRANT_COLLECTION_NAME', 'book_embeddings')

# Get first few points to inspect
points = client.scroll(
    collection_name=collection_name,
    limit=5,
    with_payload=True,
    with_vectors=False
)

print(f'Found {len(points[0])} points in collection')
print()

for idx, point in enumerate(points[0], 1):
    print(f'Point {idx}: {point.id}')
    if point.payload:
        # Print metadata keys first
        print(f'  Metadata keys: {list(point.payload.keys())}')

        # Print specific metadata
        for key in ['chapter', 'section', 'page_title', 'source_url', 'chapter_id']:
            if key in point.payload:
                value = str(point.payload[key])
                print(f'  {key}: {value}')

        # Show text length and preview
        if 'text' in point.payload:
            text = point.payload['text']
            print(f'  text_length: {len(text)} chars')
            # Safe preview - only ASCII
            preview = ''.join(c if ord(c) < 128 else '?' for c in text[:100])
            print(f'  text_preview: {preview}...')
    print()
