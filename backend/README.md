---
title: Robotics Book Backend API
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: docker
pinned: false
---

# Physical AI Textbook - Backend API

FastAPI backend with authentication and RAG (Retrieval-Augmented Generation) for the Physical AI & Humanoid Robotics textbook.

## Features

- **Authentication**: User signup, login, and JWT-based authentication
- **RAG (Retrieval-Augmented Generation)**: Context-aware chatbot using Qdrant vector database
- **Personalized Learning**: Adapts responses based on user skill level
- **PostgreSQL Database**: User management and progress tracking using Neon
- **Vector Search**: Semantic search using Qdrant and OpenAI embeddings

## Setup

### 1. Prerequisites

- Python 3.9+
- PostgreSQL database (Neon recommended)
- Qdrant Cloud account
- OpenAI API key

### 2. Environment Variables

Copy `.env` file and configure:

```bash
OPENAI_API_KEY=your_openai_key
NEON_DATABASE_URL=postgresql://user:password@host/database?sslmode=require
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
BETTER_AUTH_SECRET=your_secret_key
```

### 3. Install Dependencies

```bash
cd backend
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 4. Setup Database and Vector Store

```bash
python setup_database.py
```

This will:
- Create database tables (users, progress_records, chat_messages, translation_cache)
- Create Qdrant collections for embeddings

### 5. Ingest Content (Optional)

To enable RAG, ingest your documentation:

```bash
python -m src.scripts.ingest_content
```

This will:
- Read markdown files from `docs/` directory
- Generate embeddings using OpenAI
- Store vectors in Qdrant

### 6. Run the Server

```bash
uvicorn src.main:app --reload --port 8000
```

Server will be available at: `http://localhost:8000`

## API Endpoints

### Authentication

#### Signup
```http
POST /api/auth/signup
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword",
  "name": "John Doe",
  "software_level": "Intermediate",
  "hardware_level": "Beginner",
  "learning_goals": "Learn ROS 2 and robotics"
}
```

Response:
```json
{
  "access_token": "eyJhbGc...",
  "token_type": "bearer",
  "user": {
    "id": "uuid",
    "email": "user@example.com",
    "name": "John Doe",
    "software_level": "Intermediate",
    "hardware_level": "Beginner",
    "learning_goals": "Learn ROS 2 and robotics",
    "created_at": "2024-01-01T00:00:00",
    "updated_at": "2024-01-01T00:00:00"
  }
}
```

#### Login
```http
POST /api/auth/login
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "securepassword"
}
```

#### Get Current User
```http
GET /api/auth/me
Authorization: Bearer <token>
```

### Chatbot (with RAG)

#### Ask Question (Public)
```http
POST /api/chatbot/ask
Content-Type: application/json

{
  "message": "What is a ROS 2 node?",
  "chapter": "chapter_1",
  "use_rag": true
}
```

Response:
```json
{
  "response": "A ROS 2 node is...",
  "success": true,
  "sources": [
    {
      "content": "...",
      "section": "Introduction to Nodes",
      "chapter": "chapter_1",
      "score": 0.89
    }
  ],
  "num_sources": 3
}
```

#### Personalized Ask (Authenticated)
```http
POST /api/chatbot/personalized
Authorization: Bearer <token>
Content-Type: application/json

{
  "message": "Explain topics and subscribers",
  "chapter": "chapter_1",
  "use_rag": true
}
```

Response adapts to user's skill level from their profile.

## Architecture

### Database Schema (PostgreSQL/Neon)

**users**
- id (UUID, primary key)
- email (unique)
- password_hash
- name
- software_level (Beginner/Intermediate/Advanced)
- hardware_level (Beginner/Intermediate/Advanced)
- learning_goals (text)
- created_at, updated_at

**progress_records**
- id (UUID, primary key)
- user_id (foreign key)
- chapter_id
- sections_completed (array)
- quiz_score
- completion_date

**chat_messages**
- id (UUID, primary key)
- session_id
- user_id (nullable)
- chapter_id
- user_question
- bot_response
- context_used (JSONB)
- timestamp

### Vector Store (Qdrant)

Collections:
- `chapter_1_embeddings`: Chapter 1 content embeddings
- `chapter_2_embeddings`: Chapter 2 content embeddings

Point payload:
```json
{
  "content": "text chunk",
  "section": "section title",
  "section_level": 2,
  "chapter": "chapter_1",
  "file_path": "docs/intro.md",
  "chunk_index": 0,
  "total_chunks": 5,
  "metadata": {
    "source": "intro.md",
    "section_index": 1
  }
}
```

## RAG Pipeline

1. **User Query** → Generate embedding using OpenAI
2. **Vector Search** → Find similar content in Qdrant
3. **Context Building** → Combine top results
4. **LLM Completion** → Generate answer with context
5. **Response** → Return answer with sources

### Personalization

RAG adapts based on user skill level:
- **Beginner**: Lower threshold (0.65), more context
- **Intermediate**: Balanced (0.70)
- **Advanced**: Higher threshold (0.75), focused context

## Development

### Project Structure

```
backend/
├── src/
│   ├── auth/              # Authentication middleware
│   ├── models/            # Pydantic models
│   ├── routes/            # API routes
│   ├── services/          # Business logic (auth, RAG)
│   ├── scripts/           # Utility scripts
│   ├── ai_client.py       # OpenAI integration
│   ├── config.py          # Settings
│   ├── database.py        # Database connection
│   ├── main.py            # FastAPI app
│   └── vector_store.py    # Qdrant integration
├── setup_database.py      # Database setup script
├── requirements.txt       # Python dependencies
└── README.md
```

### Testing

Run the server and test endpoints:

```bash
# Health check
curl http://localhost:8000/health

# Signup
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"testpass123","name":"Test User","software_level":"Beginner","hardware_level":"Beginner"}'

# Ask question
curl -X POST http://localhost:8000/api/chatbot/ask \
  -H "Content-Type: application/json" \
  -d '{"message":"What is ROS 2?","use_rag":true}'
```

## Troubleshooting

### Database Connection Error
- Check `NEON_DATABASE_URL` format: `postgresql://user:password@host/database?sslmode=require`
- Ensure Neon database is accessible

### Qdrant Connection Error
- Verify `QDRANT_URL` and `QDRANT_API_KEY`
- Check Qdrant cluster is running

### RAG Returns No Results
- Run content ingestion: `python -m src.scripts.ingest_content`
- Ensure markdown files exist in `docs/` directory

## License

MIT
