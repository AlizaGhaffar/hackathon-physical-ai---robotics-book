# Phase 1 Completion Guide

## ✅ Completed Tasks (T001-T012)

### Frontend Setup
- ✅ T001: Created `package.json` with all dependencies (Docusaurus, React, Monaco Editor, React Three Fiber)
- ✅ T002-T003: Created `docusaurus.config.ts` with single-chapter configuration and GitHub Pages setup
- ✅ T004: Created `.gitignore` for Node.js and Python
- ✅ T006: Created directory structure (`src/`, `docs/chapter-1/`, `static/`)
- ✅ T008: Created `.env.example` with API key placeholders
- ✅ T009: Created `.eslintrc.json` and `.prettierrc.json`
- ✅ Created `sidebars.ts` for Chapter 1 navigation
- ✅ Created `tsconfig.json` for TypeScript configuration
- ✅ Created `src/css/custom.css` with styles for all interactive components

### Backend Setup
- ✅ T011: Created backend directory structure (`backend/src/`)
- ✅ T012: Created `backend/requirements.txt` with Python dependencies

## 🔧 Remaining Tasks to Complete Phase 1

### Step 1: Install Frontend Dependencies

```bash
# Navigate to project root
cd /c/Users/affil/Desktop/book

# Install Node.js dependencies (this will take 2-3 minutes)
npm install
```

**Expected output**: `node_modules/` folder with ~1000 packages

---

### Step 2: Install Backend Dependencies

```bash
# Create Python virtual environment
cd backend
python -m venv venv

# Activate virtual environment
# On Windows Git Bash:
source venv/Scripts/activate
# OR on Windows CMD:
# venv\Scripts\activate.bat

# Install Python packages
pip install -r requirements.txt
```

**Expected output**: All packages from requirements.txt installed

---

### Step 3: Create Backend Configuration Files

#### File 1: `backend/src/config.py` (T019)

```python
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

settings = Settings()
```

#### File 2: `backend/src/database.py` (T014-T015-T016)

```python
import psycopg2
from psycopg2.extras import RealDictCursor
from .config import settings

def get_db_connection():
    """Get database connection"""
    return psycopg2.connect(
        settings.neon_database_url,
        cursor_factory=RealDictCursor
    )

def create_tables():
    """Create database tables if they don't exist"""
    conn = get_db_connection()
    cursor = conn.cursor()

    # Users table (T015)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS users (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            email VARCHAR(255) UNIQUE NOT NULL,
            password_hash VARCHAR(255) NOT NULL,
            name VARCHAR(100) NOT NULL,
            software_level VARCHAR(20) NOT NULL CHECK (software_level IN ('Beginner', 'Intermediate', 'Advanced')),
            hardware_level VARCHAR(20) NOT NULL CHECK (hardware_level IN ('Beginner', 'Intermediate', 'Advanced')),
            learning_goals TEXT,
            created_at TIMESTAMP DEFAULT NOW(),
            updated_at TIMESTAMP DEFAULT NOW()
        )
    """)

    # Progress records table (T016)
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS progress_records (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
            chapter_id VARCHAR(50) NOT NULL,
            sections_completed TEXT[] DEFAULT '{}',
            quiz_score INTEGER CHECK (quiz_score >= 0 AND quiz_score <= 100),
            completion_date TIMESTAMP,
            updated_at TIMESTAMP DEFAULT NOW(),
            UNIQUE(user_id, chapter_id)
        )
    """)

    # Chat messages table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS chat_messages (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            session_id VARCHAR(100) NOT NULL,
            user_id UUID REFERENCES users(id) ON DELETE SET NULL,
            chapter_id VARCHAR(50) NOT NULL,
            user_question TEXT NOT NULL,
            bot_response TEXT NOT NULL,
            context_used JSONB,
            timestamp TIMESTAMP DEFAULT NOW()
        )
    """)

    # Translation cache table
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS translation_cache (
            id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
            original_text_hash VARCHAR(64) UNIQUE NOT NULL,
            original_text TEXT NOT NULL,
            translated_text TEXT NOT NULL,
            language VARCHAR(10) NOT NULL,
            created_at TIMESTAMP DEFAULT NOW()
        )
    """)

    conn.commit()
    cursor.close()
    conn.close()
    print("✅ Database tables created successfully")

if __name__ == "__main__":
    create_tables()
```

#### File 3: `backend/src/vector_store.py` (T017)

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams
from .config import settings

def get_qdrant_client():
    """Get Qdrant client"""
    return QdrantClient(
        url=settings.qdrant_url,
        api_key=settings.qdrant_api_key
    )

def setup_qdrant_collection():
    """Create Qdrant collection for Chapter 1 embeddings"""
    client = get_qdrant_client()

    collection_name = "chapter_1_embeddings"

    # Check if collection exists
    collections = client.get_collections().collections
    if collection_name not in [c.name for c in collections]:
        client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=1536,  # OpenAI text-embedding-ada-002
                distance=Distance.COSINE
            )
        )
        print(f"✅ Created Qdrant collection: {collection_name}")
    else:
        print(f"✓ Qdrant collection already exists: {collection_name}")

if __name__ == "__main__":
    setup_qdrant_collection()
```

#### File 4: `backend/src/ai_client.py` (T018)

```python
from openai import OpenAI
from .config import settings

def get_openai_client():
    """Get OpenAI client"""
    return OpenAI(api_key=settings.openai_api_key)

def generate_embedding(text: str):
    """Generate embedding for text"""
    client = get_openai_client()
    response = client.embeddings.create(
        model="text-embedding-ada-002",
        input=text
    )
    return response.data[0].embedding

def translate_to_urdu(text: str):
    """Translate text to Urdu"""
    client = get_openai_client()
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a technical translator. Translate to Urdu. PRESERVE these terms in English: ROS 2, Node, Topic, Service, Publisher, Subscriber, rclpy."},
            {"role": "user", "content": f"Translate: {text}"}
        ],
        temperature=0.3
    )
    return response.choices[0].message.content

def chat_completion(prompt: str, context: str):
    """Generate chatbot response"""
    client = get_openai_client()
    response = client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": f"Answer based on this context:\n{context}"},
            {"role": "user", "content": prompt}
        ],
        temperature=0.7
    )
    return response.choices[0].message.content
```

#### File 5: `backend/src/main.py` (T013)

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(
    title="Physical AI Textbook API",
    description="Backend for Chapter 1: ROS 2 Fundamentals",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "https://*.github.io", "https://*.vercel.app"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "Physical AI Textbook API - Chapter 1"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# TODO: Add routes for:
# - /api/auth/signup
# - /api/auth/login
# - /api/personalize
# - /api/translate
# - /api/chatbot/ask
```

---

### Step 4: Test Database Connection (T020)

```bash
# Make sure you're in backend/ directory with venv activated
cd backend
source venv/Scripts/activate  # Windows Git Bash

# Set environment variables (temporary)
export NEON_DATABASE_URL="your-neon-url-here"

# Run database setup
python -m src.database
```

**Expected output**: "✅ Database tables created successfully"

---

### Step 5: Test Frontend Server

```bash
# Navigate to project root
cd /c/Users/affil/Desktop/book

# Start Docusaurus development server
npm start
```

**Expected result**:
- Server starts at `http://localhost:3000`
- Browser opens automatically
- You see Docusaurus default page

**Note**: Chapter 1 content pages will show 404 until you create the markdown files (Phase 3).

---

### Step 6: Test Backend Server

```bash
# In a NEW terminal, navigate to backend
cd /c/Users/affil/Desktop/book/backend
source venv/Scripts/activate

# Run FastAPI server
uvicorn src.main:app --reload
```

**Expected result**:
- Server starts at `http://localhost:8000`
- Visit `http://localhost:8000/docs` to see API documentation
- Visit `http://localhost:8000/health` to see `{"status": "healthy"}`

---

## 📝 Phase 1 Completion Checklist

- [ ] Frontend dependencies installed (`npm install` successful)
- [ ] Backend virtual environment created
- [ ] Backend dependencies installed (`pip install -r requirements.txt` successful)
- [ ] All 5 backend Python files created (config.py, database.py, vector_store.py, ai_client.py, main.py)
- [ ] `.env` file created with actual API keys (copy from `.env.example`)
- [ ] Database tables created (run `python -m src.database`)
- [ ] Qdrant collection created (run `python -m src.vector_store`)
- [ ] Frontend server runs successfully (`npm start`)
- [ ] Backend server runs successfully (`uvicorn src.main:app --reload`)

---

## 🎯 Next Steps: Phase 3 (Content Creation)

Once Phase 1 is complete, you're ready for **Phase 3: Core Content** (T021-T046):
- Write Chapter 1 markdown files (intro, what-is-ros2, etc.)
- Build interactive React components (CodeEditor, RobotViewer, Quiz)
- Create Python code examples
- Add exercises and quizzes

**Estimated time to complete Phase 1 manual steps**: 30-45 minutes

---

## 🐛 Troubleshooting

### Issue: `npm install` fails
**Solution**: Ensure Node.js >= 18.0 is installed (`node --version`)

### Issue: Python package installation fails
**Solution**: Upgrade pip first: `pip install --upgrade pip`

### Issue: Database connection fails
**Solution**: Check your Neon database URL in `.env` file

### Issue: Qdrant connection fails
**Solution**: Verify Qdrant Cloud URL and API key in `.env`

---

## 📞 Need Help?

If you encounter issues:
1. Check the error message carefully
2. Verify all environment variables in `.env`
3. Make sure virtual environment is activated for Python commands
4. Ask me for help with specific error messages!

**Good luck with Phase 1 completion!** 🚀
