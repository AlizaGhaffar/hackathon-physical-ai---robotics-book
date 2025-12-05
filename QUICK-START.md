# 🚀 Quick Start Guide

## Run Frontend (Already Running ✅)
```bash
npm start
```
Opens at: **http://localhost:3000**

---

## Run Backend (For Chatbot to Work)

### 1. Setup (First Time Only)
```bash
cd backend

# Create virtual environment
python -m venv venv

# Activate
venv\Scripts\activate  # Windows
source venv/bin/activate  # Linux/Mac

# Install dependencies
pip install -r requirements.txt
```

### 2. Configure Environment
Create `backend/.env`:
```env
OPENAI_API_KEY=sk-your-key-here
NEON_DATABASE_URL=postgresql://user:pass@host/db?sslmode=require
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-key
BETTER_AUTH_SECRET=your-secret-key-min-32-chars
```

### 3. Initialize Database (First Time Only)
```bash
python setup_database.py
```

### 4. Ingest Content (First Time Only)
```bash
python -m src.scripts.ingest_content
```

### 5. Run Backend Server
```bash
uvicorn src.main:app --reload --port 8000
```

Backend at: **http://localhost:8000**

---

## Test Everything

### Frontend
- ✅ Visit http://localhost:3000
- ✅ See landing page (no auth)
- ✅ Click "What is ROS 2?" in sidebar
- ✅ See auth prompt with gradient UI
- ✅ Sign up/Login
- ✅ Browse all chapters

### Backend
- ✅ Visit http://localhost:8000/health
- ✅ Should see: `{"status": "healthy"}`

### Chatbot
- ✅ Click chatbot icon (bottom-right)
- ✅ Ask: "What is ROS 2?"
- ✅ Get AI response with sources

---

## 🎬 Demo Flow (for video)

1. **Start** - Show landing page
2. **Navigate** - Click any chapter
3. **Auth** - Show beautiful auth prompt
4. **Sign Up** - Fill form with background
5. **Explore** - Browse chapters freely
6. **Chatbot** - Ask questions, show RAG responses
7. **Done** - < 90 seconds!

---

## 🐛 Troubleshooting

### Port 3000 Already in Use
```bash
# Kill process on port 3000
npx kill-port 3000

# Or use different port
npm start -- --port 3001
```

### Backend Connection Error
- Check backend is running: `http://localhost:8000/health`
- Check .env file has correct API keys
- Check CORS settings in backend/src/main.py

### Chatbot Not Working
- Ensure backend is running
- Check browser console for errors
- Check backend logs for API errors
- Verify OpenAI API key is valid

---

**Need Help?** Check:
- FINALIZED-PROJECT.md (full details)
- backend/README.md (backend docs)
- IMPLEMENTATION-SUMMARY.md (architecture)
