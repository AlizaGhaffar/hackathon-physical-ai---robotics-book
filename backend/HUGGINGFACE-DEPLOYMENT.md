# Hugging Face Spaces Deployment Guide

Backend ko Hugging Face Spaces par deploy karne ke liye complete step-by-step guide.

## Prerequisites

1. **Hugging Face Account**: https://huggingface.co/ par account banayein
2. **Git Install**: System par Git installed hona chahiye
3. **Environment Variables**: Aapke paas ye API keys hone chahiye:
   - `OPENAI_API_KEY`
   - `COHERE_API_KEY`
   - `QDRANT_URL`
   - `QDRANT_API_KEY`
   - `DATABASE_URL` (Neon Postgres)

## Step 1: Hugging Face Space Create Karein

1. Hugging Face par login karein: https://huggingface.co/
2. Apne profile icon par click karein → **New Space** select karein
3. Space details fill karein:
   - **Space name**: `rag-chatbot-backend` (ya koi bhi naam)
   - **License**: Apache 2.0 (recommended)
   - **Space SDK**: **Docker** select karein (important!)
   - **Visibility**: Public ya Private (aapki choice)
4. **Create Space** button par click karein

## Step 2: Space Ko Local Machine Par Clone Karein

1. Terminal/Command Prompt open karein
2. Jahan aap files rakhna chahte hain wahan navigate karein (e.g., Desktop)
3. Clone command chalayein:

```bash
# Replace YOUR_USERNAME aur YOUR_SPACE_NAME apne actual names se
git clone https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME
```

**Example**:
```bash
git clone https://huggingface.co/spaces/myusername/rag-chatbot-backend
```

4. Cloned folder mein enter karein:
```bash
cd YOUR_SPACE_NAME
```

## Step 3: Backend Files Copy Karein

Ab backend ki files ko cloned Space folder mein copy karein:

### Option A: Manual Copy (Recommended for beginners)

1. `backend` folder kholen apne project mein
2. In files/folders ko COPY karein:
   - `src/` folder (complete folder with all files)
   - `migrations/` folder
   - `alembic.ini` file
   - `requirements.txt` file
   - `Dockerfile.huggingface` file

3. In sab ko cloned Space folder mein PASTE karein

4. **Important**: `Dockerfile.huggingface` ko rename karein to `Dockerfile`:
   - `Dockerfile.huggingface` ko delete kar ke
   - Rename: `Dockerfile.huggingface` → `Dockerfile`

### Option B: Command Line Copy

```bash
# Windows Command Prompt se (Desktop se run karein)
# Replace paths accordingly

# Src folder copy
xcopy "book\backend\src" "YOUR_SPACE_NAME\src" /E /I /H

# Migrations folder copy
xcopy "book\backend\migrations" "YOUR_SPACE_NAME\migrations" /E /I /H

# Individual files copy
copy "book\backend\alembic.ini" "YOUR_SPACE_NAME\"
copy "book\backend\requirements.txt" "YOUR_SPACE_NAME\"
copy "book\backend\Dockerfile.huggingface" "YOUR_SPACE_NAME\Dockerfile"
```

## Step 4: Environment Variables/Secrets Set Karein

Hugging Face Space mein environment variables add karne ke liye:

1. Apne Space page par jayen browser mein: `https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME`
2. **Settings** tab par click karein
3. **Repository secrets** section mein jayen
4. Har secret add karein:

Click **Add a secret** button aur fill karein:

- **Name**: `OPENAI_API_KEY` → **Value**: [Your OpenAI API Key]
- **Name**: `COHERE_API_KEY` → **Value**: [Your Cohere API Key]
- **Name**: `QDRANT_URL` → **Value**: [Your Qdrant URL]
- **Name**: `QDRANT_API_KEY` → **Value**: [Your Qdrant API Key]
- **Name**: `DATABASE_URL` → **Value**: [Your Neon Postgres URL]
- **Name**: `CORS_ORIGINS` → **Value**: `https://physical-ai-robotics-book.vercel.app,http://localhost:3000`
- **Name**: `API_HOST` → **Value**: `0.0.0.0`
- **Name**: `API_PORT` → **Value**: `7860`

## Step 5: Git Push Karein

Ab cloned folder se Hugging Face par push karein:

```bash
# Space folder mein hone ko confirm karein
cd YOUR_SPACE_NAME

# Check karo kya files add hui hain
git status

# Sari files add karein
git add .

# Commit karein
git commit -m "Deploy RAG chatbot backend to Hugging Face Spaces"

# Push karein
git push
```

**Note**: Pehli baar push karte waqt Hugging Face aapko login credentials maangega:
- **Username**: Aapka Hugging Face username
- **Password**: Aapka Hugging Face **Access Token** (NOT password)
  - Token banane ke liye: Settings → Access Tokens → New token

## Step 6: Deployment Monitor Karein

1. Browser mein apne Space page kholen: `https://huggingface.co/spaces/YOUR_USERNAME/YOUR_SPACE_NAME`
2. **Logs** tab par click karein
3. Build process dekhein:
   - Docker image build ho rahi hai
   - Dependencies install ho rahi hain
   - Application start ho raha hai

4. Jab deployment complete ho:
   - Status "Running" dikhayega
   - Space URL active ho jayega: `https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space`

## Step 7: Test Karein

Deployment successful hai ya nahi test karne ke liye:

### Health Check

Browser ya Postman mein:
```
https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/api/health
```

Expected response:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-17T...",
  "services": {
    "openai": "connected",
    "qdrant": "connected",
    "database": "connected"
  }
}
```

### Query Test

POST request bhejein:
```
POST https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space/api/query
```

Body:
```json
{
  "query": "What is physical AI?",
  "chapter": null,
  "selectedText": null
}
```

## Troubleshooting

### Build Failed

**Logs mein dekhein** kya error hai:
- **Import errors**: Check karein `requirements.txt` mein sare dependencies hain
- **Port error**: Dockerfile mein port 7860 confirm karein
- **Environment variable error**: Secrets sahi se set hain confirm karein

### Application Crash

**Common issues**:
1. **Database connection**: `DATABASE_URL` sahi hai?
2. **API keys invalid**: Secrets double-check karein
3. **Qdrant connection**: Free tier limit cross to nahi hui?

### Logs Kaise Dekhein

Space page → **Logs** tab:
- Build logs
- Application logs
- Error messages

## Files Structure (Final)

Aapke cloned Space folder mein ye structure hona chahiye:

```
YOUR_SPACE_NAME/
├── src/
│   ├── __init__.py
│   ├── main.py
│   ├── config.py
│   ├── api/
│   ├── middleware/
│   ├── models/
│   ├── services/
│   └── utils/
├── migrations/
├── alembic.ini
├── requirements.txt
├── Dockerfile
└── README.md (optional)
```

## Update Karne Ke Liye

Agar future mein backend update karna hai:

1. Local cloned folder mein files edit karein
2. Git add, commit, push karein:

```bash
cd YOUR_SPACE_NAME
git add .
git commit -m "Update backend with new features"
git push
```

3. Hugging Face automatically rebuild karega

## Frontend Se Connect Karein

Frontend (Vercel) mein environment variable set karein:

```env
NEXT_PUBLIC_API_URL=https://YOUR_USERNAME-YOUR_SPACE_NAME.hf.space
```

Vercel dashboard → Your Project → Settings → Environment Variables

---

## Quick Reference Commands

```bash
# Clone Space
git clone https://huggingface.co/spaces/USERNAME/SPACE_NAME
cd SPACE_NAME

# Copy files (Windows)
xcopy "..\book\backend\src" "src" /E /I /H
xcopy "..\book\backend\migrations" "migrations" /E /I /H
copy "..\book\backend\requirements.txt" .
copy "..\book\backend\alembic.ini" .
copy "..\book\backend\Dockerfile.huggingface" "Dockerfile"

# Push to Hugging Face
git add .
git commit -m "Deploy backend"
git push
```

---

**Important Notes**:

1. Hugging Face Spaces **free tier** hai, but build time slow ho sakta hai
2. Port **7860** compulsory hai Hugging Face ke liye
3. Secrets ko **never commit** mat karna Git mein
4. Backend URL frontend mein update karna na bhulein

Agar koi step clear nahi hai to mujhe batayen!
