# Physical AI Book - Complete Setup Guide

## 🎉 Features Implemented

### 1. **Authentication Required for Book Content**
- All book chapters now require user authentication
- Landing page is public and accessible to everyone
- Login/Signup modal with user profile creation
- Smooth authentication flow with automatic page reload

### 2. **Full Book Translation to Urdu**
- Dynamic translation button in header (🌐)
- Click to translate entire page content to Urdu
- Click again to restore English
- Powered by OpenAI GPT-4o-mini for accurate translations
- Translation state persists across page refreshes

### 3. **Persistent Header Options**
- All header buttons (Sign In, Translate, Personalize) visible on:
  - Landing page
  - All chapter pages
  - Every route in the application
- Smooth UI/UX with no network errors

### 4. **Personalization Feature**
- User profiles with software/hardware skill levels
- Learning goals tracking
- Personalized chatbot responses based on skill level

## 🚀 Quick Start

### Backend Server
The backend is already running at `http://localhost:8000`

To check backend status:
```bash
# Test health endpoint
curl http://localhost:8000/health

# Should return: {"status":"healthy"}
```

### Frontend Application
The frontend is running at `http://localhost:3000/book/`

## 📖 User Flow

### 1. Landing Page
- Open `http://localhost:3000/book/`
- You'll see the home page with 3 chapter cards
- Header shows: **Sign In**, **Translate (اردو)**, and **Personalize for Me**

### 2. Sign Up / Sign In
**To create an account:**
1. Click **🔐 Login / Sign Up** button in the header
2. Click **Sign Up** in the modal
3. Fill in:
   - Name
   - Email
   - Password (minimum 8 characters)
   - Software Level (Beginner/Intermediate/Advanced)
   - Hardware Level (Beginner/Intermediate/Advanced)
   - Learning Goals (optional)
4. Click **Sign Up**
5. Page will reload and you'll be logged in

**To login:**
1. Click **🔐 Login / Sign Up** button
2. Enter your email and password
3. Click **Login**
4. Page will reload

### 3. Reading Book Content
**After logging in:**
- Click on any chapter card (Chapter 1, 2, or 3)
- You can now access all book content
- Navigate through chapters using the sidebar

**Without login:**
- If you try to access chapter pages directly
- You'll see a beautiful lock screen
- Click **Sign In / Sign Up** to authenticate

### 4. Translation Feature
**To translate to Urdu:**
1. Click the **🌐 اردو** button in the header
2. The page content will translate to Urdu
3. Button changes to **🌐 English**
4. Wait a moment for all content to translate

**To restore English:**
1. Click the **🌐 English** button
2. Original English content is restored instantly

**Important Notes:**
- Translation works on any page
- Code blocks are NOT translated (they remain in original format)
- Translation state persists when you refresh
- Each page needs to be translated separately

### 5. Personalization
1. Click **👤 Personalize for Me** button
2. This feature adapts chatbot responses to your skill level
3. You must be logged in to use this

## 🛠️ Technical Details

### Backend Endpoints

**Authentication:**
- `POST /api/auth/signup` - Create new user account
- `POST /api/auth/login` - Login existing user

**Translation:**
- `POST /api/translate` - Translate text to Urdu
  ```json
  {
    "text": "Hello World",
    "target_language": "ur"
  }
  ```

**Chatbot:**
- `POST /api/chatbot/ask` - Ask questions (public)
- `POST /api/chatbot/personalized` - Personalized responses (requires auth)

### Frontend Components

**Authentication:**
- `AuthButton.tsx` - Login/Signup modal and user dropdown
- `AuthGuard.tsx` - Protected route wrapper
- Improved error handling (no more generic network errors)

**Translation:**
- `TranslateButton.tsx` - Dynamic page translation
- Uses OpenAI API for accurate translations
- Stores/restores original content

**Layout:**
- `Root.tsx` - Main wrapper with auth guard and header buttons
- Header buttons visible on all pages

### Database Schema

**Users Table:**
```sql
- id: UUID (primary key)
- email: VARCHAR (unique)
- password_hash: VARCHAR
- name: VARCHAR
- software_level: VARCHAR (Beginner/Intermediate/Advanced)
- hardware_level: VARCHAR (Beginner/Intermediate/Advanced)
- learning_goals: TEXT
- created_at: TIMESTAMP
```

## 🔧 Troubleshooting

### Backend Not Running
```bash
cd backend
python -m uvicorn src.main:app --reload --port 8000
```

### Database Issues
```bash
cd backend
python setup_database.py
```

### Frontend Not Compiling
```bash
npm start
```

### Clear Cache
```bash
npm run clear
npm start
```

### Translation Not Working
1. Check backend is running: `http://localhost:8000/health`
2. Verify OpenAI API key in `backend/.env`
3. Check browser console for errors

### Authentication Errors
1. Verify backend database connection
2. Check Neon database URL in `backend/.env`
3. Run `python backend/setup_database.py` to create tables

## 📝 Environment Variables

### Backend (.env in backend folder)
```
OPENAI_API_KEY=your-key-here
NEON_DATABASE_URL=postgresql://...
QDRANT_URL=https://...
QDRANT_API_KEY=...
BETTER_AUTH_SECRET=random-secret-key
```

### Frontend
No .env needed - uses `http://localhost:8000` by default

## ✅ What's Working

- ✅ Authentication required before reading books
- ✅ Login/Signup with user profiles
- ✅ Full page translation to Urdu
- ✅ Header buttons visible everywhere (landing page + all routes)
- ✅ No network errors - smooth error handling
- ✅ User session persistence
- ✅ Protected routes with AuthGuard
- ✅ Backend API with CORS configured
- ✅ Translation API endpoint

## 🎯 Testing Checklist

1. [ ] Open `http://localhost:3000/book/`
2. [ ] Verify header shows all three buttons
3. [ ] Click on Chapter 1 - should show auth prompt
4. [ ] Click Sign Up and create account
5. [ ] After signup, should access chapter content
6. [ ] Click Physical AI logo - returns to landing page (buttons still visible)
7. [ ] Click Translate button - content translates to Urdu
8. [ ] Click English button - content restores
9. [ ] Logout and login again - should work smoothly
10. [ ] No "Network error" messages during normal operation

## 📞 Support

If you encounter any issues:
1. Check both servers are running (frontend:3000, backend:8000)
2. Check browser console for errors (F12)
3. Verify database connection
4. Check OpenAI API key is valid

---

**Congratulations!** Your book app is now fully functional with:
- 🔐 Authentication
- 🌐 Translation
- 👤 Personalization
- 📚 Protected content
- ✨ Smooth user experience
