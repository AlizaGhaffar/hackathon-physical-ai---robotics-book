# Quickstart Guide: ChatKit Frontend Integration

**Feature**: 005-chatkit-frontend-integration
**For**: Developers implementing the chat interface
**Time**: ~2-3 hours for full implementation

---

## Prerequisites

✅ **Backend Running**: FastAPI backend must be running at `http://localhost:8000`
- Start backend: `cd backend && uvicorn src.main:app --reload`
- Verify health: `curl http://localhost:8000/api/health`

✅ **Dependencies Installed**: ChatScope already installed in `package.json`
- Verify: `npm list @chatscope/chat-ui-kit-react`
- If missing: `npm install @chatscope/chat-ui-kit-react @chatscope/chat-ui-kit-styles`

✅ **Additional Dependencies** (will be added):
- `npm install react-markdown react-syntax-highlighter`
- `npm install --save-dev @types/react-syntax-highlighter`

---

## Implementation Steps

### Step 1: Create TypeScript Types (5 min)

**File**: `src/types/chat.ts`

```typescript
// API Request/Response types (match backend Pydantic models)
export interface QueryRequest {
  query: string;
  chapter_id: string;
  session_id?: string;
  selected_text?: string;
}

export interface RetrievedChunk {
  chunk_id: string;
  content: string;
  score: number;
  source_file: string;
  chapter_id: string;
}

export interface QueryResponse {
  request_id: string;
  answer: string;
  session_id: string;
  sources: RetrievedChunk[];
  token_usage?: {
    prompt: number;
    completion: number;
    total: number;
  };
  response_time_ms?: number;
}

// Frontend-specific types
export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: RetrievedChunk[];
  isError?: boolean;
}
```

---

### Step 2: Create API Service Layer (15 min)

**File**: `src/services/chatService.ts`

```typescript
import type { QueryRequest, QueryResponse } from '../types/chat';

// Custom error class
export class ChatServiceError extends Error {
  constructor(
    public type: 'network' | 'timeout' | 'server' | 'validation',
    message: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'ChatServiceError';
  }
}

// Get API URL from Docusaurus config (or fallback to localhost)
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000';

/**
 * Send query to RAG backend
 */
export async function sendQuery(
  query: string,
  chapterId: string,
  sessionId?: string
): Promise<QueryResponse> {
  // Abort after 30 seconds
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 30000);

  try {
    const response = await fetch(`${API_BASE_URL}/api/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: query.trim(),
        chapter_id: chapterId,
        session_id: sessionId,
      }),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    // Handle HTTP errors
    if (!response.ok) {
      if (response.status === 422) {
        throw new ChatServiceError(
          'validation',
          'Invalid question format. Please try rephrasing.',
          422
        );
      }
      if (response.status >= 500) {
        throw new ChatServiceError(
          'server',
          'The chatbot service is temporarily unavailable. Please try again later.',
          response.status
        );
      }
    }

    return await response.json();
  } catch (error: any) {
    clearTimeout(timeoutId);

    // Handle timeout
    if (error.name === 'AbortError') {
      throw new ChatServiceError('timeout', 'Request took too long. Please try again.');
    }

    // Re-throw ChatServiceError
    if (error instanceof ChatServiceError) {
      throw error;
    }

    // Network error
    throw new ChatServiceError(
      'network',
      'Unable to connect to the chatbot service. Please check your connection.'
    );
  }
}

/**
 * Get or create session ID from sessionStorage
 */
export function getOrCreateSessionId(): string {
  const STORAGE_KEY = 'chat_session_id';
  let sessionId = sessionStorage.getItem(STORAGE_KEY);

  if (!sessionId) {
    sessionId = crypto.randomUUID();
    sessionStorage.setItem(STORAGE_KEY, sessionId);
  }

  return sessionId;
}

/**
 * Extract chapter ID from current page URL
 */
export function extractChapterId(pathname: string): string {
  const match = pathname.match(/\/chapter-(\d+)/);
  return match ? `chapter-${match[1]}` : 'chapter-1'; // Default fallback
}
```

---

### Step 3: Create Chat Component (45 min)

**File**: `src/components/ChatBot.tsx`

```typescript
import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message as ChatMessage,
  MessageInput,
  TypingIndicator,
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import ReactMarkdown from 'react-markdown';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { oneDark } from 'react-syntax-highlighter/dist/esm/styles/prism';

import { sendQuery, getOrCreateSessionId, extractChapterId, ChatServiceError } from '../services/chatService';
import type { Message } from '../types/chat';
import styles from './ChatBot.module.css';

export default function ChatBot() {
  const location = useLocation();
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => getOrCreateSessionId());
  const [chapterId] = useState(() => extractChapterId(location.pathname));
  const messageListRef = useRef<any>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messageListRef.current) {
      messageListRef.current.scrollToBottom();
    }
  }, [messages]);

  const handleSend = async (messageText: string) => {
    const trimmedMessage = messageText.trim();
    if (!trimmedMessage || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: crypto.randomUUID(),
      role: 'user',
      content: trimmedMessage,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);

    // Call backend
    setIsLoading(true);
    try {
      const response = await sendQuery(trimmedMessage, chapterId, sessionId);

      // Add assistant message
      const assistantMessage: Message = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: response.answer,
        timestamp: new Date(),
        sources: response.sources,
      };
      setMessages((prev) => [...prev, assistantMessage]);

      // Log performance metrics
      if (response.response_time_ms) {
        console.log(`[ChatBot] Response time: ${(response.response_time_ms / 1000).toFixed(2)}s`);
      }
    } catch (error) {
      console.error('[ChatBot] Error:', error);

      // Add error message
      const errorMessage: Message = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content:
          error instanceof ChatServiceError
            ? error.message
            : 'An unexpected error occurred. Please try again.',
        timestamp: new Date(),
        isError: true,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatContainer}>
      <MainContainer>
        <ChatContainer>
          <MessageList
            ref={messageListRef}
            typingIndicator={isLoading && <TypingIndicator content="AI is thinking..." />}
          >
            {messages.map((msg) => (
              <ChatMessage
                key={msg.id}
                model={{
                  message: (
                    <div>
                      <ReactMarkdown
                        components={{
                          code({ node, inline, className, children, ...props }) {
                            const match = /language-(\w+)/.exec(className || '');
                            return !inline && match ? (
                              <SyntaxHighlighter language={match[1]} style={oneDark} PreTag="div">
                                {String(children).replace(/\n$/, '')}
                              </SyntaxHighlighter>
                            ) : (
                              <code className={className} {...props}>
                                {children}
                              </code>
                            );
                          },
                        }}
                      >
                        {msg.content}
                      </ReactMarkdown>
                      {msg.sources && msg.sources.length > 0 && (
                        <details className={styles.sources}>
                          <summary>📚 Sources ({msg.sources.length})</summary>
                          <ul>
                            {msg.sources.map((source) => (
                              <li key={source.chunk_id}>
                                <strong>{source.source_file}</strong> (relevance: {(source.score * 100).toFixed(0)}%)
                                <p className={styles.sourceContent}>{source.content.substring(0, 200)}...</p>
                              </li>
                            ))}
                          </ul>
                        </details>
                      )}
                    </div>
                  ),
                  sender: msg.role === 'user' ? 'You' : 'AI Assistant',
                  direction: msg.role === 'user' ? 'outgoing' : 'incoming',
                  position: 'single',
                }}
                className={msg.isError ? styles.errorMessage : ''}
              />
            ))}
          </MessageList>
          <MessageInput
            placeholder={isLoading ? 'Please wait...' : 'Ask a question about this chapter...'}
            onSend={handleSend}
            disabled={isLoading}
            attachButton={false}
          />
        </ChatContainer>
      </MainContainer>
    </div>
  );
}
```

---

### Step 4: Create Component Styles (10 min)

**File**: `src/components/ChatBot.module.css`

```css
.chatContainer {
  height: 500px;
  width: 100%;
  border: 1px solid var(--ifm-color-emphasis-300);
  border-radius: 8px;
  overflow: hidden;
  margin: 2rem 0;
}

.errorMessage {
  background-color: var(--ifm-color-danger-lightest) !important;
  border-left: 3px solid var(--ifm-color-danger);
}

.sources {
  margin-top: 1rem;
  padding: 0.5rem;
  background-color: var(--ifm-color-emphasis-100);
  border-radius: 4px;
  font-size: 0.9rem;
}

.sources summary {
  cursor: pointer;
  font-weight: 600;
  margin-bottom: 0.5rem;
}

.sources ul {
  list-style: none;
  padding: 0;
  margin: 0;
}

.sources li {
  margin-bottom: 1rem;
  padding: 0.5rem;
  background-color: white;
  border-radius: 4px;
}

.sourceContent {
  margin-top: 0.5rem;
  color: var(--ifm-color-emphasis-700);
  font-size: 0.85rem;
  font-style: italic;
}

/* Dark mode support */
[data-theme='dark'] .sources {
  background-color: var(--ifm-color-emphasis-200);
}

[data-theme='dark'] .sources li {
  background-color: var(--ifm-color-emphasis-100);
}
```

---

### Step 5: Configure API URL in Docusaurus (5 min)

**File**: `docusaurus.config.ts`

Add custom field for API URL:

```typescript
const config: Config = {
  // ... existing config
  customFields: {
    apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
  },
};
```

**Optional**: Create `.env` file in project root:
```env
REACT_APP_API_URL=http://localhost:8000
```

---

### Step 6: Embed Chat in MDX Page (2 min)

**File**: `docs/chapter-1/intro.mdx` (or any chapter page)

```mdx
---
sidebar_position: 1
---

import ChatBot from '@site/src/components/ChatBot';

# Chapter 1: Introduction to ROS 2

<content here>

## Ask Questions

Have questions about this chapter? Use the AI chatbot below:

<ChatBot />
```

---

## Testing the Implementation

### 1. Start Backend

```bash
cd backend
source venv/bin/activate  # Or venv\Scripts\activate on Windows
uvicorn src.main:app --reload
```

Verify backend is running: `curl http://localhost:8000/api/health`

### 2. Start Frontend

```bash
npm start
```

Navigate to: `http://localhost:3000/chapter-1/intro`

### 3. Test Chat Interface

**Basic Tests**:
1. ✅ Type "What is ROS 2?" and send
2. ✅ Verify loading indicator appears
3. ✅ Verify answer displays with sources
4. ✅ Send multiple messages, verify conversation history builds

**Error Tests**:
1. ✅ Stop backend, verify network error message
2. ✅ Send empty message, verify no request sent
3. ✅ Send very long message (>1000 chars), verify validation error

**Chapter Switching**:
1. ✅ Send message on chapter-1 page
2. ✅ Navigate to different chapter page
3. ✅ Verify new ChatBot instance with correct chapter_id

---

## Troubleshooting

### Issue: CORS Error

**Symptom**: Browser console shows "CORS policy blocked"

**Solution**:
1. Check Docusaurus dev server port: `npm start` output
2. Update backend `.env` file:
   ```env
   CORS_ORIGINS=http://localhost:3000,http://localhost:8000
   ```
3. Restart backend server

---

### Issue: ChatScope Styles Not Loading

**Symptom**: Chat interface looks broken, no styling

**Solution**:
```bash
npm install @chatscope/chat-ui-kit-styles
```

Verify import in `ChatBot.tsx`:
```typescript
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
```

---

### Issue: Markdown Not Rendering

**Symptom**: AI answers show raw markdown (**, ##, etc.)

**Solution**:
```bash
npm install react-markdown react-syntax-highlighter
npm install --save-dev @types/react-syntax-highlighter
```

---

### Issue: Session ID Not Persisting

**Symptom**: New session ID on every message

**Solution**:
- Check browser sessionStorage: DevTools → Application → Session Storage
- Verify `getOrCreateSessionId()` function called only once
- Use `useState(() => getOrCreateSessionId())` not `useState(getOrCreateSessionId())`

---

### Issue: Chapter ID Always "chapter-1"

**Symptom**: All queries scoped to chapter-1 regardless of page

**Solution**:
- Check URL pattern: Must contain `/chapter-X/` segment
- Test with: `console.log('pathname:', location.pathname)`
- Update regex in `extractChapterId()` if URL structure differs

---

## Performance Optimization

### Lazy Load Chat Component

If chat is below the fold, lazy load to improve initial page load:

```typescript
import React, { lazy, Suspense } from 'react';

const ChatBot = lazy(() => import('@site/src/components/ChatBot'));

// In MDX:
<Suspense fallback={<div>Loading chat...</div>}>
  <ChatBot />
</Suspense>
```

### Debounce User Input (Optional)

Prevent accidental rapid submissions:

```typescript
// Add to ChatBot.tsx
import { useState, useRef } from 'react';

const lastSendTime = useRef(0);
const SEND_COOLDOWN_MS = 1000; // 1 second

const handleSend = async (messageText: string) => {
  const now = Date.now();
  if (now - lastSendTime.current < SEND_COOLDOWN_MS) {
    console.log('[ChatBot] Debouncing send');
    return;
  }
  lastSendTime.current = now;

  // ... rest of handleSend logic
};
```

---

## Next Steps

After completing implementation:

1. **Run `/sp.tasks`**: Generate detailed task breakdown
2. **Implement tests**: Add Jest/React Testing Library tests
3. **Deploy backend**: Update `REACT_APP_API_URL` for production
4. **Add analytics**: Track chat usage (optional)
5. **Enhance UX**: Add welcome message, suggested questions, etc.

---

## Estimated Timeline

| Step | Time | Cumulative |
|------|------|------------|
| 1. Create types | 5 min | 5 min |
| 2. Create service layer | 15 min | 20 min |
| 3. Create ChatBot component | 45 min | 1h 5min |
| 4. Create styles | 10 min | 1h 15min |
| 5. Configure Docusaurus | 5 min | 1h 20min |
| 6. Embed in MDX | 2 min | 1h 22min |
| 7. Testing | 30 min | 1h 52min |
| 8. Debugging/polish | 30 min | **2h 22min** |

**Total**: ~2-3 hours for experienced React developer

---

## Success Criteria

✅ Chat interface displays in chapter pages
✅ Users can send questions and receive AI answers
✅ Loading indicator shows during backend processing
✅ Error messages display clearly for failures
✅ Conversation history persists during session
✅ Session ID persists across page navigation (same tab)
✅ Chapter context automatically detected from URL
✅ Markdown in answers renders correctly
✅ Source citations display below answers
✅ No console errors in browser DevTools
