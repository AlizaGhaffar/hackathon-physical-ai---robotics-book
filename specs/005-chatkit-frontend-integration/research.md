# Research: ChatKit Frontend Integration

**Feature**: 005-chatkit-frontend-integration
**Date**: 2025-12-13
**Phase**: 0 (Research & Planning)

## Overview

This document consolidates research findings for integrating the ChatScope Chat UI Kit with the FastAPI RAG backend. All technical unknowns from the Technical Context have been resolved.

---

## 1. ChatScope Chat UI Kit Integration

### Decision
Use `@chatscope/chat-ui-kit-react` (v2.1.1) for the chat interface, which is already installed in package.json.

### Rationale
- **Already installed**: No new dependency needed
- **React-native**: Designed specifically for React applications
- **Comprehensive components**: Provides MainContainer, ChatContainer, MessageList, Message, MessageInput out of the box
- **Customizable**: Supports custom message renderers, styling via CSS modules
- **TypeScript support**: Includes type definitions
- **Lightweight**: ~50KB gzipped (well under 100KB bundle size constraint)

### Alternatives Considered
1. **Build custom chat UI**: Rejected - violates "no complexity without justification" principle, ChatScope provides exactly what we need
2. **Use shadcn/ui chat components**: Rejected - requires additional Tailwind CSS setup, ChatScope is simpler
3. **OpenAI ChatKit SDK**: Investigated but ChatScope is more appropriate for this use case (we're building a custom RAG integration, not using OpenAI's hosted chat)

### Implementation Pattern
```typescript
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message,
  MessageInput,
  TypingIndicator
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
```

**Key Features to Use**:
- `MainContainer`: Responsive wrapper with fixed height
- `MessageList`: Auto-scroll, virtualization for performance
- `Message`: User vs assistant role distinction
- `MessageInput`: Built-in send button, enter key support
- `TypingIndicator`: Display during backend processing

---

## 2. Chapter Detection Strategy

### Decision
Extract chapter ID from the current page URL using React Router/Docusaurus routing context.

### Rationale
- Docusaurus uses file-system routing: `/chapter-1/intro`, `/chapter-2/gazebo`, etc.
- URL structure already contains chapter identifier
- No need for global state management (Redux, Context) - just read from URL
- Works for all chapters (current and future) without modification

### Implementation Pattern
```typescript
// In Docusaurus, use useLocation from @docusaurus/router
import { useLocation } from '@docusaurus/router';

function ChatBot() {
  const location = useLocation();

  // Extract chapter ID from pathname: /chapter-1/intro → "chapter-1"
  const getChapterId = (pathname: string): string => {
    const match = pathname.match(/\/chapter-(\d+)/);
    return match ? `chapter-${match[1]}` : 'chapter-1'; // Default fallback
  };

  const chapterId = getChapterId(location.pathname);
}
```

**Validation**: Backend accepts both `"chapter-1"` and `"Chapter 1"` formats (verified in request_models.py:33-39).

### Alternatives Considered
1. **MDX frontmatter metadata**: Rejected - requires parsing MDX, overly complex
2. **Props from parent component**: Rejected - requires passing chapter_id through every page, not DRY
3. **Global state (React Context)**: Rejected - overkill for single value, URL is source of truth

---

## 3. API Client Error Handling

### Decision
Implement a service layer (`chatService.ts`) with structured error handling: network errors, timeout errors, HTTP errors (4xx, 5xx), and validation errors.

### Rationale
- **Separation of concerns**: Keep API logic out of UI components
- **Reusability**: Service can be used by multiple components if needed
- **Testability**: Easier to mock API calls in tests
- **Type safety**: Centralize TypeScript interfaces for request/response

### Implementation Pattern
```typescript
// src/services/chatService.ts
export class ChatServiceError extends Error {
  constructor(
    public type: 'network' | 'timeout' | 'server' | 'validation',
    public message: string,
    public statusCode?: number
  ) {
    super(message);
  }
}

export async function sendQuery(
  query: string,
  chapterId: string,
  sessionId?: string
): Promise<QueryResponse> {
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 30000); // 30s timeout

  try {
    const response = await fetch(`${API_URL}/api/query`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ query, chapter_id: chapterId, session_id: sessionId }),
      signal: controller.signal
    });

    clearTimeout(timeoutId);

    if (!response.ok) {
      if (response.status >= 500) {
        throw new ChatServiceError('server', 'Backend service error', response.status);
      } else if (response.status === 422) {
        throw new ChatServiceError('validation', 'Invalid query format', 422);
      }
    }

    return await response.json();
  } catch (error) {
    clearTimeout(timeoutId);
    if (error.name === 'AbortError') {
      throw new ChatServiceError('timeout', 'Request timeout after 30 seconds');
    }
    throw new ChatServiceError('network', 'Failed to connect to backend');
  }
}
```

**Error Messages for Users**:
- Network error: "Unable to connect to the chatbot service. Please check your connection."
- Timeout: "Request took too long. Please try again."
- Server error: "The chatbot service is temporarily unavailable. Please try again later."
- Validation error: "Invalid question format. Please try rephrasing."

### Alternatives Considered
1. **Axios library**: Rejected - native fetch is sufficient, avoids extra dependency
2. **React Query**: Rejected - overkill for this simple use case, adds complexity
3. **No error handling**: Rejected - violates spec requirement FR-008

---

## 4. Session Management

### Decision
Use browser `sessionStorage` to persist session_id within a single browser session. Generate UUID on first load, reuse for all subsequent queries in that session.

### Rationale
- **Spec compliance**: Spec states "Session management uses client-side session identifiers"
- **Simplicity**: No server-side session storage required for MVP
- **Privacy**: Session data cleared when browser tab closes
- **Backend compatibility**: Backend already generates session_id if not provided (query.py:40)

### Implementation Pattern
```typescript
// Generate or retrieve session ID
function getOrCreateSessionId(): string {
  let sessionId = sessionStorage.getItem('chat_session_id');
  if (!sessionId) {
    sessionId = crypto.randomUUID();
    sessionStorage.setItem('chat_session_id', sessionId);
  }
  return sessionId;
}
```

**Session Lifecycle**:
- Created: First query in a browser tab
- Persisted: Survives page navigation within same tab
- Cleared: When tab is closed
- Not shared: Each browser tab has independent session

### Alternatives Considered
1. **localStorage**: Rejected - sessions should not persist across browser restarts
2. **Cookies**: Rejected - unnecessary complexity, sessionStorage is simpler
3. **No session tracking**: Rejected - backend requires session_id for history feature

---

## 5. Message Display and Formatting

### Decision
Display messages using ChatScope's `Message` component with role-based styling. Parse markdown in AI responses using `react-markdown` (add as dependency).

### Rationale
- **Spec requirement FR-012**: "System MUST format AI responses appropriately (preserve line breaks, handle special characters)"
- **Backend sends markdown**: GPT-4 responses may include code blocks, lists, bold text
- **User experience**: Properly formatted responses improve readability
- **Code highlighting**: Using `react-syntax-highlighter` for code blocks in answers

### Implementation Pattern
```typescript
// Add dependencies (run during implementation):
// npm install react-markdown react-syntax-highlighter

import ReactMarkdown from 'react-markdown';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';

<Message
  model={{
    message: <ReactMarkdown
      components={{
        code({node, inline, className, children, ...props}) {
          const match = /language-(\w+)/.exec(className || '');
          return !inline && match ? (
            <SyntaxHighlighter language={match[1]} PreTag="div">
              {String(children).replace(/\n$/, '')}
            </SyntaxHighlighter>
          ) : (
            <code className={className} {...props}>{children}</code>
          );
        }
      }}
    >{answerText}</ReactMarkdown>,
    sender: "AI Assistant",
    direction: "incoming"
  }}
/>
```

### Alternatives Considered
1. **Plain text rendering**: Rejected - loses formatting, poor UX
2. **dangerouslySetInnerHTML**: Rejected - security risk (XSS)
3. **Custom markdown parser**: Rejected - reinventing the wheel

---

## 6. Loading States and UX Patterns

### Decision
Use ChatScope's `TypingIndicator` component during API calls. Disable input field to prevent multiple simultaneous requests.

### Rationale
- **Spec requirement FR-003**: "System MUST show a loading indicator while waiting for backend response"
- **Performance constraint**: Prevent users from sending multiple queries while one is processing
- **Expected backend latency**: 3-15 seconds per query (embedding + retrieval + GPT generation)
- **Professional UX**: Typing indicator is standard chat interface pattern

### Implementation Pattern
```typescript
const [isLoading, setIsLoading] = useState(false);

const handleSend = async (message: string) => {
  setIsLoading(true);
  try {
    const response = await sendQuery(message, chapterId, sessionId);
    // Add messages to state...
  } finally {
    setIsLoading(false);
  }
};

// In JSX:
{isLoading && <TypingIndicator content="AI is thinking..." />}
<MessageInput
  disabled={isLoading}
  placeholder={isLoading ? "Please wait..." : "Ask a question about this chapter"}
/>
```

**Additional UX Enhancements**:
- Show response time in footer (use `response_time_ms` from backend)
- Auto-scroll to new messages
- Clear input field after successful send (FR-010)

### Alternatives Considered
1. **Spinner icon**: Rejected - typing indicator is more engaging
2. **Progress bar**: Rejected - backend doesn't provide granular progress
3. **No loading state**: Rejected - violates spec, poor UX for 15-second waits

---

## 7. CORS Configuration

### Decision
No frontend changes needed. Backend `.env` already has `CORS_ORIGINS=http://localhost:3000,http://localhost:8000`. Verify Docusaurus dev server runs on port 3000 (default).

### Rationale
- **Already configured**: backend/src/main.py:42-48 configures CORSMiddleware
- **Environment-based**: CORS origins read from .env, easy to update for production
- **Development ready**: localhost:3000 already whitelisted

### Verification Steps
1. Check Docusaurus dev server port: Run `npm start`, verify output shows `http://localhost:3000`
2. If different port: Update backend `.env` file `CORS_ORIGINS` to include actual port
3. No code changes needed in either frontend or backend

### Alternatives Considered
1. **Frontend proxy**: Rejected - unnecessary complexity, CORS is properly configured
2. **Backend code changes**: Rejected - configuration via .env is cleaner

---

## 8. Environment Configuration for API URL

### Decision
Use Docusaurus custom fields in `docusaurus.config.ts` to set API URL. Supports environment-specific URLs (localhost for dev, production URL for deployment).

### Rationale
- **Docusaurus pattern**: Recommended approach for environment-specific config
- **Type-safe**: Can be accessed via `useDocusaurusContext()`
- **Build-time configuration**: Different URLs for dev/prod without code changes

### Implementation Pattern
```typescript
// docusaurus.config.ts
const config: Config = {
  // ... existing config
  customFields: {
    apiUrl: process.env.REACT_APP_API_URL || 'http://localhost:8000',
  },
};

// In component:
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

function ChatBot() {
  const {siteConfig} = useDocusaurusContext();
  const apiUrl = siteConfig.customFields.apiUrl as string;
}
```

**Environment Variables**:
- Development: `REACT_APP_API_URL=http://localhost:8000`
- Production: `REACT_APP_API_URL=https://your-backend-url.com`

### Alternatives Considered
1. **Hardcode URL**: Rejected - not environment-agnostic
2. **.env file**: Rejected - Docusaurus doesn't support .env by default, customFields is idiomatic
3. **Runtime config fetch**: Rejected - unnecessary HTTP request on load

---

## 9. Component Integration Pattern

### Decision
Create standalone `ChatBot.tsx` component that can be imported into any MDX page. Use Docusaurus MDX component import to embed in chapter pages.

### Rationale
- **Spec constraint**: "Must not break existing Docusaurus book layout"
- **Flexibility**: Can be placed anywhere (sidebar, footer, inline in content)
- **Docusaurus pattern**: MDX files support direct component imports
- **Progressive enhancement**: Chat feature can be added to chapters incrementally

### Implementation Pattern
```mdx
---
# docs/chapter-1/intro.mdx
---

import ChatBot from '@site/src/components/ChatBot';

# Chapter 1: Introduction to ROS 2

<content here>

## Ask Questions

Have questions about this chapter? Use the chatbot below:

<ChatBot />
```

**Styling Strategy**:
- Component uses CSS Modules (`ChatBot.module.css`)
- Fixed height container (400px default, adjustable via prop)
- Responsive width (100% of parent container)
- Z-index management to avoid conflicts with Docusaurus navbar

### Alternatives Considered
1. **Global sidebar widget**: Rejected - requires Docusaurus theme swizzling, higher complexity
2. **Floating chat button**: Rejected - out of scope for MVP, can be added later
3. **Full-page chat route**: Rejected - spec requires inline integration

---

## 10. TypeScript Type Safety

### Decision
Define strict TypeScript interfaces matching backend Pydantic models. Use these types throughout the component and service layer.

### Rationale
- **Type safety**: Catch API contract mismatches at compile time
- **Developer experience**: Autocomplete for API responses
- **Documentation**: Types serve as inline API documentation
- **Refactor confidence**: Changes to types surface errors early

### Implementation Pattern
```typescript
// src/types/chat.ts

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

export interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
  sources?: RetrievedChunk[];
}
```

**Contract Validation**:
- Backend: `backend/src/models/request_models.py` (QueryRequest)
- Backend: `backend/src/models/response_models.py` (QueryResponse)
- Ensure frontend types stay in sync with backend Pydantic models

### Alternatives Considered
1. **Any types**: Rejected - loses type safety benefits
2. **Auto-generate from OpenAPI**: Rejected - overkill for 2 endpoints, manual sync is simple

---

## Summary of Resolved Unknowns

All technical clarifications from plan.md Technical Context have been resolved:

✅ **Chat UI library**: ChatScope Chat UI Kit (already installed)
✅ **Chapter detection**: URL parsing via useLocation
✅ **Error handling**: Service layer with typed errors
✅ **Session management**: sessionStorage with UUID
✅ **Message formatting**: react-markdown with syntax highlighting
✅ **Loading states**: TypingIndicator component
✅ **CORS**: Already configured in backend
✅ **Environment config**: Docusaurus customFields
✅ **Component integration**: MDX imports
✅ **Type safety**: Strict TypeScript interfaces

**Next Phase**: Proceed to Phase 1 (Design & Contracts) to define data models and API contracts.
