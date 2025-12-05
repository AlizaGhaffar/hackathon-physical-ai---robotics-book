# Component Interface Requirements for Chapter 2

**Status**: Components NOT YET IMPLEMENTED - This document specifies required interfaces

**Purpose**: Define component prop interfaces to ensure Chapter 2 compatibility when components are implemented

---

## Required Components for Chapter 2

All interactive components MUST accept `chapterId` prop to enable multi-chapter reuse per Constitution III (Component Reusability).

### 1. PersonalizeButton

**File**: `src/components/PersonalizeButton.tsx` (TO BE IMPLEMENTED)

**Interface**:
```typescript
interface PersonalizeButtonProps {
  chapterId: string;  // "chapter-1" | "chapter-2" | ...
  contentId: string;  // Section or page identifier for caching
  originalContent: string;  // Markdown content to personalize
  onPersonalized?: (personalizedContent: string) => void;  // Callback
}
```

**Usage in Chapter 2**:
```tsx
<PersonalizeButton
  chapterId="chapter-2"
  contentId="urdf-basics"
  originalContent={markdownContent}
  onPersonalized={(content) => setDisplayContent(content)}
/>
```

**API Endpoint**: `POST /api/personalize` (already chapter-agnostic)

---

### 2. TranslateButton

**File**: `src/components/TranslateButton.tsx` (TO BE IMPLEMENTED)

**Interface**:
```typescript
interface TranslateButtonProps {
  chapterId: string;  // "chapter-1" | "chapter-2" | ...
  contentId: string;  // Section or page identifier
  originalContent: string;  // Markdown content to translate
  targetLanguage: 'ur' | 'en';  // Urdu or English
  onTranslated?: (translatedContent: string) => void;  // Callback
}
```

**Usage in Chapter 2**:
```tsx
<TranslateButton
  chapterId="chapter-2"
  contentId="physics-engines"
  originalContent={markdownContent}
  targetLanguage="ur"
  onTranslated={(content) => setDisplayContent(content)}
/>
```

**API Endpoint**: `POST /api/translate` (already chapter-agnostic)

---

### 3. RAGChatbot

**File**: `src/components/RAGChatbot.tsx` (TO BE IMPLEMENTED)

**Interface**:
```typescript
interface RAGChatbotProps {
  chapterId: string;  // "chapter-1" | "chapter-2" | ...
  sessionId?: string;  // Optional session ID for history
  userId?: string;  // Optional user ID (null for anonymous)
}
```

**Usage in Chapter 2**:
```tsx
<RAGChatbot
  chapterId="chapter-2"
  sessionId={sessionId}
  userId={currentUser?.id}
/>
```

**API Endpoint**: `POST /api/chatbot/ask` (filters by chapter_id)

**Backend Behavior**:
- Searches Qdrant collection: `chapter_2_embeddings` (determined from `chapterId`)
- Filters results by `chapter_id="chapter-2"` metadata
- Returns answers using ONLY Chapter 2 content

---

### 4. ProgressTracker

**File**: `src/components/ProgressTracker.tsx` (TO BE IMPLEMENTED)

**Interface**:
```typescript
interface ProgressTrackerProps {
  userId: string;  // Current user ID
  chapters: Chapter[];  // From chapters-config.json
}

interface Chapter {
  id: string;
  title: string;
  order: number;
  sections: string[];
}
```

**Usage in Dashboard**:
```tsx
import chaptersConfig from '@site/docs/chapters-config.json';

<ProgressTracker
  userId={currentUser.id}
  chapters={chaptersConfig.chapters}
/>
```

**API Endpoint**: `GET /api/progress?user_id=<uuid>` (returns all chapters)

---

## Implementation Checklist for Chapter 1 Components

Before Chapter 2 can be fully functional, these components MUST be implemented:

- [ ] PersonalizeButton with `chapterId` prop
- [ ] TranslateButton with `chapterId` prop
- [ ] RAGChatbot with `chapterId` prop
- [ ] ProgressTracker with multi-chapter support
- [ ] Test all components with `chapterId="chapter-1"`
- [ ] Verify components work when passed `chapterId="chapter-2"`

---

## Chapter 2 Verification Steps

Once components are implemented for Chapter 1:

1. **Test PersonalizeButton in Chapter 2**:
   - Pass `chapterId="chapter-2"`
   - Verify API call includes `chapter_id: "chapter-2"`
   - Confirm personalized content adapts based on user profile

2. **Test TranslateButton in Chapter 2**:
   - Pass `chapterId="chapter-2"`
   - Verify Urdu translation preserves Gazebo technical terms
   - Confirm code examples remain in English

3. **Test RAGChatbot in Chapter 2**:
   - Pass `chapterId="chapter-2"`
   - Ask Chapter 2 questions (e.g., "What's the difference between URDF and SDF?")
   - Verify chatbot searches `chapter_2_embeddings` collection only
   - Confirm responses use Chapter 2 content, not Chapter 1

4. **Test ProgressTracker**:
   - Complete sections in both Chapter 1 and Chapter 2
   - Verify dashboard shows independent progress per chapter
   - Confirm global progress calculation: "1.5 / 2 chapters"

---

## Backend API Compatibility

**All APIs are already chapter-agnostic** - no backend changes needed:

- `/api/personalize` - accepts `chapter_id` parameter
- `/api/translate` - accepts `chapter_id` parameter
- `/api/chatbot/ask` - filters by `chapter_id`, searches correct Qdrant collection
- `/api/progress` - returns progress for all chapters
- `/api/progress/update` - updates specific `chapter_id` record

**Database Schema** - already supports multiple chapters:
- `progress_records` table has `chapter_id` VARCHAR(50) field
- `chat_messages` table has `chapter_id` VARCHAR(50) field

**Vector Store** - Chapter 2 collection created:
- Collection: `chapter_2_embeddings` (created by T005)
- Configuration: 1536 dimensions, COSINE distance (same as Chapter 1)

---

## Notes for Developers

**Component Reusability Pattern**:
- ALL components accept `chapterId` as a prop
- NO hardcoding of chapter-specific logic in components
- Backend services determine behavior based on `chapter_id` parameter
- This pattern scales to N chapters without component modifications

**Testing Strategy**:
- Implement components for Chapter 1 first
- Add `chapterId` prop to all component interfaces
- Test Chapter 1 functionality (`chapterId="chapter-1"`)
- Then test Chapter 2 functionality (`chapterId="chapter-2"`) without changing component code
- Only Markdown content and Qdrant embeddings differ between chapters

**Blockers for Chapter 2 Full Functionality**:
- Chapter 2 navigation and content can work WITHOUT components (T007-T010 deliverable)
- Bonus features (personalization, translation, RAG) require component implementation
- MVP (User Story 1) validates architecture - components can be placeholders initially
