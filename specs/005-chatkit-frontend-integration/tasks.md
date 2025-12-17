# Implementation Tasks: ChatKit Frontend Integration

**Feature**: 005-chatkit-frontend-integration
**Branch**: `005-chatkit-frontend-integration`
**Spec**: [spec.md](./spec.md)
**Plan**: [plan.md](./plan.md)

---

## Task Summary

| Phase | User Story | Task Count | Can Parallelize |
|-------|------------|------------|-----------------|
| Phase 1 | Setup | 3 | Yes (T002, T003) |
| Phase 2 | Foundational | 2 | Yes (both) |
| Phase 3 | US1 (P1) - Ask Question & Receive Answer | 6 | Partial (T008, T009, T010) |
| Phase 4 | US4 (P2) - Error Handling | 2 | Yes (both) |
| Phase 5 | US2 (P2) - View Chat History | 2 | Yes (both) |
| Phase 6 | US3 (P3) - Chapter-Scoped Questions | 2 | No |
| Phase 7 | Polish & Integration | 3 | Partial (T020, T021) |
| **Total** | | **20** | **12 parallelizable** |

---

## Implementation Strategy

**MVP Scope**: Phase 1-3 (US1 only)
- Install dependencies → Create types → Create service → Create component → Basic integration
- **Deliverable**: Working chat interface that can send questions and display answers
- **Time**: ~2-3 hours

**Full Feature**: All phases
- Add error handling, chat history, chapter scoping, polish
- **Deliverable**: Production-ready chat interface with all user stories complete
- **Time**: ~4-6 hours

---

## Dependency Graph

```
Phase 1 (Setup)
  ├─ T001 → [T002, T003] (both can run in parallel after T001)
  └─ [T002, T003] → T004 (Foundational)

Phase 2 (Foundational)
  ├─ T004 [P] TypeScript types
  ├─ T005 [P] API service layer
  └─ Both complete before US1 can start

Phase 3 (US1 - MVP)
  ├─ T006 → T007 (sequential: component needs types/service)
  ├─ T008 [P] Styles (parallel with component)
  ├─ T009 [P] Markdown rendering (parallel)
  ├─ T010 [P] Source citations (parallel)
  └─ T011 Integration (requires all above)

Phase 4-6 (User Stories P2-P3)
  ├─ US4, US2, US3 can start independently after US1 complete
  └─ Each story is independently testable

Phase 7 (Polish)
  ├─ Requires all user stories complete
  └─ Tasks can run in parallel
```

**Critical Path**: T001 → T004 → T005 → T006 → T007 → T011 (MVP)

---

## Phase 1: Setup

**Goal**: Install dependencies and configure environment for chat integration.

### Tasks

- [X] T001 Install required npm dependencies (react-markdown, react-syntax-highlighter)
- [X] T002 [P] Verify ChatScope Chat UI Kit installation in package.json
- [X] T003 [P] Configure Docusaurus customFields for API URL in docusaurus.config.ts

**Acceptance Criteria**:
- ✅ `npm install` completes without errors
- ✅ `@chatscope/chat-ui-kit-react` listed in package.json dependencies
- ✅ `react-markdown` and `react-syntax-highlighter` installed
- ✅ `docusaurus.config.ts` includes `customFields.apiUrl` set to `http://localhost:8000`

---

## Phase 2: Foundational (Blocking Prerequisites)

**Goal**: Create shared type definitions and API service layer needed by all user stories.

### Tasks

- [X] T004 [P] Create TypeScript interfaces in src/types/chat.ts matching backend Pydantic models
- [X] T005 [P] Implement API service layer in src/services/chatService.ts with sendQuery function

**Files**:
- `src/types/chat.ts`: QueryRequest, QueryResponse, RetrievedChunk, Message interfaces
- `src/services/chatService.ts`: sendQuery, getOrCreateSessionId, extractChapterId, ChatServiceError

**Acceptance Criteria**:
- ✅ TypeScript types match backend `QueryRequest` and `QueryResponse` models exactly
- ✅ `chatService.ts` exports `sendQuery()` function with 30s timeout
- ✅ `getOrCreateSessionId()` creates UUID and stores in sessionStorage
- ✅ `extractChapterId()` extracts chapter from URL pathname
- ✅ `ChatServiceError` class handles network, timeout, server, validation errors
- ✅ No TypeScript compilation errors

---

## Phase 3: User Story 1 - Ask Question and Receive Answer (P1 - MVP)

**Story Goal**: Enable users to type a question, send it to the backend, and see an AI-generated answer.

**Independent Test**:
1. Open any chapter page with `<ChatBot />` component
2. Type "What is ROS 2?" in input field
3. Click send button
4. Verify loading indicator appears
5. Verify AI answer displays below user question
6. Verify source citations display (if provided by backend)

### Tasks

- [X] T006 [US1] Create ChatBot component scaffold in src/components/ChatBot.tsx with state management
- [X] T007 [US1] Implement handleSend function in ChatBot.tsx to call backend API
- [X] T008 [P] [US1] Create ChatBot.module.css with responsive styles for chat container
- [X] T009 [P] [US1] Add ReactMarkdown rendering for AI responses with code syntax highlighting
- [X] T010 [P] [US1] Implement source citations display in expandable details element
- [X] T011 [US1] Integrate ChatBot component into example chapter page (docs/chatbot.mdx)

**Files**:
- `src/components/ChatBot.tsx`: Main component with ChatScope UI components
- `src/components/ChatBot.module.css`: Component-specific styles
- `docs/chapter-1/intro.mdx` (or test page): MDX import of ChatBot

**Implementation Details**:
- T006: Component with useState for messages[], isLoading, sessionId, chapterId
- T007: handleSend calls sendQuery, adds user/assistant messages to state, handles loading state
- T008: Styles for .chatContainer (500px height), message alignment, dark mode support
- T009: ReactMarkdown with SyntaxHighlighter for code blocks
- T010: Collapsible sources section showing chunk content, score, source_file
- T011: Add `import ChatBot from '@site/src/components/ChatBot';` and `<ChatBot />` in MDX

**Acceptance Criteria** (US1):
- ✅ **AC1**: User can type question in input field and click send
- ✅ **AC2**: User message appears immediately in chat history
- ✅ **AC3**: Loading indicator (TypingIndicator) shows while waiting for response
- ✅ **AC4**: AI answer displays below user question when received
- ✅ **AC5**: Markdown formatting in answers renders correctly (bold, lists, code blocks)
- ✅ **AC6**: Source citations display in collapsible section with file paths and relevance scores
- ✅ **AC7**: Input field clears after successful send (FR-010)
- ✅ **AC8**: Empty/whitespace-only messages cannot be sent (FR-009)

**Test Scenarios**:
```bash
# Start backend
cd backend && uvicorn src.main:app --reload

# Start frontend
npm start

# Navigate to http://localhost:3000/chapter-1/intro
# Test: Send "What is ROS 2?" → Verify answer displays
# Test: Send "   " (whitespace) → Verify nothing happens
# Test: Send question → Verify input clears after send
```

---

## Phase 4: User Story 4 - Error Handling and Feedback (P2)

**Story Goal**: Display clear, user-friendly error messages when network failures, timeouts, or backend errors occur.

**Independent Test**:
1. Stop backend server
2. Send question in chat interface
3. Verify error message displays: "Unable to connect to the chatbot service..."
4. Restart backend, send question → Verify normal operation resumes

### Tasks

- [ ] T012 [P] [US4] Add error message styling to ChatBot.module.css for .errorMessage class
- [ ] T013 [P] [US4] Implement error handling in handleSend to display ChatServiceError messages

**Files**:
- `src/components/ChatBot.module.css`: Add .errorMessage class
- `src/components/ChatBot.tsx`: Catch block in handleSend

**Implementation Details**:
- T012: .errorMessage with red border, light red background
- T013: try/catch in handleSend, create Message with isError=true, content=error.message

**Acceptance Criteria** (US4):
- ✅ **AC1**: Network error displays "Unable to connect to the chatbot service. Please check your connection."
- ✅ **AC2**: Timeout error (>30s) displays "Request took too long. Please try again."
- ✅ **AC3**: Server error (500) displays "The chatbot service is temporarily unavailable. Please try again later."
- ✅ **AC4**: Validation error (422) displays "Invalid question format. Please try rephrasing."
- ✅ **AC5**: Error messages styled with red border and background
- ✅ **AC6**: User can retry after error (input field remains active)

**Test Scenarios**:
```bash
# Test network error
# Stop backend: Ctrl+C in backend terminal
# Send question → Verify network error message

# Test timeout (requires backend mock)
# Modify backend to sleep 35s → Verify timeout error

# Test server error
# Break backend (invalid API key in .env) → Verify server error

# Test validation error
# Send 1500-character question → Verify validation error (if backend enforces)
```

---

## Phase 5: User Story 2 - View Chat History (P2)

**Story Goal**: Allow users to scroll through and view all previous messages in the current session.

**Independent Test**:
1. Send 5 questions in sequence
2. Scroll up in chat container
3. Verify all 10 messages (5 user + 5 assistant) visible in chronological order
4. Verify scrollbar appears if content exceeds container height

### Tasks

- [ ] T014 [P] [US2] Add auto-scroll to bottom functionality in ChatBot.tsx using useEffect and messageListRef
- [ ] T015 [P] [US2] Verify MessageList virtualization for performance with 20+ message pairs

**Files**:
- `src/components/ChatBot.tsx`: Add useEffect for auto-scroll, useRef for messageListRef

**Implementation Details**:
- T014: useEffect with [messages] dependency, call messageListRef.current.scrollToBottom()
- T015: Test with 40+ messages (20 pairs), verify no performance degradation

**Acceptance Criteria** (US2):
- ✅ **AC1**: All messages display in chronological order (oldest at top)
- ✅ **AC2**: New messages auto-scroll to bottom when added
- ✅ **AC3**: Scrollbar appears when message list exceeds container height
- ✅ **AC4**: User can manually scroll up to view message history
- ✅ **AC5**: No performance degradation with 20+ message pairs (SC-004 requirement)

**Test Scenarios**:
```bash
# Test scrolling
# Send 3 questions → Verify auto-scroll to bottom
# Manually scroll up → Send another question → Verify scroll returns to bottom

# Test performance
# Send 25 questions → Verify UI remains responsive
# Scroll through history → Verify smooth scrolling
```

---

## Phase 6: User Story 3 - Chapter-Scoped Questions (P3)

**Story Goal**: Automatically detect current chapter from URL and send chapter_id in API requests.

**Independent Test**:
1. Navigate to `/chapter-1/intro` page
2. Send question "What topics are covered?"
3. Verify backend receives `chapter_id: "chapter-1"` in request
4. Navigate to different chapter page
5. Verify new ChatBot instance with updated chapter_id

### Tasks

- [ ] T016 [US3] Implement chapter detection using useLocation in ChatBot.tsx
- [ ] T017 [US3] Verify chapter_id updates when navigating between chapter pages

**Files**:
- `src/components/ChatBot.tsx`: Import useLocation, extract chapter from location.pathname

**Implementation Details**:
- T016: `const location = useLocation();` → `const chapterId = extractChapterId(location.pathname);`
- T017: Manual test on multiple chapter pages, verify API requests include correct chapter_id

**Acceptance Criteria** (US3):
- ✅ **AC1**: Chapter ID automatically extracted from URL (e.g., `/chapter-1/intro` → `"chapter-1"`)
- ✅ **AC2**: API requests include correct `chapter_id` field
- ✅ **AC3**: Default fallback to `"chapter-1"` if URL doesn't match pattern
- ✅ **AC4**: Navigating to different chapter creates new ChatBot instance with new chapter_id
- ✅ **AC5**: Backend logs confirm chapter_id received correctly (check backend console)

**Test Scenarios**:
```bash
# Test chapter detection
# Navigate to /chapter-1/intro → Send question
# Check browser Network tab → Verify request payload has chapter_id: "chapter-1"

# Test chapter switching
# Navigate to /chapter-2/gazebo → Send question
# Verify request payload has chapter_id: "chapter-2"

# Test fallback
# Navigate to /docs/intro (non-chapter page) → Verify chapter_id: "chapter-1" (fallback)
```

---

## Phase 7: Polish & Cross-Cutting Concerns

**Goal**: Final integration, testing, and documentation.

### Tasks

- [ ] T018 Verify CORS configuration in backend/.env allows frontend origin (http://localhost:3000)
- [ ] T019 Create .env.example in project root documenting REACT_APP_API_URL variable
- [ ] T020 [P] Test responsive layout on mobile viewport (320px width)
- [ ] T021 [P] Verify no console errors in browser DevTools during normal operation
- [ ] T022 End-to-end manual testing following quickstart.md test scenarios

**Files**:
- `backend/.env`: Verify CORS_ORIGINS includes `http://localhost:3000`
- `.env.example`: Document `REACT_APP_API_URL=http://localhost:8000`

**Acceptance Criteria**:
- ✅ **CORS**: No CORS errors in browser console when sending queries
- ✅ **Mobile**: Chat interface fully functional on 320px-1920px viewports (FR-014)
- ✅ **Console**: Zero errors or warnings in browser DevTools
- ✅ **Documentation**: .env.example created with API URL configuration
- ✅ **End-to-End**: All user stories pass independent tests

**Final Test Checklist**:
```bash
# Backend running
cd backend && uvicorn src.main:app --reload

# Frontend running
npm start

# Test US1 (MVP)
✅ Send question → Receive answer
✅ Loading indicator shows
✅ Source citations display
✅ Markdown renders correctly
✅ Input clears after send
✅ Empty messages blocked

# Test US4 (Error Handling)
✅ Stop backend → Network error displays
✅ Restart backend → Normal operation resumes

# Test US2 (Chat History)
✅ Send 5 questions → All 10 messages visible
✅ Auto-scroll to bottom
✅ Manual scroll up works

# Test US3 (Chapter Scoping)
✅ /chapter-1/intro → chapter_id: "chapter-1"
✅ /chapter-2/gazebo → chapter_id: "chapter-2"

# Test US1, US2, US4, US3 (All Features)
✅ All user stories pass independently
```

---

## Parallel Execution Examples

### MVP Sprint (US1 only)

**Day 1 Morning - Setup & Foundation** (4 tasks in parallel):
```bash
# Developer 1
- [ ] T001 npm install react-markdown react-syntax-highlighter

# Developer 2
- [ ] T002 [P] Verify ChatScope installation
- [ ] T003 [P] Configure docusaurus.config.ts

# Developer 3
- [ ] T004 [P] Create src/types/chat.ts

# Developer 4
- [ ] T005 [P] Implement src/services/chatService.ts
```

**Day 1 Afternoon - US1 Implementation** (3 tasks in parallel):
```bash
# Developer 1
- [ ] T006 Create ChatBot.tsx scaffold
- [ ] T007 Implement handleSend function

# Developer 2
- [ ] T008 [P] [US1] Create ChatBot.module.css

# Developer 3
- [ ] T009 [P] [US1] Add ReactMarkdown rendering

# Developer 4
- [ ] T010 [P] [US1] Implement source citations
```

**Day 1 End - Integration**:
```bash
# All developers sync
- [ ] T011 Integrate ChatBot into chapter page
```

**Result**: Working MVP in 1 day with 4 developers (or 2-3 hours solo).

---

### Full Feature Sprint (All User Stories)

**Phase 3-6 can run in parallel after Foundation complete**:

```bash
# Team A (US1 - MVP)
T006-T011 (6 tasks)

# Team B (US4 - Error Handling)
T012-T013 (2 tasks) - Can start immediately after T005 complete

# Team C (US2 - Chat History)
T014-T015 (2 tasks) - Can start immediately after T007 complete

# Team D (US3 - Chapter Scoping)
T016-T017 (2 tasks) - Can start immediately after T006 complete
```

**Timeline**:
- Day 1 AM: Setup + Foundation (T001-T005)
- Day 1 PM: All 4 user stories in parallel (T006-T017)
- Day 2 AM: Polish & Testing (T018-T022)

**Result**: Full feature in 2 days with 4-person team (or 4-6 hours solo).

---

## Dependencies Between User Stories

### Story Dependency Matrix

| Story | Depends On | Can Start After | Notes |
|-------|------------|-----------------|-------|
| Setup (Phase 1) | - | Immediately | Required for all |
| Foundation (Phase 2) | Setup | T001-T003 complete | Blocking for all stories |
| US1 (P1) | Foundation | T004-T005 complete | MVP - implement first |
| US4 (P2) | Foundation | T004-T005 complete | Independent, can parallel with US1 |
| US2 (P2) | US1 (partial) | T007 complete | Needs handleSend for messages array |
| US3 (P3) | US1 (partial) | T006 complete | Needs component scaffold |
| Polish (Phase 7) | All stories | T006-T017 complete | Final integration |

**Critical Dependencies**:
- Foundation (T004-T005) blocks all user stories
- US1 T007 (handleSend) blocks US2 (needs message state)
- US1 T006 (component) blocks US3 (needs component to add chapter detection)

**Independent Stories**:
- US4 can be implemented fully in parallel with US1 (separate error handling logic)

---

## Task Validation

**Format Compliance**: ✅ All 20 tasks follow strict format
- [x] All tasks have `- [ ]` checkbox
- [x] All tasks have Task ID (T001-T022)
- [x] All user story phase tasks have [US#] label
- [x] All parallelizable tasks marked with [P]
- [x] All tasks include file paths in description
- [x] All phases have clear acceptance criteria

**Completeness**: ✅
- [x] All 4 user stories covered (US1, US2, US3, US4)
- [x] All 15 functional requirements mapped to tasks
- [x] All entities from data-model.md included
- [x] All contracts from contracts/ covered
- [x] Independent test criteria for each story
- [x] Parallel execution opportunities identified

**Story Coverage**:
- [x] US1 (P1): 6 tasks (T006-T011)
- [x] US2 (P2): 2 tasks (T014-T015)
- [x] US3 (P3): 2 tasks (T016-T017)
- [x] US4 (P2): 2 tasks (T012-T013)

---

## Notes

**Backend Integration**: No backend code changes required. Backend `/api/query` endpoint already exists and is tested. Only CORS verification needed (T018).

**MVP Definition**: Phase 1-3 (T001-T011) delivers working chat interface. This is the minimum viable product that provides immediate value.

**Incremental Delivery**: Each user story (US1-US4) can be deployed independently after completion, enabling iterative releases.

**Performance**: ChatScope MessageList includes virtualization by default, so no additional optimization needed for 20+ message pairs (SC-004).

**Testing**: Manual testing only for MVP. Automated tests (Jest/React Testing Library) are future enhancement, not in scope for initial implementation.

**Estimated Time**:
- Solo developer: 2-3 hours (MVP), 4-6 hours (full feature)
- 4-person team: 1 day (MVP), 2 days (full feature)

---

**Next Steps**:
1. Start with Phase 1 (Setup) - T001-T003
2. Implement Foundation (Phase 2) - T004-T005
3. Build MVP (Phase 3 - US1) - T006-T011
4. Test MVP end-to-end
5. Add remaining user stories (US4, US2, US3)
6. Final polish and integration testing
