---
description: "Task list for Embedding Pipeline Setup"
---

# Tasks: Embedding Pipeline Setup

**Input**: Design documents from `/specs/002-embedding-pipeline/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, quickstart.md

**Feature**: Build a single-file Python script that crawls the deployed Docusaurus site, extracts clean text, generates embeddings using Cohere, and stores them in Qdrant Cloud vector database for RAG retrieval.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- Backend embedding pipeline: `backend/embedding-pipeline/`
- Main script: `backend/embedding-pipeline/main.py`
- Tests: `backend/embedding-pipeline/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create backend/embedding-pipeline directory structure
- [X] T002 Initialize UV project with pyproject.toml in backend/embedding-pipeline/
- [X] T003 [P] Create .env.example file with required environment variables in backend/embedding-pipeline/
- [X] T004 [P] Create .gitignore file to exclude .env and __pycache__ in backend/embedding-pipeline/
- [X] T005 [P] Install dependencies: cohere, qdrant-client, beautifulsoup4, httpx, python-dotenv via UV
- [X] T006 [P] Install dev dependencies: pytest, pytest-asyncio via UV
- [X] T007 Create README.md with setup instructions in backend/embedding-pipeline/
- [X] T008 Create main.py skeleton with function stubs in backend/embedding-pipeline/
- [X] T009 Create tests/ directory structure in backend/embedding-pipeline/

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T010 Define DocumentPage dataclass in backend/embedding-pipeline/main.py
- [X] T011 [P] Define PageMetadata dataclass in backend/embedding-pipeline/main.py
- [X] T012 [P] Define TextChunk dataclass in backend/embedding-pipeline/main.py
- [X] T013 [P] Define ChunkMetadata dataclass in backend/embedding-pipeline/main.py
- [X] T014 Configure environment variable loading with python-dotenv in backend/embedding-pipeline/main.py
- [X] T015 [P] Initialize Cohere client with API key in backend/embedding-pipeline/main.py
- [X] T016 [P] Initialize Qdrant client with URL and API key in backend/embedding-pipeline/main.py
- [X] T017 Add logging configuration (LOG_LEVEL) in backend/embedding-pipeline/main.py
- [X] T018 [P] Add retry logic utility with exponential backoff in backend/embedding-pipeline/main.py

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Extract and Process Documentation Content (Priority: P1) 🎯 MVP

**Goal**: Crawl the deployed Docusaurus site and extract clean text content from each page to build a searchable knowledge base

**Independent Test**: Run crawler against test Docusaurus URL and verify extracted text is clean (no HTML tags, navigation) with proper metadata (chapter, section, URL)

### Implementation for User Story 1

- [X] T019 [US1] Implement get_all_urls function to fetch sitemap.xml from base URL in backend/embedding-pipeline/main.py
- [X] T020 [US1] Add URL extraction from sitemap.xml with BeautifulSoup in backend/embedding-pipeline/main.py
- [X] T021 [US1] Add URL deduplication and filtering logic (exclude /blog, /api-reference) in backend/embedding-pipeline/main.py
- [X] T022 [US1] Add fallback crawling logic for sites without sitemap in backend/embedding-pipeline/main.py
- [X] T023 [US1] Implement extract_text_from_url function with httpx async fetch in backend/embedding-pipeline/main.py
- [X] T024 [US1] Add BeautifulSoup HTML parsing to find main content container in backend/embedding-pipeline/main.py
- [X] T025 [US1] Extract page metadata (chapter from breadcrumbs/URL, section from h2, title from h1) in backend/embedding-pipeline/main.py
- [X] T026 [US1] Remove navigation, footer, sidebar, and ToC elements in backend/embedding-pipeline/main.py
- [X] T027 [US1] Preserve code blocks with language identifiers during extraction in backend/embedding-pipeline/main.py
- [X] T028 [US1] Return DocumentPage object with clean text and metadata in backend/embedding-pipeline/main.py
- [X] T029 [US1] Add error handling for HTTP errors and HTML parsing failures in backend/embedding-pipeline/main.py
- [X] T030 [US1] Add logging for crawling progress (pages discovered, processed) in backend/embedding-pipeline/main.py
- [X] T031 [US1] Implement chunk_text function with recursive character splitting in backend/embedding-pipeline/main.py
- [X] T032 [US1] Add token estimation heuristic (length / 4 characters per token) in backend/embedding-pipeline/main.py
- [X] T033 [US1] Implement splitting on paragraph boundaries (\n\n) first, then sentences (.), then words in backend/embedding-pipeline/main.py
- [X] T034 [US1] Add chunk overlap logic (last 100 tokens of chunk N in chunk N+1) in backend/embedding-pipeline/main.py
- [X] T035 [US1] Generate chunk_id using MD5(url + chunk_index) in backend/embedding-pipeline/main.py
- [X] T036 [US1] Attach ChunkMetadata to each TextChunk in backend/embedding-pipeline/main.py
- [X] T037 [US1] Add validation for chunk size (10-2000 chars, ≤500 tokens) in backend/embedding-pipeline/main.py

**Checkpoint**: At this point, crawling, extraction, and chunking should work end-to-end and be testable independently

---

## Phase 4: User Story 2 - Generate Vector Embeddings (Priority: P2)

**Goal**: Convert extracted text into vector embeddings using Cohere to enable semantic search

**Independent Test**: Take sample extracted text, send to Cohere API, verify valid vectors (1024 dimensions) returned

### Implementation for User Story 2

- [X] T038 [US2] Implement embed function to batch text chunks (up to 96 per request) in backend/embedding-pipeline/main.py
- [X] T039 [US2] Call Cohere API with model="embed-english-v3.0", input_type="search_document" in backend/embedding-pipeline/main.py
- [X] T040 [US2] Add truncate="END" parameter for chunks exceeding token limit in backend/embedding-pipeline/main.py
- [X] T041 [US2] Parse Cohere API response and extract embedding vectors in backend/embedding-pipeline/main.py
- [X] T042 [US2] Validate vector dimensions (must equal 1024) in backend/embedding-pipeline/main.py
- [X] T043 [US2] Validate all vector values are finite floats (no NaN, Inf) in backend/embedding-pipeline/main.py
- [X] T044 [US2] Pair each embedding vector with its source TextChunk in backend/embedding-pipeline/main.py
- [X] T045 [US2] Add retry logic for Cohere API errors with exponential backoff in backend/embedding-pipeline/main.py
- [X] T046 [US2] Add rate limiting to respect Cohere API limits (100 req/min free tier) in backend/embedding-pipeline/main.py
- [X] T047 [US2] Add logging for embedding generation progress (chunks processed, API calls) in backend/embedding-pipeline/main.py
- [X] T048 [US2] Handle batch failures by retrying individual chunks in backend/embedding-pipeline/main.py

**Checkpoint**: At this point, embedding generation should work independently and produce valid vectors

---

## Phase 5: User Story 3 - Store Embeddings in Qdrant (Priority: P3)

**Goal**: Store generated embeddings in Qdrant vector database with metadata to enable fast semantic retrieval

**Independent Test**: Take sample embeddings and metadata, store in Qdrant, verify retrieval via vector similarity search with metadata filtering

### Implementation for User Story 3

- [X] T049 [US3] Implement create_collection function to check if collection exists in backend/embedding-pipeline/main.py
- [X] T050 [US3] Create collection with VectorParams: size=1024, distance=COSINE in backend/embedding-pipeline/main.py
- [X] T051 [US3] Add collection existence check to avoid recreating in backend/embedding-pipeline/main.py
- [X] T052 [US3] Add logging for collection creation status in backend/embedding-pipeline/main.py
- [X] T053 [US3] Implement save_chunk_to_qdrant function to prepare PointStruct objects in backend/embedding-pipeline/main.py
- [X] T054 [US3] Use chunk_id as Qdrant point ID for idempotent upserts in backend/embedding-pipeline/main.py
- [X] T055 [US3] Build payload with chapter, section, page_title, source_url, text, chunk_index, created_at in backend/embedding-pipeline/main.py
- [X] T056 [US3] Batch upsert points (100 points per batch) to Qdrant collection in backend/embedding-pipeline/main.py
- [X] T057 [US3] Add retry logic for Qdrant upsert errors with exponential backoff in backend/embedding-pipeline/main.py
- [X] T058 [US3] Handle batch upsert failures by retrying with smaller batch size in backend/embedding-pipeline/main.py
- [X] T059 [US3] Return count of successfully upserted vectors in backend/embedding-pipeline/main.py
- [X] T060 [US3] Add logging for storage progress (vectors stored, batch count) in backend/embedding-pipeline/main.py
- [X] T061 [US3] Log failed batches to failed_batches.json for manual retry in backend/embedding-pipeline/main.py

**Checkpoint**: All user stories should now be independently functional - full pipeline works end-to-end

---

## Phase 6: Pipeline Orchestration

**Purpose**: Integrate all user stories into cohesive end-to-end pipeline

- [X] T062 Implement main function to orchestrate pipeline steps in backend/embedding-pipeline/main.py
- [X] T063 Add Step 1: Call get_all_urls to crawl documentation URLs in backend/embedding-pipeline/main.py
- [X] T064 Add Step 2: Loop through URLs and call extract_text_from_url in backend/embedding-pipeline/main.py
- [X] T065 Add Step 3: Call chunk_text on each DocumentPage in backend/embedding-pipeline/main.py
- [X] T066 Add Step 4: Batch TextChunks and call embed function in backend/embedding-pipeline/main.py
- [X] T067 Add Step 5: Call create_collection to ensure Qdrant collection exists in backend/embedding-pipeline/main.py
- [X] T068 Add Step 6: Call save_chunk_to_qdrant to store embeddings in backend/embedding-pipeline/main.py
- [X] T069 Add progress logging between each pipeline step in backend/embedding-pipeline/main.py
- [X] T070 Add summary statistics at end (total pages, chunks, vectors, time elapsed) in backend/embedding-pipeline/main.py
- [X] T071 Add error handling for pipeline interruptions in backend/embedding-pipeline/main.py
- [X] T072 Add entry point: if __name__ == "__main__": asyncio.run(main()) in backend/embedding-pipeline/main.py
- [X] T073 Log failed URLs to failed_urls.txt for manual review in backend/embedding-pipeline/main.py

---

## Phase 7: Testing (Optional - explicitly requested in spec)

**Purpose**: Validate each pipeline component works correctly

- [ ] T074 [P] Create test_crawler.py with test for get_all_urls function in backend/embedding-pipeline/tests/
- [ ] T075 [P] Create test_extraction.py with test for extract_text_from_url function in backend/embedding-pipeline/tests/
- [ ] T076 [P] Create test_chunking.py with tests for chunk_text function in backend/embedding-pipeline/tests/
- [ ] T077 [P] Create test_embedding.py with mock tests for embed function in backend/embedding-pipeline/tests/
- [ ] T078 [P] Create test_qdrant.py with tests for create_collection and save_chunk_to_qdrant in backend/embedding-pipeline/tests/
- [ ] T079 Add integration test for full pipeline with sample URL in backend/embedding-pipeline/tests/test_integration.py
- [ ] T080 Add test for idempotency (re-running same URL replaces old embedding) in backend/embedding-pipeline/tests/test_integration.py
- [ ] T081 Run pytest to validate all tests pass in backend/embedding-pipeline/

---

## Phase 8: Documentation & Validation

**Purpose**: Complete documentation and validate setup instructions

- [ ] T082 [P] Update README.md with complete usage instructions in backend/embedding-pipeline/
- [ ] T083 [P] Add troubleshooting section to README.md in backend/embedding-pipeline/
- [ ] T084 Create verify.py script to check Qdrant collection status in backend/embedding-pipeline/
- [ ] T085 Validate quickstart.md instructions work end-to-end
- [ ] T086 Add performance metrics to README.md (expected crawl time, embedding time) in backend/embedding-pipeline/
- [ ] T087 Document required environment variables in README.md in backend/embedding-pipeline/

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T088 [P] Add type hints to all function signatures in backend/embedding-pipeline/main.py
- [ ] T089 [P] Add docstrings to all functions in backend/embedding-pipeline/main.py
- [ ] T090 Review and optimize batch sizes for performance in backend/embedding-pipeline/main.py
- [ ] T091 Add configuration validation at startup (check all required env vars) in backend/embedding-pipeline/main.py
- [ ] T092 Add graceful shutdown handling for Ctrl+C in backend/embedding-pipeline/main.py
- [ ] T093 Review constitution compliance (Cohere model, Qdrant collection structure)
- [ ] T094 Update CLAUDE.md with embedding pipeline technologies and commands
- [ ] T095 Run full pipeline against production URL and verify results in Qdrant dashboard

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (US1 → US2 → US3)
- **Orchestration (Phase 6)**: Depends on all user stories (US1, US2, US3) being complete
- **Testing (Phase 7)**: Can run in parallel with implementation or after
- **Documentation (Phase 8)**: Depends on core implementation being complete
- **Polish (Phase 9)**: Depends on all previous phases

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Consumes output from US1 but can be tested independently with mock data
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Consumes output from US2 but can be tested independently with sample embeddings

### Within Each User Story

- Tasks within a story follow sequential implementation order
- Tasks marked [P] within same story can run in parallel if in different logical sections
- Complete story implementation before integration testing

### Parallel Opportunities

- All Setup tasks (T003, T004, T005, T006) can run in parallel
- All Foundational dataclass definitions (T011, T012, T013) can run in parallel
- Foundational client initialization (T015, T016) can run in parallel
- All Testing tasks (T074-T078) can run in parallel
- Documentation tasks (T082, T083) can run in parallel
- Once Foundational phase completes, all three user stories can start in parallel (if team capacity allows)

---

## Parallel Example: Foundational Phase

```bash
# Launch all dataclass definitions together:
Task: "Define PageMetadata dataclass in backend/embedding-pipeline/main.py"
Task: "Define TextChunk dataclass in backend/embedding-pipeline/main.py"
Task: "Define ChunkMetadata dataclass in backend/embedding-pipeline/main.py"

# Launch client initialization together:
Task: "Initialize Cohere client with API key in backend/embedding-pipeline/main.py"
Task: "Initialize Qdrant client with URL and API key in backend/embedding-pipeline/main.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T009)
2. Complete Phase 2: Foundational (T010-T018) - CRITICAL
3. Complete Phase 3: User Story 1 (T019-T037)
4. **STOP and VALIDATE**: Test crawling, extraction, chunking independently
5. Proceed to US2 and US3

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Validate crawling works
3. Add User Story 2 → Test independently → Validate embeddings work
4. Add User Story 3 → Test independently → Validate Qdrant storage works
5. Add Orchestration (Phase 6) → Full pipeline runs end-to-end
6. Each phase adds value without breaking previous work

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1 (Crawling & Chunking)
   - Developer B: User Story 2 (Embeddings)
   - Developer C: User Story 3 (Qdrant Storage)
3. Stories complete independently
4. Team integrates in Phase 6 (Orchestration)

---

## Task Summary

- **Total Tasks**: 95
- **Setup**: 9 tasks
- **Foundational**: 9 tasks
- **User Story 1** (Crawling & Chunking): 19 tasks
- **User Story 2** (Embeddings): 11 tasks
- **User Story 3** (Qdrant Storage): 13 tasks
- **Orchestration**: 12 tasks
- **Testing**: 8 tasks
- **Documentation**: 6 tasks
- **Polish**: 8 tasks

**Parallel Opportunities**: 15 tasks marked [P] can run in parallel within their phases

**MVP Scope**: Setup (9) + Foundational (9) + US1 (19) = 37 tasks to validate core extraction

---

## Notes

- [P] tasks = different files or independent logical sections, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Single-file implementation (main.py) means sequential execution within each story
- Stop at any checkpoint to validate story independently
- All paths assume backend/embedding-pipeline/ as project root
- Environment variables must be configured before running (see .env.example)
- Tests are included per spec requirement (FR-010: progress logging, SC-002: HTML tag leakage validation)
