---
description: "Implementation tasks for retrieval verification feature"
---

# Tasks: Embedding Retrieval & Pipeline Verification

**Input**: Design documents from `/specs/003-retrieval-verification/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/diagnostic-report.json

**Tests**: Tests are included as requested in the specification and plan

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Project type**: Single-file script in `backend/embedding-pipeline/`
- **Tests**: `backend/embedding-pipeline/tests/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure for verification script

- [ ] T001 Create `backend/embedding-pipeline/retrieval_test.py` file with basic imports and structure
- [ ] T002 [P] Create test directory structure: `backend/embedding-pipeline/tests/` with `__init__.py`
- [ ] T003 [P] Create fixtures directory: `backend/embedding-pipeline/tests/fixtures/` for test data

**Acceptance Criteria for Phase 1**:
- File `backend/embedding-pipeline/retrieval_test.py` exists with skeleton structure
- Test directories created with proper Python package structure
- Fixtures directory ready for test query JSON files

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T004 Define dataclass entities in `retrieval_test.py`: QdrantCollection, StoredVector, ChunkMetadata (from data-model.md)
- [ ] T005 [P] Define dataclass entities in `retrieval_test.py`: SearchQuery, SearchResult (from data-model.md)
- [ ] T006 [P] Define dataclass entities in `retrieval_test.py`: ValidationCheck, DiagnosticReport (from data-model.md)
- [ ] T007 Initialize Cohere client in `retrieval_test.py` using API key from .env (COHERE_API_KEY)
- [ ] T008 Initialize Qdrant client in `retrieval_test.py` using credentials from .env (QDRANT_URL, QDRANT_API_KEY)
- [ ] T009 Implement `load_collection_metadata()` function to retrieve QdrantCollection info from `rag_embedding`
- [ ] T010 Implement error handling wrapper for API calls with exponential backoff retry logic (max 3 retries)

**Acceptance Criteria for Phase 2**:
- All 7 dataclass entities defined with type hints and validation methods
- Cohere client successfully connects using .env credentials
- Qdrant client successfully connects to `rag_embedding` collection
- `load_collection_metadata()` returns QdrantCollection with vector_count, dimensions, distance_metric
- Error handling catches API failures and retries with exponential backoff

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Validate Pipeline Health and Data Integrity (Priority: P1) 🎯 MVP

**Goal**: Verify all stored vectors have correct metadata, proper dimensionality, and no duplicates

**Independent Test**: Run diagnostic queries against Qdrant collection and validate vector count, dimensions, metadata structure, uniqueness constraints

### Tests for User Story 1

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T011 [P] [US1] Unit test for `check_collection_exists()` in `tests/test_validation.py` - verify function detects existing vs missing collection
- [ ] T012 [P] [US1] Unit test for `check_dimensions()` in `tests/test_validation.py` - verify 1024-dim vectors pass, wrong dims fail
- [ ] T013 [P] [US1] Unit test for `check_metadata_completeness()` in `tests/test_validation.py` - verify all required fields present
- [ ] T014 [P] [US1] Unit test for `check_duplicates()` in `tests/test_validation.py` - verify duplicate detection logic works
- [ ] T015 [P] [US1] Unit test for `check_valid_urls()` in `tests/test_validation.py` - verify HTTPS URL validation
- [ ] T016 [P] [US1] Unit test for `check_chunk_lengths()` in `tests/test_validation.py` - verify 10-2000 char bounds

### Implementation for User Story 1

- [ ] T017 [P] [US1] Implement `check_collection_exists()` in `retrieval_test.py` - P1 blocker validation (FR-001)
  - **Acceptance**: Returns ValidationCheck(status="PASS") if collection exists, "FAIL" otherwise
  - **Test**: Query Qdrant for collection "rag_embedding", handle 404 error

- [ ] T018 [P] [US1] Implement `check_non_empty()` in `retrieval_test.py` - P1 blocker validation
  - **Acceptance**: Returns ValidationCheck(status="PASS") if vector_count > 0, "FAIL" if empty
  - **Test**: Check collection_info.points_count > 0

- [ ] T019 [P] [US1] Implement `check_dimensions()` in `retrieval_test.py` - P1 blocker validation (FR-003)
  - **Acceptance**: Returns ValidationCheck(status="PASS") if all vectors have 1024 dimensions
  - **Test**: Retrieve sample vectors with `with_vectors=True`, validate len(vector) == 1024 for each

- [ ] T020 [US1] Implement `check_metadata_completeness()` in `retrieval_test.py` - P2 quality validation (FR-005)
  - **Acceptance**: Returns ValidationCheck(status="PASS") if all required fields present (chapter, page_title, source_url, text, chunk_index, created_at)
  - **Test**: Scroll through all vectors, check payload.get(field) is not None for each required field
  - **Details**: If failures found, populate affected_vectors list with UUIDs
  - **Depends on**: T018 (needs non-empty collection)

- [ ] T021 [US1] Implement `check_duplicates()` in `retrieval_test.py` - P2 quality validation (FR-004)
  - **Acceptance**: Returns ValidationCheck(status="PASS") if zero duplicate chunk_ids found
  - **Test**: Collect all chunk_id values, check for duplicates using Counter
  - **Details**: If duplicates found, list chunk_ids in ValidationCheck.details
  - **Depends on**: T018 (needs non-empty collection)

- [ ] T022 [US1] Implement `check_valid_urls()` in `retrieval_test.py` - P2 quality validation (FR-006)
  - **Acceptance**: Returns ValidationCheck(status="PASS") if all source_url fields are valid HTTPS URLs
  - **Test**: Validate each source_url starts with "https://" using regex or urlparse
  - **Details**: List invalid URLs in ValidationCheck.details if failures found
  - **Depends on**: T020 (needs metadata completeness check)

- [ ] T023 [US1] Implement `check_chunk_lengths()` in `retrieval_test.py` - P3 best-practice validation (FR-007)
  - **Acceptance**: Returns ValidationCheck(status="PASS") if all text chunks are 10-2000 characters
  - **Test**: Check 10 <= len(payload['text']) <= 2000 for each vector
  - **Details**: Report vectors outside bounds in ValidationCheck.details
  - **Depends on**: T020 (needs metadata completeness check)

- [ ] T024 [US1] Implement `run_validation_checks()` function in `retrieval_test.py` - orchestrate all validation checks
  - **Acceptance**: Runs all 7 validation checks (T017-T023) and returns List[ValidationCheck]
  - **Test**: Execute checks in priority order (P1 first, then P2, then P3)
  - **Optimization**: Run P2 checks in parallel using asyncio.gather()
  - **Depends on**: T017, T018, T019, T020, T021, T022, T023

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently. Running `uv run python retrieval_test.py` should validate all vectors and output validation results.

---

## Phase 4: User Story 2 - Perform Semantic Search Queries (Priority: P2)

**Goal**: Run semantic similarity searches and verify relevant document chunks are returned with high scores

**Independent Test**: Generate query embeddings for predefined test queries, search Qdrant, verify semantically relevant results with similarity scores > 0.6

### Tests for User Story 2

- [ ] T025 [P] [US2] Integration test for `embed_query()` in `tests/test_search.py` - verify Cohere API integration works
- [ ] T026 [P] [US2] Integration test for `semantic_search()` in `tests/test_search.py` - verify Qdrant search returns results
- [ ] T027 [US2] End-to-end test for semantic search pipeline in `tests/test_search.py` - verify full query→embedding→search→results flow

### Implementation for User Story 2

- [ ] T028 [P] [US2] Create test queries fixture in `tests/fixtures/sample_queries.json` with 10 diverse queries
  - **Acceptance**: JSON file with queries covering: ROS 2 basics, technical how-tos, vague queries, filtered queries
  - **Content**: Include expected result characteristics (topic, min_score) for each query
  - **Example queries**: "What is ROS 2?", "How to create a publisher?", "robotics simulation", "DDS middleware"

- [ ] T029 [US2] Implement `embed_query(query: str) -> List[float]` in `retrieval_test.py` (FR-008)
  - **Acceptance**: Returns 1024-dim embedding vector from Cohere embed-english-v3.0 model
  - **Test**: Call Cohere API with input_type="search_query", model="embed-english-v3.0"
  - **Error handling**: Retry with exponential backoff if API call fails
  - **Depends on**: T007 (Cohere client initialized)

- [ ] T030 [US2] Implement `semantic_search(query: SearchQuery) -> List[SearchResult]` in `retrieval_test.py` (FR-009, FR-010)
  - **Acceptance**: Returns top-K search results from Qdrant with similarity scores
  - **Test**: Call qdrant_client.search() with query_embedding, limit=query.top_k, score_threshold=query.min_score
  - **Metadata filtering**: Apply filter_criteria if provided (e.g., {"chapter": "Chapter 1"})
  - **Result mapping**: Convert Qdrant ScoredPoint to SearchResult with metadata
  - **Depends on**: T008 (Qdrant client initialized), T029 (query embedding)

- [ ] T031 [US2] Implement `load_test_queries()` function in `retrieval_test.py` - load queries from fixtures
  - **Acceptance**: Reads `tests/fixtures/sample_queries.json` and returns List[Dict] of test queries
  - **Error handling**: Raise clear error if fixture file missing or malformed
  - **Depends on**: T028 (fixtures file created)

- [ ] T032 [US2] Implement `run_semantic_search_tests()` function in `retrieval_test.py` (FR-011)
  - **Acceptance**: Executes all test queries from fixture and returns List[Tuple[SearchQuery, List[SearchResult]]]
  - **Test**: For each query: embed → search → validate top result score > 0.6
  - **Logging**: Print query text, top 3 results (score, chapter, title, text preview), retrieval time
  - **Parallel execution**: Run queries concurrently using asyncio.gather() for performance
  - **Depends on**: T029 (embed_query), T030 (semantic_search), T031 (load_test_queries)

- [ ] T033 [US2] Implement `validate_search_results()` helper in `retrieval_test.py` - check result relevance
  - **Acceptance**: Returns ValidationCheck indicating if search results meet quality thresholds
  - **Test**: Check top result score >= 0.6, results are diverse (not all from same chunk), results match query topic
  - **Depends on**: T032 (search results available)

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently. Running script should validate data AND perform semantic search tests.

---

## Phase 5: User Story 3 - Log and Report Pipeline Diagnostics (Priority: P3)

**Goal**: Clear diagnostic output showing collection stats, sample retrievals, validation results

**Independent Test**: Run verification script and confirm structured report includes collection stats, search results, pass/fail validation summary

### Tests for User Story 3

- [ ] T034 [P] [US3] Unit test for `DiagnosticReport.determine_status()` in `tests/test_validation.py` - verify status logic (PASS/FAIL/WARNING)
- [ ] T035 [P] [US3] Unit test for `DiagnosticReport.format_summary()` in `tests/test_validation.py` - verify console output format
- [ ] T036 [P] [US3] Unit test for `DiagnosticReport.to_json()` in `tests/test_validation.py` - verify JSON export matches schema

### Implementation for User Story 3

- [ ] T037 [US3] Implement `format_console_output()` function in `retrieval_test.py` (FR-012, FR-013)
  - **Acceptance**: Returns human-readable string with validation checks, search results, collection stats
  - **Format**: Use emoji indicators (✅ PASS, ❌ FAIL, ⚠️ WARNING), clear section headers, readable timestamps
  - **Content**: Collection stats (vector count, dimensions), validation results (P1/P2/P3 with details), search results (query, top 3 with scores), summary (total checks, passed/failed, recommendations)
  - **Depends on**: T024 (validation results), T032 (search results)

- [ ] T038 [US3] Implement `export_json()` function in `retrieval_test.py` (FR-014)
  - **Acceptance**: Returns JSON string matching contracts/diagnostic-report.json schema
  - **Test**: Call DiagnosticReport.to_json(), validate against JSON schema
  - **Content**: All validation checks, search results, collection stats, summary with counts
  - **Depends on**: T024 (validation results), T032 (search results)

- [ ] T039 [US3] Implement `generate_recommendations()` helper in `retrieval_test.py` (FR-014)
  - **Acceptance**: Returns List[str] of actionable next steps based on validation results
  - **Logic**: If all PASS → "Pipeline production-ready"; If P1 FAIL → "Re-run embedding pipeline"; If P2 FAIL → "Review metadata issues"; If search scores low → "Curate better test queries"
  - **Depends on**: T024 (validation results), T032 (search results)

- [ ] T040 [US3] Implement `print_sample_payloads()` helper in `retrieval_test.py` (FR-016)
  - **Acceptance**: Prints 3-5 sample vector payloads to illustrate data structure
  - **Format**: Show full metadata (chapter, section, page_title, source_url, text preview, chunk_index, created_at)
  - **Test**: Retrieve random sample of vectors using client.scroll(limit=5)
  - **Depends on**: T009 (collection metadata loaded)

- [ ] T041 [US3] Implement `log_error_with_context()` helper in `retrieval_test.py` (FR-015)
  - **Acceptance**: Logs errors with step name, error message, timestamp, continues execution
  - **Format**: Use Python logging module with ERROR level
  - **Behavior**: Don't crash script on non-critical errors, accumulate errors for final report

- [ ] T042 [US3] Implement `main()` async function in `retrieval_test.py` - orchestrate full verification pipeline
  - **Acceptance**: Runs entire pipeline: load collection → validate → search → report → output
  - **Flow**:
    1. Print banner with timestamp (FR-012)
    2. Load collection metadata (T009)
    3. Print sample payloads (T040)
    4. Run validation checks (T024)
    5. Run semantic search tests (T032)
    6. Generate diagnostic report with recommendations (T039)
    7. Print console output (T037)
    8. Export JSON report to file (T038)
    9. Print completion message with total runtime
  - **Error handling**: Wrap each step in try/except, log errors (T041), continue to next step
  - **Depends on**: All previous tasks

- [ ] T043 [US3] Add CLI argument parsing in `retrieval_test.py` - support command-line options
  - **Acceptance**: Supports `--verbose`, `--format json`, `--output <file>`, `--queries <query1,query2>`, `--timeout <seconds>`
  - **Default behavior**: Console output, 30s timeout, queries from fixtures
  - **Implementation**: Use argparse module

**Checkpoint**: All user stories should now be independently functional. Script produces complete diagnostic reports.

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T044 [P] Add comprehensive docstrings to all functions in `retrieval_test.py` following Google style
- [ ] T045 [P] Add type hints to all function signatures in `retrieval_test.py` for type safety
- [ ] T046 Optimize performance: implement connection pooling, batch operations, parallel execution per research.md
- [ ] T047 [P] Create `backend/embedding-pipeline/tests/conftest.py` with pytest fixtures (mock clients, sample data)
- [ ] T048 Run full test suite: `uv run pytest tests/ -v --cov=. --cov-report=html` and ensure >80% coverage
- [ ] T049 Validate script against quickstart.md: follow all setup/execution steps, confirm expected output
- [ ] T050 [P] Update `.env.example` with verification-specific config options (VERIFICATION_TOP_K, VERIFICATION_MIN_SCORE, VERIFICATION_TIMEOUT)
- [ ] T051 Test edge cases from spec.md: empty collection, invalid API keys, missing collection, dimension mismatch, missing metadata
- [ ] T052 Performance testing: run script with 100+ vectors, confirm <30s total runtime
- [ ] T053 [P] Add logging configuration: set up Python logging with INFO/ERROR levels, log to console and file
- [ ] T054 Final validation: run script end-to-end, verify JSON output matches contracts/diagnostic-report.json schema

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-5)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - Independent of US1 (but builds on validated data)
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - Independent of US1/US2 (but aggregates their results)

### Within Each User Story

- Tests MUST be written and FAIL before implementation (T011-T016, T025-T027, T034-T036)
- Dataclasses before validation functions (T004-T006 before T017-T023)
- Validation functions before orchestration (T017-T023 before T024)
- Search functions before search tests (T029-T030 before T032)
- All components before main pipeline (T042 depends on all previous)

### Parallel Opportunities

- **Setup tasks** (T001-T003): All can run in parallel
- **Foundational entities** (T004-T006): Can define dataclasses in parallel
- **Foundational clients** (T007-T008): Can initialize clients in parallel
- **US1 tests** (T011-T016): All unit tests can run in parallel
- **US1 validation functions** (T017-T019, T020-T023): P1 checks can run parallel, P2/P3 checks can run parallel
- **US2 tests** (T025-T026): Integration tests can run in parallel
- **US3 tests** (T034-T036): All unit tests can run in parallel
- **Polish tasks** (T044, T045, T047, T050, T053): Documentation, fixtures, config can run in parallel
- **Different user stories** can be worked on in parallel by different team members after Phase 2

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T010) - **CRITICAL**
3. Complete Phase 3: User Story 1 (T011-T024)
4. **STOP and VALIDATE**: Run `uv run python retrieval_test.py` and verify validation checks work
5. Commit and demo MVP: Data validation pipeline functional

### Incremental Delivery

1. Setup + Foundational (T001-T010) → Foundation ready ✅
2. Add User Story 1 (T011-T024) → Data validation works → **MVP deliverable** 🎯
3. Add User Story 2 (T025-T033) → Semantic search tests work → **Enhanced verification** 🔍
4. Add User Story 3 (T034-T043) → Diagnostic reports generated → **Full feature complete** 📊
5. Polish (T044-T054) → Production ready → **Ship it** 🚀

Each story adds value without breaking previous stories.

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together (T001-T010)
2. Once Foundational is done:
   - **Developer A**: User Story 1 (T011-T024) - Data validation
   - **Developer B**: User Story 2 (T025-T033) - Semantic search
   - **Developer C**: User Story 3 (T034-T043) - Diagnostic reports
3. Stories complete and integrate independently
4. Team reconvenes for Polish (T044-T054)

---

## Notes

- **[P] tasks** = different sections of file or different files, no dependencies, can run in parallel
- **[Story] label** maps task to specific user story for traceability (US1, US2, US3)
- Each user story should be independently completable and testable
- **Verify tests fail before implementing** - TDD approach
- Commit after each task or logical group of tasks
- Stop at any checkpoint to validate story independently
- **Single-file implementation**: All code in `retrieval_test.py` (~500-800 LOC total)
- **No new dependencies**: Reuses cohere, qdrant-client, pytest from embedding pipeline
- **Performance target**: Complete verification in <30s for ~50 vectors
- Follow code style from `backend/embedding-pipeline/main.py` for consistency

---

## Task Count Summary

- **Phase 1 (Setup)**: 3 tasks
- **Phase 2 (Foundational)**: 7 tasks
- **Phase 3 (US1 - P1)**: 14 tasks (6 tests + 8 implementation)
- **Phase 4 (US2 - P2)**: 9 tasks (3 tests + 6 implementation)
- **Phase 5 (US3 - P3)**: 10 tasks (3 tests + 7 implementation)
- **Phase 6 (Polish)**: 11 tasks
- **Total**: 54 tasks

**Estimated Time**: 4-6 hours for experienced Python developer following plan.md architecture
