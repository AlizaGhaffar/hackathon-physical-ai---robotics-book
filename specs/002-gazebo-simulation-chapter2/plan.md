# Implementation Plan: Gazebo Simulation Chapter 2

**Branch**: `002-gazebo-simulation-chapter2` | **Date**: 2025-12-03 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-gazebo-simulation-chapter2/spec.md`

## Summary

Add Chapter 2 "Gazebo Simulation: Creating Digital Twins" to the existing Physical AI textbook, reusing all Chapter 1 infrastructure (Docusaurus, React components, FastAPI backend, authentication, database schema) while adding new educational content about robot simulation. The implementation focuses on maximum component reuse per Constitution v1.1.0, requiring only: (1) new Markdown content in `docs/chapter-2/`, (2) sidebar navigation updates, (3) new Qdrant collection for RAG embeddings, and (4) embedding generation for Chapter 2 content. All existing bonus features (PersonalizeButton, TranslateButton, RAGChatbot) work without code changes by accepting `chapterId` props.

## Technical Context

**Language/Version**:
- Frontend: TypeScript 5.3.0, React 18.2.0, Docusaurus 3.0.0
- Backend: Python 3.11+

**Primary Dependencies**:
- Frontend: Docusaurus 3.0.0, React 18.2.0, Monaco Editor 4.6.0, React Three Fiber 8.15.0, Three.js 0.158.0
- Backend: FastAPI ≥0.109.0, OpenAI ≥1.12.0, Qdrant Client ≥1.16.1, psycopg[binary] ≥3.1.13, pydantic ≥2.6.0

**Storage**:
- Database: Neon Serverless Postgres (schema already multi-chapter ready with `chapter_id` fields)
- Vector Store: Qdrant Cloud Free Tier (will create new `chapter_2_embeddings` collection)
- Translation Cache: Postgres table (reusable across chapters)

**Testing**:
- Frontend: Manual testing (demo-ready priority per constitution)
- Backend: pytest (optional per constitution, only if time permits)
- RAG Validation: 10 sample questions accuracy testing (90% threshold per SC-004)

**Target Platform**:
- Deployment: GitHub Pages or Vercel
- Browsers: Chrome, Firefox, Safari (cross-browser testing required)
- Mobile: Responsive 320px-1920px (per FR-031)

**Project Type**: Web application (Docusaurus frontend + FastAPI backend)

**Performance Goals**:
- Page load: <3 seconds (SC-001)
- Personalization: <2 seconds (SC-002, FR-012)
- Translation: <5 seconds (SC-002, FR-016)
- RAG chatbot: <3 seconds (SC-002, FR-020)
- Chapter switching: <1 second (Constitution: Multi-Chapter Quality Gates)

**Constraints**:
- Must reuse all Chapter 1 components without modification (Constitution III: Component Reusability)
- Must match Chapter 1 visual design exactly (Constitution II: Consistency First)
- Must not break Chapter 1 functionality (Constitution I: Multi-Chapter Architecture)
- No implementation of interactive Gazebo simulations (Out of Scope per spec)
- No downloadable robot models (Out of Scope per spec)

**Scale/Scope**:
- Content: 4 major sections (Introduction, URDF/SDF, Physics, Sensors) matching Chapter 1's ~8 sections
- Embeddings: ~50-100 text chunks for RAG (similar to Chapter 1)
- Users: Existing user base (no new infrastructure needed)
- Chapters: 2 total (architecture supports N chapters per Constitution VI: Scalability)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### I. Multi-Chapter Architecture
✅ **PASS**: Implementation reuses all Chapter 1 architectural patterns
- Same Docusaurus structure (`docs/chapter-2/` mirrors `docs/chapter-1/`)
- Same React component library (no new components, only new `chapterId` prop values)
- Same FastAPI backend services (already parameterized by `chapter_id`)
- Same database schema (already supports multiple chapters via `chapter_id` field)

### II. Consistency First
✅ **PASS**: Chapter 2 achieves visual and functional parity
- Uses identical UI components (PersonalizeButton, TranslateButton, ChatBot)
- Follows same content structure (intro → concepts → examples → exercises)
- Applies global CSS classes from `src/css/custom.css`
- Maintains identical interaction patterns (button positions per existing layout)

### III. Component Reusability
✅ **PASS**: All components designed chapter-agnostic
- Components accept `chapterId` prop (e.g., `<RAGChatbot chapterId="chapter-2" />`)
- Backend APIs parameterized by `chapter_id` (no code changes)
- PersonalizeButton: Adapts any chapter content based on user profile
- TranslateButton: Translates any chapter text via GPT-4
- RAG chatbot: Filters embeddings by `chapter_id` metadata

### IV. Chapter Independence with Navigation
✅ **PASS**: Chapters are independently accessible
- Chapter 2 URL: `/docs/chapter-2/intro` (direct access, no prerequisites)
- Navigation menu: Update `sidebars.ts` to add Chapter 2 category
- Content: Self-contained (no references to Chapter 3-8)
- RAG scope: Chatbot uses only `chapter_2_embeddings` collection

### V. Multi-Chapter Progression
✅ **PASS**: Database already supports per-chapter tracking
- `progress_records` table has `chapter_id` VARCHAR(50) field
- Tracks `sections_completed` TEXT[] per chapter
- UNIQUE(user_id, chapter_id) constraint ensures independent tracking
- No schema migration needed

### VI. Scalability to N Chapters
✅ **PASS**: Architecture supports future chapters
- Chapter routing: Dynamic via Docusaurus categories (add category in `sidebars.ts`)
- Vector stores: Namespace pattern `chapter_{N}_embeddings`
- Database: Generic `chapter_id` field (not chapter-specific columns)
- Components: Prop-based `chapterId`, not hardcoded values

### VII. Bonus Features Continuity
✅ **PASS**: All bonus features work in Chapter 2
- Personalization: Same API endpoint `/api/personalize` (accepts `chapter_id` + content)
- Translation: Same API endpoint `/api/translate` (accepts any text)
- RAG Chatbot: Same component, different Qdrant collection (`chapter_2_embeddings`)
- Authentication: Global session, works across all chapters
- User Profiling: Same profile applies to all chapter content

### Constitutional Violations
**None** - All gates pass. No complexity justification needed.

## Project Structure

### Documentation (this feature)

```text
specs/002-gazebo-simulation-chapter2/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (interactive elements research)
├── data-model.md        # Phase 1 output (chapter metadata schema)
├── quickstart.md        # Phase 1 output (developer guide for adding Chapter 2)
├── contracts/           # Phase 1 output (no new APIs needed - document reuse)
│   └── chapter-api-reuse.md  # Documents how existing APIs work with Chapter 2
├── checklists/
│   └── requirements.md  # Spec quality checklist (completed, 16/16 pass)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

**Web Application Structure** (Option 2: Frontend + Backend)

```text
# FRONTEND (Docusaurus)
docs/
├── chapter-1/           # Existing Chapter 1 content (DO NOT MODIFY)
│   ├── intro.md
│   ├── what-is-ros2.md
│   ├── core-concepts/
│   ├── examples/
│   └── exercises/
└── chapter-2/           # NEW: Chapter 2 content (TO BE CREATED)
    ├── intro.md         # Landing page with learning objectives
    ├── what-is-gazebo.md  # Gazebo overview and ROS 2 integration
    ├── urdf-sdf/        # Robot description formats
    │   ├── urdf-basics.md
    │   ├── sdf-basics.md
    │   └── urdf-vs-sdf.md
    ├── physics/         # Physics simulation
    │   ├── physics-engines.md  # ODE, Bullet, Simbody
    │   ├── gravity-friction.md
    │   └── collisions.md
    ├── sensors/         # Sensor simulation
    │   ├── cameras.md
    │   ├── lidar.md
    │   ├── imu.md
    │   └── sensor-plugins.md
    ├── examples/        # Code examples
    │   ├── simple-robot.md      # Basic URDF example
    │   ├── robot-with-sensors.md
    │   └── physics-demo.md
    └── exercises/       # Hands-on exercises
        └── build-your-robot.md

src/
├── components/          # Existing components (REUSE, NO CHANGES)
│   ├── PersonalizeButton.tsx
│   ├── TranslateButton.tsx
│   ├── RAGChatbot.tsx
│   ├── QuizWidget.tsx
│   ├── CodeEditor.tsx
│   ├── RobotViewer.tsx  # May add Gazebo-specific props (research needed)
│   ├── ArchitectureDiagram.tsx
│   ├── ProgressTracker.tsx
│   └── auth/
│       ├── SignUpForm.tsx
│       └── SignInForm.tsx
├── css/
│   └── custom.css       # Global styles (NO CHANGES NEEDED)
└── pages/
    └── index.tsx        # Homepage (may add Chapter 2 link)

# CONFIGURATION FILES (UPDATED)
sidebars.ts              # MODIFY: Add Chapter 2 category
docusaurus.config.ts     # NO CHANGES (already multi-chapter ready)
package.json             # NO CHANGES (dependencies sufficient)

# BACKEND (FastAPI)
backend/
├── src/
│   ├── main.py          # NO CHANGES (routes already chapter-agnostic)
│   ├── config.py        # NO CHANGES (env vars sufficient)
│   ├── database.py      # NO CHANGES (schema already multi-chapter)
│   ├── vector_store.py  # MODIFY: Add create_chapter_2_collection()
│   ├── ai_client.py     # NO CHANGES (functions already generic)
│   ├── models/          # NO CHANGES (Pydantic models chapter-agnostic)
│   │   ├── user.py
│   │   ├── progress.py
│   │   └── chat.py
│   ├── routes/          # NO CHANGES (routes parameterized by chapter_id)
│   │   ├── auth.py
│   │   ├── personalization.py
│   │   ├── translation.py
│   │   └── chatbot.py
│   ├── services/        # NO CHANGES (services accept chapter_id param)
│   │   ├── personalization_engine.py
│   │   ├── translation_service.py
│   │   └── rag_service.py
│   └── scripts/         # MODIFY: Add chunk_chapter2_content.py
│       ├── chunk_content.py  # Existing (reusable)
│       └── embed_chapter2.py  # NEW: Generate Chapter 2 embeddings
├── requirements.txt     # NO CHANGES (dependencies sufficient)
└── .env                 # NO CHANGES (same API keys/secrets)

# DATABASE (Neon Postgres - NO SCHEMA CHANGES)
# Existing tables already support Chapter 2:
# - users (global)
# - progress_records (has chapter_id field)
# - chat_messages (has chapter_id field)
# - translation_cache (global, content-hash based)

# VECTOR STORE (Qdrant Cloud - NEW COLLECTION)
# Existing: chapter_1_embeddings
# NEW: chapter_2_embeddings (same config: 1536 dims, COSINE distance)
```

**Structure Decision**: Web application structure selected. Frontend uses Docusaurus monorepo pattern with chapter-based content directories (`docs/chapter-1/`, `docs/chapter-2/`). Backend remains unchanged as all services are already parameterized by `chapter_id`. The only new code required is: (1) Chapter 2 Markdown content files, (2) sidebar configuration update, (3) Qdrant collection creation script, and (4) embedding generation script.

## Complexity Tracking

**No violations** - Constitution Check passed all gates.

This implementation follows the principle of minimum viable change. Adding Chapter 2 requires almost zero new infrastructure due to Chapter 1's excellent architectural modularity.

---

## Phase 0: Research & Unknowns Resolution

### Research Questions

Based on user input requesting "NEW INTERACTIVE ELEMENTS" (Embedded Gazebo WebGL viewer, URDF model editor, Physics parameter sliders, Sensor visualization), we need to research feasibility and align with constitution's constraints.

#### R1: Embedded Gazebo WebGL Viewer Feasibility

**Question**: Can we embed a Gazebo simulation viewer in the browser, or does this violate "Out of Scope: Creating interactive Gazebo simulations embedded in the browser" (spec.md)?

**Research Tasks**:
1. Review spec Out of Scope section for exact wording
2. Determine if "viewer" (read-only visualization) differs from "simulation" (interactive physics)
3. Investigate Gazebo WebGL/WASM options (gzweb, Gazebo Web, gz-sim WASM builds)
4. Estimate implementation effort vs hackathon timeline constraints

**Decision Criteria**: If implementation exceeds 1 day of effort OR conflicts with spec, defer to post-MVP enhancement.

#### R2: URDF Model Editor Implementation

**Question**: Should we build a custom URDF editor component, or use existing solutions?

**Research Tasks**:
1. Search for open-source React URDF editors (npm, GitHub)
2. Evaluate Monaco Editor XML mode as simpler alternative
3. Check if Three.js RobotViewer can provide live URDF preview
4. Assess if this adds unique educational value or is feature creep

**Decision Criteria**: Use existing tools (Monaco + RobotViewer) unless custom editor provides substantial learning benefit and takes <2 days to implement.

#### R3: Physics Parameter Sliders & Sensor Visualization

**Question**: Do interactive physics sliders require live Gazebo simulation backend, or can we demonstrate with static visualizations?

**Research Tasks**:
1. Define scope: Static diagrams with sliders (simple) vs live simulation (complex)
2. Investigate Chart.js/D3.js for sensor data visualization (LiDAR point clouds, camera feeds)
3. Determine if user input requested live interactivity or pedagogical illustrations

**Decision Criteria**: Implement static interactive diagrams (React state + SVG/Canvas). Live simulation deferred as out of scope.

#### R4: Chapter Navigation Menu UX

**Question**: What's the optimal UI pattern for chapter navigation that scales to 8 chapters?

**Research Tasks**:
1. Review Docusaurus sidebar best practices for multi-section docs
2. Design chapter switcher component (dropdown? sidebar? breadcrumb?)
3. Ensure mobile-friendly navigation (per FR-031)
4. Reference Chapter 1 implementation for consistency

**Decision Criteria**: Use Docusaurus built-in sidebar categories. Add custom ChapterSwitcher component only if sidebar insufficient.

#### R5: Progress Tracking UI Location

**Question**: Where should per-chapter progress be displayed? (Sidebar? Dashboard? Both?)

**Research Tasks**:
1. Design progress indicator for sidebar (checkmarks per chapter)
2. Design dashboard view showing all chapters (ProgressTracker component)
3. Ensure progress updates trigger UI refresh (React state management)

**Decision Criteria**: Sidebar shows simple checkmarks, dedicated dashboard page shows detailed progress (implement dashboard as P4 task).

---

## Phase 1: Design & Contracts

### Deliverables

1. **data-model.md**: Chapter metadata schema, progress tracking updates
2. **contracts/chapter-api-reuse.md**: Document how existing APIs work with Chapter 2 (no new endpoints)
3. **quickstart.md**: Developer guide for adding Chapter 2 content and embeddings

### Design Tasks

#### D1: Chapter Metadata Schema

Define chapter configuration structure for scalability to N chapters:

**File**: `docs/chapters-config.json` (NEW)

```json
{
  "chapters": [
    {
      "id": "chapter-1",
      "title": "ROS 2 Fundamentals",
      "slug": "chapter-1",
      "order": 1,
      "status": "published",
      "sections": ["intro", "what-is-ros2", "core-concepts", "examples", "exercises"]
    },
    {
      "id": "chapter-2",
      "title": "Gazebo Simulation: Creating Digital Twins",
      "slug": "chapter-2",
      "order": 2,
      "status": "published",
      "sections": ["intro", "what-is-gazebo", "urdf-sdf", "physics", "sensors", "examples", "exercises"]
    }
  ]
}
```

**Usage**:
- ChapterSwitcher component reads config to render navigation
- ProgressTracker component maps `chapter_id` to title
- Backend validates `chapter_id` exists before operations

#### D2: Sidebar Navigation Update

**File**: `sidebars.ts` (MODIFY)

Add new category for Chapter 2:

```typescript
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Chapter 1: ROS 2 Fundamentals',
      items: [
        'chapter-1/intro',
        'chapter-1/what-is-ros2',
        // ... existing Chapter 1 items
      ],
    },
    {
      type: 'category',
      label: 'Chapter 2: Gazebo Simulation',
      items: [
        'chapter-2/intro',
        'chapter-2/what-is-gazebo',
        {
          type: 'category',
          label: 'URDF & SDF',
          items: [
            'chapter-2/urdf-sdf/urdf-basics',
            'chapter-2/urdf-sdf/sdf-basics',
            'chapter-2/urdf-sdf/urdf-vs-sdf',
          ],
        },
        // ... nested categories for physics, sensors, examples, exercises
      ],
    },
  ],
};
```

#### D3: Qdrant Collection for Chapter 2

**File**: `backend/src/vector_store.py` (MODIFY)

Add function to create Chapter 2 collection:

```python
def create_chapter_2_collection():
    """Create Qdrant collection for Chapter 2 embeddings."""
    client = get_qdrant_client()

    collection_name = "chapter_2_embeddings"

    # Check if collection exists
    collections = client.get_collections().collections
    if any(c.name == collection_name for c in collections):
        print(f"Collection {collection_name} already exists")
        return

    # Create collection with same config as Chapter 1
    client.create_collection(
        collection_name=collection_name,
        vectors_config=VectorParams(size=1536, distance=Distance.COSINE)
    )
    print(f"Created collection: {collection_name}")
```

**Script**: `backend/src/scripts/embed_chapter2.py` (NEW)

Generate embeddings for Chapter 2 content:

```python
import sys
sys.path.append('../')

from pathlib import Path
from ai_client import generate_embedding
from vector_store import get_qdrant_client
from qdrant_client.models import PointStruct
import uuid
import hashlib

def chunk_markdown_file(file_path: Path, chunk_size: int = 500) -> list[dict]:
    """Split markdown file into chunks with metadata."""
    content = file_path.read_text(encoding='utf-8')

    # Simple chunking by paragraphs (improve with semantic chunking if needed)
    paragraphs = content.split('\n\n')
    chunks = []

    for i, para in enumerate(paragraphs):
        if len(para.strip()) < 50:  # Skip very short paragraphs
            continue

        chunks.append({
            'text': para.strip(),
            'section': file_path.parent.name,
            'page': file_path.stem,
            'chunk_id': i,
            'chapter_id': 'chapter-2'
        })

    return chunks

def embed_chapter_2_content():
    """Generate and upload embeddings for all Chapter 2 content."""
    docs_path = Path('../../../docs/chapter-2')

    if not docs_path.exists():
        raise FileNotFoundError(f"Chapter 2 content not found at {docs_path}")

    # Collect all .md files
    md_files = list(docs_path.rglob('*.md'))
    print(f"Found {len(md_files)} markdown files in Chapter 2")

    client = get_qdrant_client()
    collection_name = "chapter_2_embeddings"

    all_points = []

    for md_file in md_files:
        print(f"Processing {md_file.name}...")
        chunks = chunk_markdown_file(md_file)

        for chunk in chunks:
            # Generate embedding
            embedding = generate_embedding(chunk['text'])

            # Create unique ID from content hash
            content_hash = hashlib.sha256(chunk['text'].encode()).hexdigest()[:16]
            point_id = str(uuid.UUID(content_hash + '0' * 16))

            # Create point with metadata
            point = PointStruct(
                id=point_id,
                vector=embedding,
                payload={
                    'text': chunk['text'],
                    'chapter_id': chunk['chapter_id'],
                    'section': chunk['section'],
                    'page': chunk['page'],
                    'chunk_id': chunk['chunk_id']
                }
            )

            all_points.append(point)

    # Upload in batches
    batch_size = 100
    for i in range(0, len(all_points), batch_size):
        batch = all_points[i:i+batch_size]
        client.upsert(collection_name=collection_name, points=batch)
        print(f"Uploaded batch {i//batch_size + 1}/{len(all_points)//batch_size + 1}")

    print(f"✅ Successfully embedded {len(all_points)} chunks for Chapter 2")

if __name__ == "__main__":
    embed_chapter_2_content()
```

#### D4: RAG Service Chapter Filtering

**File**: `backend/src/services/rag_service.py` (VERIFY NO CHANGES NEEDED)

Existing RAG service should already filter by `chapter_id`:

```python
def search_chapter_content(query: str, chapter_id: str, top_k: int = 5) -> list[dict]:
    """Search for relevant content within a specific chapter."""
    client = get_qdrant_client()

    # Determine collection name from chapter_id
    collection_name = f"{chapter_id.replace('-', '_')}_embeddings"

    # Generate query embedding
    query_embedding = generate_embedding(query)

    # Search with chapter_id filter
    results = client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=top_k,
        query_filter=Filter(
            must=[
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_id)
                )
            ]
        )
    )

    return [
        {
            'text': hit.payload['text'],
            'section': hit.payload['section'],
            'page': hit.payload['page'],
            'score': hit.score
        }
        for hit in results
    ]
```

**Validation**: Test with `chapter_id="chapter-2"` after embeddings uploaded.

#### D5: Component Props Validation

Verify all Chapter 1 components accept `chapterId` prop (review existing implementations during Phase 1 or mark for implementation):

```typescript
// Example: RAGChatbot.tsx
interface RAGChatbotProps {
  chapterId: string;  // "chapter-1" or "chapter-2"
}

export function RAGChatbot({ chapterId }: RAGChatbotProps) {
  const handleAskQuestion = async (question: string) => {
    const response = await fetch('/api/chatbot/ask', {
      method: 'POST',
      body: JSON.stringify({ question, chapter_id: chapterId }),
      headers: { 'Content-Type': 'application/json' }
    });
    // ... handle response
  };

  // ... rest of component
}
```

**Action**: If components NOT yet implemented, ensure design docs specify `chapterId` prop requirement.

---

## Phase 2: Implementation Plan Summary

**Note**: Detailed tasks will be generated by `/sp.tasks` command. This section provides high-level implementation phases.

### Phase 2.1: Content Creation (Priority: P2 - Core Content)

1. Write Chapter 2 Markdown files in `docs/chapter-2/`
   - Introduction to Gazebo (what-is-gazebo.md)
   - URDF vs SDF formats (urdf-sdf/*.md)
   - Physics simulation (physics/*.md)
   - Sensor simulation (sensors/*.md)
   - Code examples (examples/*.md)
   - Exercises (exercises/*.md)

2. Ensure content matches Chapter 1 style:
   - Learning objectives at start of each section
   - Code examples with syntax highlighting
   - Clear section headings
   - Consistent tone and complexity level

### Phase 2.2: Navigation Integration (Priority: P1 - Foundation)

1. Update `sidebars.ts` to add Chapter 2 category
2. Create `docs/chapters-config.json` metadata file
3. Test navigation: Homepage → Chapter 1 → Chapter 2 → Homepage
4. Verify direct URL access works (`/docs/chapter-2/intro`)

### Phase 2.3: Vector Embeddings (Priority: P2 - RAG Chatbot)

1. Run `create_chapter_2_collection()` in vector_store.py
2. Run `embed_chapter2.py` script to generate embeddings
3. Verify embeddings uploaded to Qdrant dashboard
4. Test RAG search with sample Chapter 2 questions

### Phase 2.4: Component Integration (Priority: P1 - Bonus Features)

1. Add `<PersonalizeButton chapterId="chapter-2" />` to Chapter 2 pages
2. Add `<TranslateButton chapterId="chapter-2" />` to Chapter 2 pages
3. Add `<RAGChatbot chapterId="chapter-2" />` to Chapter 2 pages
4. Test all bonus features work in Chapter 2

### Phase 2.5: Progress Tracking (Priority: P4 - Enhancement)

1. Verify database records progress for `chapter_id="chapter-2"`
2. Add progress indicators to sidebar (checkmarks per chapter)
3. Create dashboard page showing multi-chapter progress
4. Test progress persists across logout/login

### Phase 2.6: Testing & Validation (Priority: P1 - Quality Gates)

1. Visual consistency check (Chapter 1 vs Chapter 2 side-by-side)
2. RAG accuracy test (10 Chapter 2 questions, target 90% accuracy)
3. Cross-browser testing (Chrome, Firefox, Safari)
4. Mobile responsive testing (320px-1920px)
5. Performance testing (page load <3s, chatbot <3s)

---

## Next Steps

1. **Complete Phase 0 Research**: Generate `research.md` with decisions on interactive elements (R1-R5)
2. **Complete Phase 1 Design**: Generate `data-model.md`, `contracts/chapter-api-reuse.md`, `quickstart.md`
3. **Run `/sp.tasks`**: Generate detailed task list with dependencies and parallel execution opportunities
4. **Implement Chapter 2**: Follow task list, starting with P1 (navigation) and P2 (content) tasks

---

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Chapter 1 components not yet implemented | HIGH | HIGH | Phase 0 research will verify component status. If not implemented, Chapter 2 blocks on Chapter 1 completion. |
| Interactive elements (Gazebo viewer) scope creep | MEDIUM | MEDIUM | Phase 0 research will define strict scope. Defer complex features to post-MVP. |
| RAG accuracy <90% for Chapter 2 | LOW | MEDIUM | Use proven Chapter 1 chunking strategy. Test early with 10 questions, iterate on chunks if needed. |
| Visual inconsistency between chapters | LOW | HIGH | Use global CSS classes, no chapter-specific styles. Manual visual comparison testing. |
| Performance degradation with 2 chapters | LOW | LOW | Static site generation (Docusaurus SSG) scales well. Qdrant handles multiple collections efficiently. |

---

## Open Questions (Defer to Phase 0 Research)

1. **Interactive Elements**: Should we implement Gazebo WebGL viewer, URDF editor, physics sliders, or defer to static diagrams?
2. **Component Status**: Are Chapter 1 components (PersonalizeButton, TranslateButton, RAGChatbot) fully implemented or still in progress?
3. **Navigation UX**: Sidebar-only or add custom ChapterSwitcher component?
4. **Progress Dashboard**: Separate page or embedded in sidebar?
5. **Embedding Strategy**: Use same chunking size (500 chars) as Chapter 1, or adjust for Gazebo content?

**Action**: Phase 0 research.md will resolve these questions before Phase 1 design.
