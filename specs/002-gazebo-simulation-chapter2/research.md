# Research: Gazebo Simulation Chapter 2

**Date**: 2025-12-03
**Purpose**: Resolve open questions and unknowns before Phase 1 design
**Status**: Completed

## Research Questions & Decisions

### R1: Embedded Gazebo WebGL Viewer Feasibility

**Question**: Can we embed a Gazebo simulation viewer in the browser, or does this violate "Out of Scope: Creating interactive Gazebo simulations embedded in the browser"?

**Research Findings**:

**Specification Review**:
- spec.md Out of Scope states: "Creating interactive Gazebo simulations embedded in the browser (content focuses on explaining concepts, not running live simulations)"
- Key phrase: "interactive simulations" and "running live simulations"
- Distinction: Static visualizations vs live physics simulations

**Technical Options**:
1. **gzweb** (Gazebo Web): Requires Gazebo server backend, WebSocket connection, live sim
ulation rendering - **Complex, out of scope**
2. **Three.js Static Models**: Display pre-rendered URDF/SDF models without physics - **Feasible**
3. **Static Screenshots/Videos**: Show Gazebo interface and simulations via media - **Simplest**

**Implementation Effort**:
- gzweb: 3-5 days (server setup, WebSocket integration, frontend client)
- Three.js viewer: 1-2 days (if RobotViewer component already exists for Chapter 1)
- Static media: <1 day (capture screenshots/videos, embed in Markdown)

**Constitution Alignment**:
- Principle VI (Maximum Points Optimization): "Trade-offs favoring bonus features over content depth are acceptable"
- Live Gazebo simulation adds content depth but NOT a bonus feature
- Static visualizations sufficient for educational value

**DECISION**: ❌ **Defer live Gazebo WebGL viewer to post-MVP**

**Rationale**:
1. Violates spec Out of Scope (interactive simulations)
2. High implementation effort (3-5 days) vs limited educational ROI
3. Static screenshots/videos + existing RobotViewer component for URDF models provides 80% of value at 10% of effort
4. Focus remains on bonus features (personalization, translation, RAG) per constitution

**Alternatives Approved**:
- ✅ Static Gazebo screenshots in Markdown (show UI, world setup, robot spawning)
- ✅ Embedded YouTube videos of Gazebo simulations (if available or easy to create)
- ✅ Use existing RobotViewer component (Three.js) to display URDF models (no physics, just 3D visualization)

---

### R2: URDF Model Editor Implementation

**Question**: Should we build a custom URDF editor component, or use existing solutions?

**Research Findings**:

**Existing Solutions**:
1. **urdf-js**: React component for URDF visualization (view-only, no editing) - Used in RobotViewer
2. **Monaco Editor XML mode**: Syntax-highlighted XML editor (general-purpose) - Already in dependencies
3. **ros-urdf-editor** (GitHub): Standalone tool, not React component, requires porting

**Custom Editor Requirements**:
- Syntax validation (XML schema)
- Live preview (Three.js integration)
- Auto-completion (URDF tags: `<robot>`, `<link>`, `<joint>`)
- Error highlighting

**Implementation Effort**:
- Custom editor with live preview: 2-3 days
- Monaco Editor XML mode only: <1 hour (already exists)
- Monaco + manual RobotViewer refresh: <2 hours

**Educational Value Assessment**:
- **Primary goal**: Teach URDF syntax and structure
- **Learners need**: Understand tags, attributes, relationships
- **Live editing value**: High (immediate feedback), but not critical for comprehension

**DECISION**: ✅ **Use Monaco Editor XML mode + RobotViewer (no live sync)**

**Rationale**:
1. Monaco Editor already exists in Chapter 1 dependencies (4.6.0)
2. XML mode provides syntax highlighting and basic validation
3. Learners can copy code examples into Monaco, then manually load in RobotViewer
4. This matches hackathon priority: functional over perfect UX
5. Live sync is a "nice-to-have", not a "must-have" for learning

**Implementation Approach**:
- Use existing `<CodeEditor language="xml" />` component
- Provide downloadable URDF files (click to download → open in RobotViewer)
- Add "Copy to Clipboard" button for URDF examples
- Use RobotViewer component to display pre-built URDF examples

**Post-MVP Enhancement**: Add live sync between Monaco and RobotViewer (WebSocket or React state)

---

### R3: Physics Parameter Sliders & Sensor Visualization

**Question**: Do interactive physics sliders require live Gazebo simulation backend, or can we demonstrate with static visualizations?

**Research Findings**:

**User Input Analysis**:
- User requested: "Physics parameter sliders" and "Sensor visualization"
- Context: Educational textbook, not interactive playground
- Goal: Demonstrate concepts, not provide simulation sandbox

**Implementation Options**:

**Physics Sliders**:
1. **Live simulation**: Adjust gravity → see robot fall faster (requires Gazebo backend) - **Out of scope**
2. **Static interactive diagrams**: Slider changes SVG/Canvas visualization (client-side React) - **Feasible**
3. **Before/After comparisons**: Show two static images (low gravity vs high gravity) - **Simplest**

**Sensor Visualization**:
1. **Live sensor feeds**: Stream LiDAR point clouds, camera images from Gazebo - **Out of scope**
2. **Static visualizations**: Chart.js/D3.js to display sample sensor data (JSON → charts) - **Feasible**
3. **Static screenshots**: Show Gazebo sensor outputs (RViz visualizations) - **Simplest**

**Implementation Effort**:
- Live simulation: 5+ days (Gazebo backend API, WebSocket, frontend rendering)
- Static interactive diagrams: 1-2 days (React components, SVG/Canvas, Chart.js)
- Static screenshots: <1 day

**DECISION**: ✅ **Implement static interactive diagrams (React + SVG/Canvas + Chart.js)**

**Rationale**:
1. Provides interactivity without Gazebo backend complexity
2. Demonstrates concepts effectively (e.g., "Move gravity slider → see force vector change on diagram")
3. Manageable implementation time (1-2 days)
4. Reusable component for future chapters (generic PhysicsSlider, SensorChart components)

**Implementation Approach**:

**Physics Sliders**:
- Create `PhysicsParameterSlider` component:
  - Input: Slider for gravity (0-20 m/s²)
  - Output: SVG diagram showing force arrows on robot
  - Calculation: F = m × g (update force magnitude in real-time)
- Parameters to demonstrate: Gravity, Friction, Mass, Restitution (bounciness)

**Sensor Visualization**:
- Create `SensorVisualization` component:
  - **LiDAR**: D3.js polar chart (angle vs distance) with sample point cloud data
  - **Camera**: Display sample image with bounding boxes (object detection visualization)
  - **IMU**: Line charts (Chart.js) showing acceleration/gyroscope data over time
- Use sample JSON data files (no live Gazebo connection)

**Dependencies**: Chart.js (add to package.json if not present), D3.js (optional, can use Chart.js only)

**Post-MVP Enhancement**: Connect to live Gazebo backend via WebSocket (if backend infrastructure built later)

---

### R4: Chapter Navigation Menu UX

**Question**: What's the optimal UI pattern for chapter navigation that scales to 8 chapters?

**Research Findings**:

**Docusaurus Sidebar Capabilities**:
- Built-in category support (collapsible sections)
- Auto-generated navigation from file structure
- Mobile-responsive sidebar (hamburger menu)
- Breadcrumb navigation (built-in)
- Version dropdown (can repurpose for chapter switcher)

**Existing Chapter 1 Implementation**:
- sidebars.ts defines `tutorialSidebar` with single category
- Category label: "Chapter 1: ROS 2 Fundamentals"
- Items: Flat list + nested subcategories

**Scalability to 8 Chapters**:
- Option 1: **8 top-level categories** (Chapter 1, Chapter 2, ..., Chapter 8) - **Simple, scales well**
- Option 2: **Custom ChapterSwitcher dropdown** - **Custom development needed**
- Option 3: **Multi-docusaurus instances** (separate docs per chapter) - **Overly complex**

**Mobile Responsiveness**:
- Docusaurus sidebar collapses to hamburger menu on mobile (<768px)
- Categories are collapsible (click to expand/collapse)
- No custom mobile optimization needed

**DECISION**: ✅ **Use Docusaurus built-in sidebar categories (Option 1)**

**Rationale**:
1. Zero custom development required
2. Mobile-responsive out of the box
3. Proven pattern in multi-section documentation sites
4. Scales to 8+ chapters without UI clutter (categories are collapsible)
5. Aligns with Constitution III (Component Reusability): Reuse built-in Docusaurus features

**Implementation**:
- Update `sidebars.ts`:
  ```typescript
  const sidebars = {
    tutorialSidebar: [
      {
        type: 'category',
        label: 'Chapter 1: ROS 2 Fundamentals',
        collapsed: true,  // Default collapsed after completion
        items: [/* Chapter 1 items */],
      },
      {
        type: 'category',
        label: 'Chapter 2: Gazebo Simulation',
        collapsed: false,  // Expanded for active chapter
        items: [/* Chapter 2 items */],
      },
      // Future: Chapter 3, 4, 5, ...
    ],
  };
  ```

**Additional Navigation Elements**:
- ✅ **Breadcrumbs**: Already provided by Docusaurus (Home > Chapter 2 > URDF Basics)
- ✅ **Prev/Next buttons**: Already provided by Docusaurus (bottom of each page)
- ❌ **Custom ChapterSwitcher**: Not needed (sidebar sufficient)

**Post-MVP Enhancement**: Add chapter progress indicators (checkmarks) next to category labels in sidebar (requires custom Docusaurus plugin or CSS overlay)

---

### R5: Progress Tracking UI Location

**Question**: Where should per-chapter progress be displayed? (Sidebar? Dashboard? Both?)

**Research Findings**:

**User Story 4 Requirements** (Priority: P4):
- "View profile or dashboard" → implies dedicated dashboard page
- "Per-chapter progress displayed in chapter navigation menu" → implies sidebar indicators

**UI Patterns for Progress Tracking**:
1. **Sidebar checkmarks**: Simple icons next to chapter/section labels (✓ = complete, ○ = incomplete)
2. **Dashboard page**: Dedicated route (`/dashboard` or `/profile`) with detailed progress UI
3. **Modal overlay**: Click "View Progress" → modal shows detailed stats
4. **Header badge**: Global progress (e.g., "1.5 / 2 chapters") in navbar

**Implementation Complexity**:
- Sidebar checkmarks: Medium (requires custom Docusaurus sidebar plugin or CSS hack)
- Dashboard page: Low (standard React component, already have ProgressTracker component planned)
- Modal overlay: Low (React Portal + modal component)
- Header badge: Low (Docusaurus navbar customization)

**Docusaurus Limitations**:
- Sidebar customization requires swizzling Docusaurus components (complex)
- Navbar customization is easier (docusaurus.config.ts)

**DECISION**: ✅ **Dashboard page (primary) + Navbar badge (secondary)**

**Rationale**:
1. Dashboard page provides detailed progress (per-section, timestamps, quiz scores)
2. Navbar badge provides at-a-glance global progress
3. Sidebar checkmarks deferred to post-MVP (too complex for hackathon timeline)
4. Aligns with User Story 4 acceptance scenario: "view their profile or dashboard"

**Implementation**:

**Dashboard Page** (`src/pages/dashboard.tsx`):
- Route: `/dashboard`
- Components:
  - `<ProgressTracker />` - Chapter completion cards
  - `<ProfileSummary />` - User name, background levels
  - `<RecentActivity />` - Last accessed chapter/section
- Data source: Fetch from `/api/progress` (returns all `progress_records` for user)

**Navbar Badge** (docusaurus.config.ts):
- Custom navbar item: "Progress: 1.5 / 2"
- On click: Redirect to `/dashboard`
- Update badge via React context (ProgressContext) on section completion

**Sidebar Indicators** (deferred to post-MVP):
- Requires Docusaurus sidebar plugin or component swizzling
- Complexity exceeds hackathon timeline

**Post-MVP Enhancement**: Add per-section checkmarks in sidebar using Docusaurus plugin

---

## Additional Research: Component Status Verification

**Question**: Are Chapter 1 components (PersonalizeButton, TranslateButton, RAGChatbot) fully implemented or still in progress?

**Findings from Architecture Exploration**:
- `src/components/` directory exists but is currently **empty**
- Components are **planned** but **not yet implemented** (per `PROGRESS-SUMMARY.md`)
- Implementation status: "⏳ Pending"

**Impact on Chapter 2**:
- **HIGH RISK**: Chapter 2 cannot demonstrate bonus features without Chapter 1 components
- **Blocking dependency**: Chapter 2 implementation blocks on Chapter 1 component completion

**DECISION**: ✅ **Document component interface requirements in Phase 1**

**Rationale**:
1. Chapter 2 planning can proceed with interface definitions
2. Component implementation is a Chapter 1 task, not a Chapter 2 task
3. Chapter 2 tasks will include "Integrate components" (depends on Ch1 completion)

**Action Items for Phase 1**:
- Document required component props in `data-model.md`
- Specify API contracts for bonus features in `contracts/chapter-api-reuse.md`
- Flag component implementation as a prerequisite in `quickstart.md`

---

## Summary of Decisions

| Question | Decision | Rationale |
|----------|----------|-----------|
| R1: Gazebo WebGL Viewer | ❌ Defer to post-MVP | Out of scope per spec, high effort, low ROI |
| R2: URDF Editor | ✅ Monaco XML + RobotViewer | Reuse existing tools, adequate for learning |
| R3: Physics Sliders | ✅ Static interactive diagrams | Client-side React + SVG, no backend needed |
| R4: Chapter Navigation | ✅ Docusaurus sidebar categories | Zero custom dev, scales to N chapters |
| R5: Progress Tracking | ✅ Dashboard page + Navbar badge | Clean UX, manageable complexity |
| Component Status | ⚠️ Not yet implemented | Document interfaces, flag as prerequisite |

---

## Technical Dependencies Validated

| Dependency | Status | Notes |
|------------|--------|-------|
| Docusaurus 3.0.0 | ✅ Installed | Sufficient for Chapter 2, no upgrades needed |
| Monaco Editor 4.6.0 | ✅ Installed | XML mode sufficient for URDF editing |
| React Three Fiber 8.15.0 | ✅ Installed | RobotViewer can display URDF models |
| Chart.js | ❌ Not installed | **ADD**: For sensor data visualization |
| FastAPI backend | ✅ Exists | Already chapter-agnostic, no changes needed |
| Qdrant | ✅ Configured | Create new collection for Chapter 2 |
| Neon Postgres | ✅ Configured | Schema already multi-chapter ready |

**Action**: Add Chart.js to `package.json` during implementation phase.

---

## Risks Identified & Mitigation

| Risk | Severity | Mitigation |
|------|----------|------------|
| Chapter 1 components not implemented | **HIGH** | Document interfaces now; Chapter 2 blocks on Ch1 completion |
| Physics/Sensor diagrams take >2 days | **MEDIUM** | Start with simplest visualizations (SVG arrows); defer complex charts if time runs out |
| Chart.js learning curve | **LOW** | Chart.js has excellent docs; fallback to simple HTML tables if needed |
| Qdrant collection creation fails | **LOW** | Collection creation script is simple; test early in implementation |

---

## Next Steps (Phase 1)

1. Generate `data-model.md`:
   - Chapter metadata schema
   - Component prop interfaces
   - Progress tracking data flow
   - Vector store structure

2. Generate `contracts/chapter-api-reuse.md`:
   - Document existing API endpoints
   - Show how Chapter 2 reuses them with `chapter_id` parameter
   - No new endpoints needed

3. Generate `quickstart.md`:
   - Developer guide for adding Chapter 2
   - Prerequisites (Chapter 1 component completion)
   - Step-by-step: Content → Sidebar → Embeddings → Components
   - Testing checklist

**Phase 0 Complete** ✅
