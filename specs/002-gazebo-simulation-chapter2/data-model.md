# Data Model: Gazebo Simulation Chapter 2

**Date**: 2025-12-03
**Purpose**: Define data structures, component interfaces, and data flow for Chapter 2 implementation
**Status**: Completed

## Overview

Chapter 2 reuses all existing data models from Chapter 1 with minimal modifications. The multi-chapter architecture already supports independent chapters via `chapter_id` fields in database tables and vector stores.

---

## 1. Chapter Metadata Schema

### 1.1 Configuration File

**File**: `docs/chapters-config.json` (NEW)

**Purpose**: Centralized chapter metadata for scalable N-chapter architecture

**Schema**:

```json
{
  "$schema": "http://json-schema.org/draft-07/schema#",
  "type": "object",
  "properties": {
    "chapters": {
      "type": "array",
      "items": {
        "type": "object",
        "properties": {
          "id": {
            "type": "string",
            "description": "Unique chapter identifier (kebab-case)",
            "pattern": "^chapter-[0-9]+$",
            "examples": ["chapter-1", "chapter-2"]
          },
          "title": {
            "type": "string",
            "description": "Human-readable chapter title",
            "examples": ["ROS 2 Fundamentals", "Gazebo Simulation: Creating Digital Twins"]
          },
          "slug": {
            "type": "string",
            "description": "URL slug (matches id)",
            "pattern": "^chapter-[0-9]+$"
          },
          "order": {
            "type": "integer",
            "description": "Chapter sequence number (1-indexed)",
            "minimum": 1
          },
          "status": {
            "type": "string",
            "enum": ["draft", "published", "archived"],
            "description": "Publication status"
          },
          "sections": {
            "type": "array",
            "items": {"type": "string"},
            "description": "Ordered list of section slugs",
            "examples": [["intro", "what-is-gazebo", "urdf-sdf", "physics", "sensors"]]
          }
        },
        "required": ["id", "title", "slug", "order", "status", "sections"]
      }
    }
  },
  "required": ["chapters"]
}
```

**Concrete Example**:

```json
{
  "chapters": [
    {
      "id": "chapter-1",
      "title": "ROS 2 Fundamentals",
      "slug": "chapter-1",
      "order": 1,
      "status": "published",
      "sections": ["intro", "what-is-ros2", "why-ros2", "core-concepts", "examples", "exercises"]
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
- Frontend: ChapterSwitcher component reads config to render navigation
- Frontend: ProgressTracker maps `chapter_id` to title for display
- Backend: Validate `chapter_id` exists before API operations (optional validation)

---

## 2. Database Schema (REUSE - No Changes)

### 2.1 Existing Tables (Multi-Chapter Ready)

#### users

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    name VARCHAR(100),
    software_level VARCHAR(20) CHECK (software_level IN ('Beginner', 'Intermediate', 'Advanced')),
    hardware_level VARCHAR(20) CHECK (hardware_level IN ('Beginner', 'Intermediate', 'Advanced')),
    learning_goals TEXT,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Notes**:
- Global table (not chapter-specific)
- User profile applies to all chapters

#### progress_records (Multi-Chapter Support)

```sql
CREATE TABLE progress_records (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID REFERENCES users(id) ON DELETE CASCADE,
    chapter_id VARCHAR(50) NOT NULL,  -- 'chapter-1', 'chapter-2', etc.
    sections_completed TEXT[] DEFAULT '{}',  -- ['intro', 'what-is-gazebo', 'urdf-sdf']
    quiz_score INTEGER CHECK (quiz_score >= 0 AND quiz_score <= 100),
    completion_date TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    UNIQUE(user_id, chapter_id)  -- One record per user per chapter
);
```

**Notes**:
- `chapter_id` enables independent progress tracking per chapter
- `sections_completed` array holds completed section slugs
- UNIQUE constraint ensures one progress record per user per chapter

**Example Data**:

| user_id | chapter_id | sections_completed | quiz_score | completion_date | updated_at |
|---------|------------|-------------------|------------|-----------------|------------|
| uuid-1 | chapter-1 | ['intro', 'what-is-ros2', 'core-concepts', 'examples', 'exercises'] | 95 | 2025-12-02 10:30:00 | 2025-12-02 10:30:00 |
| uuid-1 | chapter-2 | ['intro', 'what-is-gazebo'] | NULL | NULL | 2025-12-03 14:15:00 |

#### chat_messages (Multi-Chapter Support)

```sql
CREATE TABLE chat_messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    session_id VARCHAR(100) NOT NULL,
    user_id UUID REFERENCES users(id) ON DELETE SET NULL,  -- Nullable for anonymous users
    chapter_id VARCHAR(50) NOT NULL,  -- 'chapter-1', 'chapter-2', etc.
    user_question TEXT NOT NULL,
    bot_response TEXT NOT NULL,
    context_used JSONB,  -- [{text, section, page, score}, ...]
    timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Notes**:
- `chapter_id` enables chapter-specific chatbot history
- `context_used` stores RAG retrieval results (which sections were used)

**Example Data**:

| id | session_id | user_id | chapter_id | user_question | bot_response | context_used | timestamp |
|----|------------|---------|------------|--------------|-------------|--------------|-----------|
| uuid-10 | sess-abc | uuid-1 | chapter-2 | "What's the difference between URDF and SDF?" | "URDF is ROS-specific..." | [{"text": "...", "section": "urdf-sdf", "score": 0.92}] | 2025-12-03 14:20:00 |

#### translation_cache

```sql
CREATE TABLE translation_cache (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    original_text_hash VARCHAR(64) UNIQUE NOT NULL,  -- SHA-256 hash
    original_text TEXT NOT NULL,
    translated_text TEXT NOT NULL,
    language VARCHAR(10) NOT NULL,  -- 'ur' for Urdu
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);
```

**Notes**:
- Global cache (not chapter-specific)
- Hash-based to avoid re-translating identical text across chapters

---

## 3. Vector Store Structure

### 3.1 Qdrant Collections

#### Collection: `chapter_2_embeddings` (NEW)

**Configuration**:

```python
{
    "collection_name": "chapter_2_embeddings",
    "vectors_config": {
        "size": 1536,  # OpenAI text-embedding-ada-002
        "distance": "Cosine"
    }
}
```

**Point Structure**:

```python
{
    "id": "<UUID>",  # Generated from content hash
    "vector": [0.123, -0.456, ...],  # 1536-dimensional embedding
    "payload": {
        "text": "Gazebo is a 3D robot simulator...",  # Original text chunk
        "chapter_id": "chapter-2",  # Chapter identifier
        "section": "intro",  # Section slug
        "page": "what-is-gazebo",  # Page/file name
        "chunk_id": 3  # Chunk number within page
    }
}
```

**Query Example**:

```python
from qdrant_client.models import Filter, FieldCondition, MatchValue

results = client.search(
    collection_name="chapter_2_embeddings",
    query_vector=query_embedding,
    limit=5,
    query_filter=Filter(
        must=[
            FieldCondition(
                key="chapter_id",
                match=MatchValue(value="chapter-2")
            )
        ]
    )
)
```

**Comparison with Chapter 1**:

| Property | Chapter 1 | Chapter 2 | Notes |
|----------|-----------|-----------|-------|
| Collection Name | `chapter_1_embeddings` | `chapter_2_embeddings` | Namespace by chapter |
| Vector Size | 1536 | 1536 | Same embedding model |
| Distance Metric | Cosine | Cosine | Same |
| `chapter_id` Payload | "chapter-1" | "chapter-2" | Different value |
| Chunk Strategy | Paragraph-based (~500 chars) | Paragraph-based (~500 chars) | Same |

---

## 4. Component Interfaces

### 4.1 React Component Props

All interactive components must accept `chapterId` prop to enable multi-chapter reuse.

#### PersonalizeButton

```typescript
interface PersonalizeButtonProps {
  chapterId: string;  // "chapter-1" | "chapter-2" | ...
  contentId: string;  // Section or page identifier for caching
  originalContent: string;  // Markdown content to personalize
  onPersonalized?: (personalizedContent: string) => void;  // Callback
}

// Example usage in Chapter 2
<PersonalizeButton
  chapterId="chapter-2"
  contentId="urdf-basics"
  originalContent={markdownContent}
  onPersonalized={(content) => setDisplayContent(content)}
/>
```

**API Call**:
```typescript
POST /api/personalize
{
  "chapter_id": "chapter-2",
  "content_id": "urdf-basics",
  "original_content": "...",
  "user_id": "uuid-1"
}

// Response
{
  "personalized_content": "...",
  "background_level": "software-intermediate",
  "adjustments_made": ["Added software analogies", "Code-focused examples"]
}
```

#### TranslateButton

```typescript
interface TranslateButtonProps {
  chapterId: string;  // "chapter-1" | "chapter-2" | ...
  contentId: string;  // Section or page identifier
  originalContent: string;  // Markdown content to translate
  targetLanguage: 'ur' | 'en';  // Urdu or English
  onTranslated?: (translatedContent: string) => void;  // Callback
}

// Example usage in Chapter 2
<TranslateButton
  chapterId="chapter-2"
  contentId="physics-engines"
  originalContent={markdownContent}
  targetLanguage="ur"
  onTranslated={(content) => setDisplayContent(content)}
/>
```

**API Call**:
```typescript
POST /api/translate
{
  "chapter_id": "chapter-2",
  "content_id": "physics-engines",
  "original_content": "...",
  "target_language": "ur"
}

// Response
{
  "translated_content": "...",
  "technical_terms_preserved": ["Gazebo", "URDF", "SDF", "ODE", "Bullet"],
  "cached": false  // true if retrieved from translation_cache
}
```

#### RAGChatbot

```typescript
interface RAGChatbotProps {
  chapterId: string;  // "chapter-1" | "chapter-2" | ...
  sessionId?: string;  // Optional session ID for history
  userId?: string;  // Optional user ID (null for anonymous)
}

// Example usage in Chapter 2
<RAGChatbot
  chapterId="chapter-2"
  sessionId={sessionId}
  userId={currentUser?.id}
/>
```

**API Call**:
```typescript
POST /api/chatbot/ask
{
  "chapter_id": "chapter-2",
  "question": "How do I add a LiDAR sensor to my robot?",
  "session_id": "sess-xyz",
  "user_id": "uuid-1"
}

// Response
{
  "answer": "To add a LiDAR sensor to your robot in Gazebo...",
  "context_used": [
    {
      "text": "LiDAR sensors in Gazebo are defined using the <sensor> tag...",
      "section": "sensors",
      "page": "lidar",
      "score": 0.89
    }
  ],
  "confidence": 0.89
}
```

#### ProgressTracker

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

// Example usage in Dashboard
<ProgressTracker
  userId={currentUser.id}
  chapters={chaptersConfig.chapters}
/>
```

**API Call**:
```typescript
GET /api/progress?user_id=uuid-1

// Response
{
  "user_id": "uuid-1",
  "chapters": [
    {
      "chapter_id": "chapter-1",
      "sections_completed": ["intro", "what-is-ros2", "core-concepts", "examples", "exercises"],
      "total_sections": 6,
      "completion_percentage": 100,
      "quiz_score": 95,
      "completion_date": "2025-12-02T10:30:00Z"
    },
    {
      "chapter_id": "chapter-2",
      "sections_completed": ["intro", "what-is-gazebo"],
      "total_sections": 7,
      "completion_percentage": 28.57,
      "quiz_score": null,
      "completion_date": null
    }
  ],
  "overall_progress": {
    "chapters_completed": 1,
    "total_chapters": 2,
    "global_completion_percentage": 64.29  # (1 * 100 + 2 * 28.57) / 2
  }
}
```

### 4.2 Interactive Visualization Components (NEW)

#### PhysicsParameterSlider

```typescript
interface PhysicsParameterSliderProps {
  parameter: 'gravity' | 'friction' | 'mass' | 'restitution';
  minValue: number;
  maxValue: number;
  defaultValue: number;
  unit: string;  // "m/s²", "kg", "coefficient"
  onValueChange?: (value: number) => void;
}

// Example usage
<PhysicsParameterSlider
  parameter="gravity"
  minValue={0}
  maxValue={20}
  defaultValue={9.81}
  unit="m/s²"
  onValueChange={(g) => updateForceVisualization(g)}
/>
```

**Implementation**: React component with HTML range input + SVG/Canvas visualization

#### SensorVisualization

```typescript
interface SensorVisualizationProps {
  sensorType: 'lidar' | 'camera' | 'imu';
  sampleDataFile: string;  // Path to JSON file with sample data
  width?: number;
  height?: number;
}

// Example usage
<SensorVisualization
  sensorType="lidar"
  sampleDataFile="/data/lidar-sample.json"
  width={600}
  height={400}
/>
```

**Sample Data Format** (lidar-sample.json):

```json
{
  "sensorType": "lidar",
  "dataPoints": [
    {"angle": 0, "distance": 5.2},
    {"angle": 15, "distance": 4.8},
    {"angle": 30, "distance": 3.9},
    ...
  ],
  "maxRange": 10.0,
  "units": "meters"
}
```

**Implementation**: Chart.js polar chart for LiDAR, HTML canvas for camera, Chart.js line chart for IMU

---

## 5. API Endpoints (REUSE - No New Endpoints)

All existing API endpoints are chapter-agnostic. Chapter 2 uses the same endpoints with `chapter_id="chapter-2"` parameter.

### 5.1 Personalization API

**Endpoint**: `POST /api/personalize`

**Request**:
```json
{
  "chapter_id": "chapter-2",
  "content_id": "urdf-basics",
  "original_content": "URDF (Unified Robot Description Format) is an XML format...",
  "user_id": "uuid-1"
}
```

**Response**:
```json
{
  "personalized_content": "URDF (Unified Robot Description Format) is an XML format similar to HTML/JSX in web development...",
  "background_level": "software-intermediate",
  "adjustments_made": ["Added software analogies"]
}
```

### 5.2 Translation API

**Endpoint**: `POST /api/translate`

**Request**:
```json
{
  "chapter_id": "chapter-2",
  "content_id": "physics-engines",
  "original_content": "Gazebo supports three physics engines: ODE, Bullet, and Simbody...",
  "target_language": "ur"
}
```

**Response**:
```json
{
  "translated_content": "Gazebo تین physics engines کو support کرتا ہے: ODE, Bullet, اور Simbody...",
  "technical_terms_preserved": ["Gazebo", "physics engines", "ODE", "Bullet", "Simbody"],
  "cached": false
}
```

### 5.3 RAG Chatbot API

**Endpoint**: `POST /api/chatbot/ask`

**Request**:
```json
{
  "chapter_id": "chapter-2",
  "question": "What's the difference between URDF and SDF?",
  "session_id": "sess-abc",
  "user_id": "uuid-1"
}
```

**Response**:
```json
{
  "answer": "URDF (Unified Robot Description Format) is specific to ROS and focuses on robot kinematics, while SDF (Simulation Description Format) is more general-purpose and includes additional simulation features like physics and sensor definitions...",
  "context_used": [
    {
      "text": "URDF is designed for ROS and describes robot structure...",
      "section": "urdf-sdf",
      "page": "urdf-vs-sdf",
      "score": 0.91
    }
  ],
  "confidence": 0.91
}
```

### 5.4 Progress API

**Endpoint**: `GET /api/progress?user_id=<uuid>`

**Response**: See ProgressTracker component example above

**Endpoint**: `POST /api/progress/update`

**Request**:
```json
{
  "user_id": "uuid-1",
  "chapter_id": "chapter-2",
  "section_completed": "urdf-sdf"
}
```

**Response**:
```json
{
  "success": true,
  "updated_progress": {
    "chapter_id": "chapter-2",
    "sections_completed": ["intro", "what-is-gazebo", "urdf-sdf"],
    "completion_percentage": 42.86
  }
}
```

---

## 6. Data Flow Diagrams

### 6.1 Personalization Flow

```
User clicks PersonalizeButton (Chapter 2)
  ↓
Frontend: POST /api/personalize
  {chapter_id: "chapter-2", content_id: "urdf-basics", user_id, original_content}
  ↓
Backend: Fetch user profile from users table
  ↓
Backend: Call PersonalizationEngine.adapt_content(content, software_level="intermediate")
  ↓
Backend: Use GPT-4 to adapt content with software analogies
  ↓
Backend: Return {personalized_content}
  ↓
Frontend: Update displayed content with personalized version
```

### 6.2 RAG Chatbot Flow

```
User asks question in RAGChatbot (Chapter 2)
  ↓
Frontend: POST /api/chatbot/ask
  {chapter_id: "chapter-2", question, session_id, user_id}
  ↓
Backend: Generate query embedding via OpenAI
  ↓
Backend: Search Qdrant collection "chapter_2_embeddings"
  Filter: chapter_id == "chapter-2"
  ↓
Backend: Retrieve top 5 relevant chunks
  ↓
Backend: Call GPT-4 with context: "{chunk1}\n{chunk2}\n...\n\nQuestion: {question}"
  ↓
Backend: Store question + response in chat_messages table
  ↓
Backend: Return {answer, context_used, confidence}
  ↓
Frontend: Display answer in chatbot UI
```

### 6.3 Progress Tracking Flow

```
User completes section in Chapter 2
  ↓
Frontend: POST /api/progress/update
  {user_id, chapter_id: "chapter-2", section_completed: "urdf-sdf"}
  ↓
Backend: Query progress_records WHERE user_id AND chapter_id = "chapter-2"
  ↓
Backend: IF record exists:
    Append section to sections_completed array
    UPDATE progress_records SET sections_completed, updated_at
  ELSE:
    INSERT new progress_records row
  ↓
Backend: Return updated progress
  ↓
Frontend: Update ProgressContext (React context)
  ↓
Frontend: Update navbar badge: "Progress: 1.5 / 2"
```

---

## 7. Dependencies

| Component | Dependency | Version | Notes |
|-----------|-----------|---------|-------|
| chapters-config.json | None | N/A | Static JSON file |
| progress_records table | PostgreSQL | 15+ | Existing schema |
| chapter_2_embeddings | Qdrant | 1.16.1+ | New collection |
| PersonalizeButton | OpenAI GPT-4 | 1.12.0+ | Existing API |
| TranslateButton | OpenAI GPT-4 | 1.12.0+ | Existing API |
| RAGChatbot | OpenAI Embeddings + GPT-3.5 | 1.12.0+ | Existing API |
| PhysicsParameterSlider | React + SVG | 18.2.0 | No external deps |
| SensorVisualization | Chart.js | **ADD** | Not yet installed |

**Action**: Add Chart.js to package.json:
```json
{
  "dependencies": {
    "chart.js": "^4.4.0",
    "react-chartjs-2": "^5.2.0"
  }
}
```

---

## 8. Summary

**What's Reused**:
- All database tables (users, progress_records, chat_messages, translation_cache)
- All API endpoints (/api/personalize, /api/translate, /api/chatbot/ask, /api/progress)
- All React component patterns (PersonalizeButton, TranslateButton, RAGChatbot accept `chapterId` prop)

**What's New**:
- `chapters-config.json` metadata file
- `chapter_2_embeddings` Qdrant collection
- Chapter 2 Markdown content files
- PhysicsParameterSlider component (static interactive diagrams)
- SensorVisualization component (Chart.js-based)
- Chart.js dependency

**No Schema Changes**: Database schema already supports multiple chapters via `chapter_id` fields.

**No API Changes**: Backend services already parameterized by `chapter_id`.

**Minimal Frontend Changes**: Add `chapterId="chapter-2"` prop to existing components.

This data model demonstrates the architectural excellence of Chapter 1's design - adding Chapter 2 requires almost zero data structure changes.
