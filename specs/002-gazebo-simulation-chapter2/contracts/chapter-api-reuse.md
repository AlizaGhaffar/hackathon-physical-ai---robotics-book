# API Contracts: Chapter 2 Reuse Documentation

**Date**: 2025-12-03
**Purpose**: Document how Chapter 2 reuses existing Chapter 1 API endpoints without modifications
**Status**: Completed

## Overview

Chapter 2 requires **ZERO new API endpoints**. All existing backend services are already chapter-agnostic through the `chapter_id` parameter design. This document provides examples of how each API is used with `chapter_id="chapter-2"`.

---

## 1. Authentication API (REUSE - Global)

### POST /api/auth/signup

**Purpose**: Create new user account with background questionnaire

**Request**:
```json
{
  "email": "user@example.com",
  "password": "securepassword123",
  "name": "John Doe",
  "software_level": "Intermediate",
  "hardware_level": "Beginner",
  "learning_goals": "Learn robot simulation and ROS 2 integration"
}
```

**Response**:
```json
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "message": "Account created successfully"
}
```

**Notes**:
- Global endpoint (not chapter-specific)
- User profile applies to all chapters (personalization works across Ch1 and Ch2)

---

### POST /api/auth/login

**Purpose**: Authenticate user and return JWT token

**Request**:
```json
{
  "email": "user@example.com",
  "password": "securepassword123"
}
```

**Response**:
```json
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
  "message": "Login successful"
}
```

**Notes**:
- Session persists across chapter navigation (per Constitution IV)
- No re-authentication needed when switching from Chapter 1 to Chapter 2

---

## 2. Personalization API (REUSE - Chapter-Agnostic)

### POST /api/personalize

**Purpose**: Adapt content based on user's software/hardware background

**Chapter 1 Example**:
```json
{
  "chapter_id": "chapter-1",
  "content_id": "ros2-nodes",
  "original_content": "In ROS 2, a node is a process that performs computation...",
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Chapter 2 Example (SAME ENDPOINT)**:
```json
{
  "chapter_id": "chapter-2",
  "content_id": "urdf-basics",
  "original_content": "URDF (Unified Robot Description Format) is an XML format for describing robot structures...",
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response**:
```json
{
  "personalized_content": "URDF (Unified Robot Description Format) is an XML format for describing robot structures, similar to HTML in web development. Just as HTML defines document structure with tags like <div> and <p>, URDF uses tags like <link> and <joint> to define robot components...",
  "background_level": "software-intermediate",
  "adjustments_made": [
    "Added software engineering analogies (XML → HTML comparison)",
    "Emphasized code structure and syntax",
    "Included programming best practices"
  ]
}
```

**Backend Logic** (Python):
```python
@router.post("/api/personalize")
async def personalize_content(
    chapter_id: str,
    content_id: str,
    original_content: str,
    user_id: str
):
    # Fetch user profile
    user = db.query(User).filter(User.id == user_id).first()

    # Generate personalization prompt
    if user.software_level == "Intermediate":
        prompt = f"""
        Adapt the following {chapter_id} content for a software engineer with intermediate experience.
        Add programming analogies and code-focused examples.

        Original content:
        {original_content}
        """
    elif user.hardware_level == "Intermediate":
        prompt = f"""
        Adapt the following {chapter_id} content for a hardware engineer with intermediate experience.
        Emphasize real-world components, specifications, and physical constraints.

        Original content:
        {original_content}
        """

    # Call OpenAI GPT-4
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )

    return {
        "personalized_content": response.choices[0].message.content,
        "background_level": f"{user.software_level or user.hardware_level}",
        "adjustments_made": [...]
    }
```

**Key Design**:
- `chapter_id` is passed but not used in core logic (personalization is content-based, not chapter-based)
- Same API works for Chapter 1, 2, 3, ..., N

---

## 3. Translation API (REUSE - Chapter-Agnostic)

### POST /api/translate

**Purpose**: Translate content to Urdu while preserving technical terms

**Chapter 1 Example**:
```json
{
  "chapter_id": "chapter-1",
  "content_id": "ros2-topics",
  "original_content": "In ROS 2, topics are named buses over which nodes exchange messages...",
  "target_language": "ur"
}
```

**Chapter 2 Example (SAME ENDPOINT)**:
```json
{
  "chapter_id": "chapter-2",
  "content_id": "gazebo-sensors",
  "original_content": "Gazebo supports camera, LiDAR, and IMU sensors through SDF plugins...",
  "target_language": "ur"
}
```

**Response**:
```json
{
  "translated_content": "Gazebo camera, LiDAR, اور IMU sensors کو SDF plugins کے ذریعے support کرتا ہے...",
  "technical_terms_preserved": ["Gazebo", "camera", "LiDAR", "IMU", "SDF plugins"],
  "cached": false,
  "translation_time_ms": 1247
}
```

**Backend Logic** (Python):
```python
@router.post("/api/translate")
async def translate_content(
    chapter_id: str,
    content_id: str,
    original_content: str,
    target_language: str = "ur"
):
    # Check translation cache (global, not chapter-specific)
    content_hash = hashlib.sha256(original_content.encode()).hexdigest()
    cached = db.query(TranslationCache).filter(
        TranslationCache.original_text_hash == content_hash,
        TranslationCache.language == target_language
    ).first()

    if cached:
        return {
            "translated_content": cached.translated_text,
            "cached": True
        }

    # Call OpenAI GPT-4 for translation
    prompt = f"""
    Translate the following technical content to Urdu.
    PRESERVE these technical terms in English: Gazebo, URDF, SDF, LiDAR, IMU, ROS 2, camera, physics engine, ODE, Bullet, Simbody.

    Content:
    {original_content}
    """

    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[{"role": "user", "content": prompt}]
    )

    translated_text = response.choices[0].message.content

    # Cache translation
    db.add(TranslationCache(
        original_text_hash=content_hash,
        original_text=original_content,
        translated_text=translated_text,
        language=target_language
    ))
    db.commit()

    return {
        "translated_content": translated_text,
        "cached": False
    }
```

**Key Design**:
- Translation cache is global (benefits all chapters)
- `chapter_id` helps with logging/analytics but doesn't affect translation logic

---

## 4. RAG Chatbot API (REUSE - Chapter-Filtered)

### POST /api/chatbot/ask

**Purpose**: Answer questions using chapter-specific vector embeddings

**Chapter 1 Example**:
```json
{
  "chapter_id": "chapter-1",
  "question": "How do I create a publisher in ROS 2?",
  "session_id": "sess-abc123",
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Chapter 2 Example (SAME ENDPOINT)**:
```json
{
  "chapter_id": "chapter-2",
  "question": "How do I add a LiDAR sensor to my robot in Gazebo?",
  "session_id": "sess-abc123",
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

**Response**:
```json
{
  "answer": "To add a LiDAR sensor to your robot in Gazebo, you need to define a <sensor> tag inside a <link> element in your robot's SDF file. Here's a basic example:\n\n```xml\n<sensor name=\"lidar\" type=\"ray\">\n  <ray>\n    <scan>\n      <horizontal>\n        <samples>640</samples>\n        <resolution>1</resolution>\n        <min_angle>-1.57</min_angle>\n        <max_angle>1.57</max_angle>\n      </horizontal>\n    </scan>\n    <range>\n      <min>0.1</min>\n      <max>10.0</max>\n    </range>\n  </ray>\n  <plugin name=\"lidar_plugin\" filename=\"libgazebo_ros_ray_sensor.so\"/>\n</sensor>\n```\n\nThis configuration creates a LiDAR sensor with 640 samples spanning 180 degrees (±1.57 radians) with a 10-meter maximum range.",
  "context_used": [
    {
      "text": "LiDAR sensors in Gazebo are defined using the <sensor> tag with type=\"ray\". The <scan> element specifies angular resolution and sample count...",
      "section": "sensors",
      "page": "lidar",
      "score": 0.89
    },
    {
      "text": "Sensor plugins connect Gazebo sensors to ROS 2 topics. For LiDAR, use libgazebo_ros_ray_sensor.so...",
      "section": "sensors",
      "page": "sensor-plugins",
      "score": 0.85
    }
  ],
  "confidence": 0.89
}
```

**Backend Logic** (Python):
```python
@router.post("/api/chatbot/ask")
async def ask_chatbot(
    chapter_id: str,
    question: str,
    session_id: str,
    user_id: str | None = None
):
    # Determine Qdrant collection from chapter_id
    collection_name = f"{chapter_id.replace('-', '_')}_embeddings"
    # Example: "chapter-2" → "chapter_2_embeddings"

    # Generate query embedding
    query_embedding = openai.Embedding.create(
        model="text-embedding-ada-002",
        input=question
    )["data"][0]["embedding"]

    # Search Qdrant with chapter filter
    results = qdrant_client.search(
        collection_name=collection_name,
        query_vector=query_embedding,
        limit=5,
        query_filter=Filter(
            must=[
                FieldCondition(
                    key="chapter_id",
                    match=MatchValue(value=chapter_id)
                )
            ]
        )
    )

    # Extract context
    context = "\n\n".join([hit.payload["text"] for hit in results])

    # Call GPT-3.5-turbo with context
    prompt = f"""
    You are a helpful assistant for a robotics textbook. Answer the question using ONLY the provided context.

    Context:
    {context}

    Question: {question}
    """

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": prompt}]
    )

    answer = response.choices[0].message.content

    # Store in chat_messages table
    db.add(ChatMessage(
        session_id=session_id,
        user_id=user_id,
        chapter_id=chapter_id,
        user_question=question,
        bot_response=answer,
        context_used=[
            {"text": hit.payload["text"], "section": hit.payload["section"], "score": hit.score}
            for hit in results
        ]
    ))
    db.commit()

    return {
        "answer": answer,
        "context_used": [...],
        "confidence": results[0].score if results else 0.0
    }
```

**Key Design**:
- `chapter_id` determines which Qdrant collection to search ("chapter_2_embeddings")
- RAG isolation ensures Chapter 2 chatbot only answers from Chapter 2 content
- Same logic works for Chapter 1, 2, 3, ..., N (just change collection name)

---

## 5. Progress Tracking API (REUSE - Chapter-Filtered)

### GET /api/progress

**Purpose**: Retrieve user progress across all chapters

**Request**:
```
GET /api/progress?user_id=550e8400-e29b-41d4-a716-446655440000
```

**Response**:
```json
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "chapters": [
    {
      "chapter_id": "chapter-1",
      "sections_completed": ["intro", "what-is-ros2", "why-ros2", "core-concepts", "examples", "exercises"],
      "total_sections": 6,
      "completion_percentage": 100.0,
      "quiz_score": 95,
      "completion_date": "2025-12-02T10:30:00Z",
      "last_accessed": "2025-12-02T10:30:00Z"
    },
    {
      "chapter_id": "chapter-2",
      "sections_completed": ["intro", "what-is-gazebo", "urdf-sdf"],
      "total_sections": 7,
      "completion_percentage": 42.86,
      "quiz_score": null,
      "completion_date": null,
      "last_accessed": "2025-12-03T14:20:00Z"
    }
  ],
  "overall_progress": {
    "chapters_completed": 1,
    "chapters_in_progress": 1,
    "total_chapters": 2,
    "global_completion_percentage": 71.43
  }
}
```

**Backend Logic** (Python):
```python
@router.get("/api/progress")
async def get_progress(user_id: str):
    # Fetch all progress records for user
    records = db.query(ProgressRecord).filter(
        ProgressRecord.user_id == user_id
    ).all()

    # Load chapters metadata
    with open("docs/chapters-config.json") as f:
        chapters_config = json.load(f)["chapters"]

    chapters_progress = []
    for chapter in chapters_config:
        record = next((r for r in records if r.chapter_id == chapter["id"]), None)

        if record:
            total_sections = len(chapter["sections"])
            completed_sections = len(record.sections_completed)
            completion_pct = (completed_sections / total_sections) * 100

            chapters_progress.append({
                "chapter_id": chapter["id"],
                "sections_completed": record.sections_completed,
                "total_sections": total_sections,
                "completion_percentage": completion_pct,
                "quiz_score": record.quiz_score,
                "completion_date": record.completion_date,
                "last_accessed": record.updated_at
            })
        else:
            # Chapter not started
            chapters_progress.append({
                "chapter_id": chapter["id"],
                "sections_completed": [],
                "total_sections": len(chapter["sections"]),
                "completion_percentage": 0.0,
                "quiz_score": null,
                "completion_date": null,
                "last_accessed": null
            })

    # Calculate overall progress
    total_completion = sum(c["completion_percentage"] for c in chapters_progress)
    global_completion_pct = total_completion / len(chapters_config)

    return {
        "user_id": user_id,
        "chapters": chapters_progress,
        "overall_progress": {
            "chapters_completed": len([c for c in chapters_progress if c["completion_percentage"] == 100]),
            "chapters_in_progress": len([c for c in chapters_progress if 0 < c["completion_percentage"] < 100]),
            "total_chapters": len(chapters_config),
            "global_completion_percentage": global_completion_pct
        }
    }
```

**Key Design**:
- API returns progress for ALL chapters (both Chapter 1 and Chapter 2)
- `chapter_id` field in `progress_records` table enables independent tracking

---

### POST /api/progress/update

**Purpose**: Update user progress when completing a section

**Chapter 1 Example**:
```json
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "chapter_id": "chapter-1",
  "section_completed": "exercises"
}
```

**Chapter 2 Example (SAME ENDPOINT)**:
```json
{
  "user_id": "550e8400-e29b-41d4-a716-446655440000",
  "chapter_id": "chapter-2",
  "section_completed": "physics"
}
```

**Response**:
```json
{
  "success": true,
  "updated_progress": {
    "chapter_id": "chapter-2",
    "sections_completed": ["intro", "what-is-gazebo", "urdf-sdf", "physics"],
    "completion_percentage": 57.14,
    "last_updated": "2025-12-03T15:30:00Z"
  }
}
```

**Backend Logic** (Python):
```python
@router.post("/api/progress/update")
async def update_progress(
    user_id: str,
    chapter_id: str,
    section_completed: str
):
    # Find existing progress record
    record = db.query(ProgressRecord).filter(
        ProgressRecord.user_id == user_id,
        ProgressRecord.chapter_id == chapter_id
    ).first()

    if record:
        # Append section if not already completed
        if section_completed not in record.sections_completed:
            record.sections_completed.append(section_completed)
            record.updated_at = datetime.utcnow()
            db.commit()
    else:
        # Create new progress record
        record = ProgressRecord(
            user_id=user_id,
            chapter_id=chapter_id,
            sections_completed=[section_completed],
            updated_at=datetime.utcnow()
        )
        db.add(record)
        db.commit()

    # Calculate completion percentage
    with open("docs/chapters-config.json") as f:
        chapters_config = json.load(f)["chapters"]
    chapter_meta = next(c for c in chapters_config if c["id"] == chapter_id)
    total_sections = len(chapter_meta["sections"])
    completion_pct = (len(record.sections_completed) / total_sections) * 100

    return {
        "success": True,
        "updated_progress": {
            "chapter_id": chapter_id,
            "sections_completed": record.sections_completed,
            "completion_percentage": completion_pct,
            "last_updated": record.updated_at
        }
    }
```

**Key Design**:
- `chapter_id` parameter selects which chapter's progress to update
- UNIQUE(user_id, chapter_id) constraint ensures one record per user per chapter

---

## 6. Summary

| API Endpoint | Chapter-Specific? | Chapter 2 Usage | Code Changes Needed |
|-------------|-------------------|-----------------|---------------------|
| POST /api/auth/signup | ❌ Global | Same as Chapter 1 | **None** |
| POST /api/auth/login | ❌ Global | Same as Chapter 1 | **None** |
| POST /api/personalize | ✅ Via param | `chapter_id="chapter-2"` | **None** |
| POST /api/translate | ✅ Via param | `chapter_id="chapter-2"` | **None** |
| POST /api/chatbot/ask | ✅ Via collection | Searches `chapter_2_embeddings` | **None** |
| GET /api/progress | ✅ Multi-chapter | Returns Ch1 + Ch2 progress | **None** |
| POST /api/progress/update | ✅ Via param | `chapter_id="chapter-2"` | **None** |

**Key Takeaway**: All APIs are production-ready for Chapter 2 with **ZERO code changes**. The only new infrastructure needed is:
1. Create `chapter_2_embeddings` Qdrant collection
2. Populate embeddings with Chapter 2 content

This demonstrates excellent API design from Chapter 1 - fully extensible to N chapters.
