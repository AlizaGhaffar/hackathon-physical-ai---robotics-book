---
id: 005
title: Add RAG Backend Principles to Constitution
stage: constitution
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: affil
command: /sp.constitution
labels: ["constitution", "rag", "backend", "fastapi", "qdrant", "neon", "openai"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

step 2    ye hai 2\. Integrated RAG Chatbot Development: Build and embed a Retrieval-Augmented Generation (RAG) chatbot within the published book. This chatbot, utilizing the OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres database, and Qdrant Cloud Free Tier, must be able to answer user questions about the book's content, including answering questions based only on text selected by the user. ismain tum backend per fast api use kro convert text to embedding  saves vector into qdrant retrives relevent book sections sends final context to openai or data base neon Serverless Postgres ka use karo storing ky liye to sahi to means backend generate karwao g  to subse pehle iski contitution banani hai tumne /sp.constitution banao

Translation: "In this, you use FastAPI on backend, convert text to embedding, save vector into Qdrant, retrieve relevant book sections, send final context to OpenAI. Or use Neon Serverless Postgres database for storing. So that's right - means generate backend. So first of all, we need to make its constitution, you need to make /sp.constitution"

## Response snapshot

Updated project constitution from version 1.2.0 to 1.3.0 (MINOR version bump).

Added comprehensive RAG chatbot backend principles (Sections XIII-XIX):
- XIII. RAG Backend Architecture - FastAPI clean architecture with service separation
- XIV. Embedding Pipeline Excellence - OpenAI embeddings, chunking strategy, batch processing
- XV. Vector Storage Strategy - Qdrant Cloud integration, namespacing, metadata management
- XVI. Context Retrieval Precision - Semantic search, re-ranking, chapter-scoped queries
- XVII. Response Generation Quality - GPT-4 integration, prompt engineering, streaming
- XVIII. Database Management - Neon Serverless Postgres for chat history and analytics
- XIX. Selected Text Query Support - User-selected text Q&A feature

Added technical sections:
- RAG Backend Technology Stack - Python 3.11+, FastAPI, Qdrant, Neon, OpenAI dependencies
- RAG API Standards - 4 required endpoints (query, embed, health, feedback), performance/security requirements

Constitution validates against existing templates:
- ✅ plan-template.md (RAG backend follows web app structure)
- ✅ spec-template.md (RAG features map to user stories)
- ✅ tasks-template.md (RAG tasks follow phase-based implementation)

## Outcome

- ✅ Impact: Constitution now includes 7 new RAG backend principles with 56 non-negotiable rules covering architecture, embeddings, vector storage, retrieval, response generation, database management, and selected text querying. Provides complete governance for building production-ready RAG chatbot backend.
- 🧪 Tests: N/A (constitution document)
- 📁 Files: Updated .specify/memory/constitution.md (version 1.2.0 → 1.3.0)
- 🔁 Next prompts: Create RAG backend specification using /sp.specify, then planning with /sp.plan
- 🧠 Reflection: Constitution successfully extended with detailed RAG backend principles while preserving existing chapter-focused principles. Clear separation between frontend (existing Docusaurus) and backend (new FastAPI RAG service). All technology choices align with hackathon requirements (OpenAI, Qdrant, Neon, FastAPI).

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All placeholders filled, version incremented correctly, principles are testable and specific
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Proceed to specification phase with /sp.specify to define RAG backend user stories and functional requirements
