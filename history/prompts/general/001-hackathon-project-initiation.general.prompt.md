---
id: 001
title: Hackathon Project Initiation and Workflow Discussion
stage: general
date: 2025-12-03
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: affil
command: none
labels: ["hackathon", "project-initiation", "workflow", "planning"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docs/hackathon-requirements.md
tests: []
---

## Prompt

User shared hackathon requirements document for creating a textbook on Physical AI & Humanoid Robotics. The hackathon is a 5-day project requiring:

**Core Requirements (100 points):**
1. Create a Docusaurus book covering Physical AI & Humanoid Robotics (13-week curriculum)
2. Integrate RAG chatbot using OpenAI Agents/ChatKit SDKs, FastAPI, Neon Postgres, Qdrant Cloud

**Bonus Features (up to 200 extra points):**
3. Claude Code Subagents & Agent Skills (50 pts)
4. Authentication with better-auth.com including user background questions (50 pts)
5. Content personalization per chapter (50 pts)
6. Urdu translation per chapter (50 pts)

**Timeline:** Submission by Nov 30, 2025 at 6:00 PM with 90-second demo video

User initially requested to add hackathon details to CLAUDE.md file. User then chose "Option 2" (proper planning approach) and instructed in Urdu/Hinglish to:
- NOT do any work right now
- Remember the workflow: constitution → specification → plan → tasks → implement
- Wait for user to initiate when ready

## Response snapshot

1. Explained that CLAUDE.md is for operational rules, not project content
2. Saved hackathon requirements to `docs/hackathon-requirements.md`
3. Presented two approaches:
   - Option 1: Quick start (dive in immediately)
   - Option 2: Proper planning (Spec-Driven Development workflow)
4. User chose Option 2 but requested to wait
5. Acknowledged workflow sequence: constitution → specification → plan → tasks → implement
6. Confirmed ready to start when user initiates

## Outcome

- ✅ Impact: Hackathon requirements documented and saved; workflow approach agreed upon
- 🧪 Tests: No tests run (planning phase)
- 📁 Files: Created docs/hackathon-requirements.md (4,200+ lines)
- 🔁 Next prompts: User will initiate constitution phase when ready
- 🧠 Reflection: Proper project setup - avoided premature work, established clear workflow, saved comprehensive requirements for reference

## Evaluation notes (flywheel)

- Failure modes observed: None - successfully redirected from modifying CLAUDE.md to proper documentation location
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Wait for user to initiate constitution phase, then follow SDD workflow strictly
