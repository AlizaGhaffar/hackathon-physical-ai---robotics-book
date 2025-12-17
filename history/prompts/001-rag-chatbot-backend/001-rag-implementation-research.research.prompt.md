---
id: 001
title: RAG Implementation Best Practices Research
stage: research
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-rag-chatbot-backend
branch: 001-rag-chatbot-backend
user: aliza
command: none
labels: ["rag", "research", "fastapi", "openai", "qdrant", "best-practices"]
links:
  spec: /mnt/c/Users/affil/Desktop/book/specs/001-rag-chatbot-backend/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - /mnt/c/Users/affil/Desktop/book/specs/001-rag-chatbot-backend/research.md
tests:
  - none (research phase)
---

## Prompt

Research RAG (Retrieval-Augmented Generation) implementation best practices for a FastAPI backend with the following stack:

**Context**: Building a RAG chatbot backend for a Physical AI textbook. Users ask questions about book content, system retrieves relevant chunks from vector database, generates answers with GPT-4.

**Technology Stack**:
- FastAPI (Python 3.11+)
- OpenAI API (text-embedding-3-small for embeddings, GPT-4-turbo for generation)
- Qdrant Cloud (vector database)
- Neon Serverless Postgres (chat history)

**Research Questions**:

1. **Text Chunking Strategy**: What are best practices for chunking educational/technical content for RAG?
   - Optimal chunk size and overlap for textbook content?
   - Should chunks respect semantic boundaries (paragraphs, sections, code blocks)?
   - How to preserve context across chunks (heading metadata, etc.)?

2. **Embedding Best Practices**: OpenAI text-embedding-3-small usage patterns
   - Batch size recommendations for embedding multiple chunks?
   - Rate limiting and cost optimization strategies?
   - Should queries and content use same embedding model?

3. **Vector Search Optimization**: Qdrant-specific best practices
   - Collection structure for multi-chapter book (single collection with metadata vs. multiple collections)?
   - Optimal similarity threshold for educational content (cosine similarity)?
   - Metadata filtering performance considerations (chapter scoping)?

4. **Context Retrieval**: How many chunks to retrieve (top-K) for optimal answer quality?
   - Is top-5 sufficient or should it be dynamic based on query?
   - Should retrieved chunks be re-ranked or just use vector similarity scores?
   - How to handle selected-text queries (boost specific chunks)?

5. **Prompt Engineering**: Best practices for RAG prompts with GPT-4
   - How to structure system prompt for educational Q&A?
   - How to format retrieved chunks in the prompt (citations, separators)?
   - How to instruct model to say "I don't know" when context is insufficient?

6. **Async FastAPI Patterns**: Best practices for async RAG pipeline
   - Should embedding/vector-search/GPT calls be sequential or any parallelization opportunities?
   - Connection pooling for Qdrant and Neon?
   - Streaming responses (SSE) implementation patterns?

7. **Error Handling**: Common failure modes in RAG systems
   - What happens when OpenAI API is down (retry logic, fallbacks)?
   - How to handle Qdrant timeouts or connectivity issues?
   - Should system degrade gracefully or fail fast?

8. **Security**: RAG-specific security considerations
   - Prompt injection attack prevention (malicious user queries)?
   - Rate limiting strategies (per-user, per-IP, global)?
   - How to secure admin embedding endpoints?

**Output Format**: For each research question, provide:
- **Decision**: Recommended approach
- **Rationale**: Why this is best practice
- **Alternatives Considered**: Other options and why they weren't chosen
- **Implementation Guidance**: Specific values/patterns to use

Focus on production-ready patterns, not experimental techniques.

## Response snapshot

I conducted comprehensive research using web searches of 2025 industry sources and compiled a detailed research document covering all 8 areas. The research document is structured as a decision log with rationale, alternatives, and implementation guidance for each component.

**Key Decisions Made**:

1. **Chunking**: Recursive chunking with 400-600 tokens and 50-100 token overlap
   - Rationale: 88-91% accuracy, 10x more cost-efficient than semantic chunking, preserves document structure

2. **Embeddings**: OpenAI text-embedding-3-small for both queries and documents
   - Rationale: MUST use same model for comparable vector spaces, batch processing for cost optimization

3. **Vector Search**: Single Qdrant collection with metadata filtering
   - Rationale: Filterable HNSW maintains speed, flexible cross-chapter search, simpler management

4. **Retrieval**: Two-stage retrieval (over-fetch top-20, re-rank to top-5)
   - Rationale: 48% accuracy improvement from re-ranking, minimal latency overhead (50-100ms)

5. **Prompting**: Structured system prompt with strict grounding and citation requirements
   - Rationale: Prevents hallucinations, enables verification, educational tone

6. **FastAPI**: Fully async with SSE streaming and connection pooling
   - Rationale: 10-100x throughput improvement, better UX with streaming

7. **Error Handling**: Circuit breaker pattern with graceful degradation
   - Rationale: Prevents cascading failures, maintains partial functionality during outages

8. **Security**: Multi-layered defense (input validation + rate limiting + output filtering)
   - Rationale: OWASP LLM01:2025 compliance, prompt injection prevention, abuse protection

The research document includes production checklist, code examples, and 40+ authoritative sources from 2025.

**File Created**: `/mnt/c/Users/affil/Desktop/book/specs/001-rag-chatbot-backend/research.md` (12,500+ words)

## Outcome

- ✅ Impact: Comprehensive research document with production-ready best practices for all 8 RAG implementation areas
- 🧪 Tests: None (research phase)
- 📁 Files: Created research.md with 8 detailed sections covering chunking, embeddings, vector search, retrieval, prompting, FastAPI patterns, error handling, and security
- 🔁 Next prompts: Update plan.md with research findings, generate tasks.md for implementation
- 🧠 Reflection: Research was thorough and current (2025 sources). Key insight: Two-stage retrieval (re-ranking) provides 48% accuracy improvement for minimal latency cost. Security considerations (prompt injection) are critical for RAG systems and often overlooked.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (research phase)
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Implement chunking strategy on sample textbook content to validate 400-600 token recommendation
