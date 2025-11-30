# Feature Specification: RAG Chatbot for Physical AI Textbook

**Feature ID**: `rag-chatbot`
**Title**: Integrated RAG Chatbot with OpenAI Agents SDK
**Owner**: panaversity-physical-ai
**Status**: Draft
**Iteration**: v1
**Created**: 2025-11-30

## Summary
This feature implements a Retrieval-Augmented Generation (RAG) chatbot embedded within the Docusaurus textbook. The chatbot uses the **OpenAI Agents Python SDK** to orchestrate intelligent agents that can answer student questions based on the book's content. The backend is built with **FastAPI**, uses **Qdrant** for vector storage, and **Neon (Postgres)** for session persistence.

## Objectives
- Enable students to ask natural language questions about the course material.
- Provide accurate answers cited from the textbook content (RAG).
- Support context-aware conversations (memory).
- (Bonus) Personalize answers based on user background.
- (Bonus) Translate content on demand.

## Tech Stack
- **Framework**: FastAPI (Python)
- **Agent Orchestration**: OpenAI Agents Python SDK
- **Vector Database**: Qdrant Cloud (Free Tier)
- **Database**: Neon Serverless Postgres (for conversation history/sessions)
- **LLM**: OpenAI GPT-4o
- **Frontend Integration**: React component in Docusaurus

## Architecture

```mermaid
graph TD
    User[Student] -->|Chat Request| API[FastAPI Backend]
    API -->|Run Agent| Runner[Agent Runner]
    
    subgraph "OpenAI Agents SDK"
        Runner --> Triage[Triage Agent]
        Triage -->|Handoff| Expert[Book Expert Agent]
        Triage -->|Tool Call| Translator[Translator Agent]
        
        Expert -->|Tool Call| Search[search_book_tool]
    end
    
    Search -->|Query| Qdrant[Qdrant Vector DB]
    Qdrant -->|Chunks| Search
    
    Runner -->|Store/Load| Session[Neon Postgres (SQLAlchemySession)]
```

## Functional Requirements

### FR-4-001: RAG Pipeline (Ingestion)
**Requirement**: A script to parse all markdown files in `docs/`, chunk them, generate embeddings (text-embedding-3-small), and store them in Qdrant.
**Metadata**: Each chunk must include `source_file` and `chapter_title`.

### FR-4-002: RAG Pipeline (Retrieval)
**Requirement**: A function tool `search_book_content(query: str)` that queries Qdrant and returns relevant text chunks.

### FR-4-003: Agent Structure
**Requirement**:
1.  **Triage Agent**: Determines intent (Question about book vs General chit-chat).
2.  **Book Expert Agent**: Specialized agent with `search_book_content` tool. Instructions to answer strictly based on context.
3.  **Translator Agent** (Bonus): Agent as a tool to translate responses if requested.

### FR-4-004: Session Management
**Requirement**: Use `SQLAlchemySession` with Neon Postgres to persist conversation history across page reloads.

### FR-4-005: API Endpoints
**Requirement**:
- `POST /chat`: Accepts `{message, session_id, user_context}`. Returns `{response}`.
- `POST /ingest`: Triggers the ingestion process.

## Non-Functional Requirements
- **Latency**: Chat responses should be generated within 5 seconds (excluding model generation time).
- **Accuracy**: Answers must be grounded in the provided text chunks.

## User Stories

### US-4-01: Ask a Question
**As a** student
**I want to** ask "How do I install Webots?"
**So that** I get a specific answer without searching the whole chapter.

### US-4-02: Contextual Follow-up
**As a** student
**I want to** ask "What about Windows?" after asking about installation
**So that** the bot understands I mean "How do I install Webots on Windows?".

## Implementation Plan

### Phase 1: Backend Setup
- Initialize FastAPI project `rag-backend/`.
- Configure Qdrant and OpenAI clients.
- Setup Neon database connection.

### Phase 2: Ingestion Script
- Implement `ingest.py` to process `docusaurus/book/docs/`.
- Run ingestion to populate Qdrant.

### Phase 3: Agent Implementation
- Define `search_book_content` tool.
- Create `Book Expert Agent`.
- Create `Triage Agent`.
- Implement `POST /chat` endpoint using `Runner`.

### Phase 4: Frontend Integration
- Create a Docusaurus Chat Component (using Swizzling or a custom React component).
- Connect it to the FastAPI backend.

## Bonus Features (Targeting 50+ Points)

### Personalization
- Pass `user_background` (e.g., "CS Student" vs "Hobbyist") in the API request.
- Inject this into the Agent's system prompt or context.

### Translation
- Add a button in the UI or handle requests like "Explain in Urdu".
- Use a `Translator Agent` to rewrite the final response.

## Dependencies
- `openai-agents`
- `fastapi`
- `uvicorn`
- `qdrant-client`
- `sqlalchemy`
- `psycopg2-binary` (for Neon)
