---
description: "Task list for RAG Chatbot feature implementation"
---

# Tasks: RAG Chatbot for Physical AI Textbook

**Input**: Design documents from `/specs/004-rag-chatbot/`
**Prerequisites**: spec.md

**Tests**: No tests were explicitly requested in the spec, so they are omitted.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2)
- Include exact file paths in descriptions

## Path Conventions

- Backend: `rag-backend/`
- Frontend: `docusaurus/book/src/`

---

## Phase 1: Setup (Backend Setup)

**Purpose**: Project initialization and basic structure for the FastAPI backend.

- [ ] T001 Create project directory `rag-backend/`.
- [ ] T002 Initialize Python project with a virtual environment and `requirements.txt`.
- [ ] T003 Add dependencies to `rag-backend/requirements.txt`: `fastapi`, `uvicorn`, `qdrant-client`, `openai`, `python-dotenv`, `sqlalchemy`, `psycopg2-binary`.
- [ ] T004 Create main application file `rag-backend/main.py` with a basic FastAPI app instance.
- [ ] T005 [P] Create a configuration module `rag-backend/config.py` to load environment variables (OpenAI API key, Qdrant URL, Neon DB URL).
- [ ] T006 [P] Create a database connection module `rag-backend/database.py` to set up the SQLAlchemy engine and session management for Neon Postgres.

---

## Phase 2: Foundational (Ingestion Pipeline - FR-4-001)

**Purpose**: Core data pipeline that MUST be complete before any user story can be implemented.

**⚠️ CRITICAL**: No user story work can begin until this phase is complete.

- [ ] T007 Implement the data ingestion script `rag-backend/ingest.py`.
- [ ] T008 In `rag-backend/ingest.py`, add logic to find all markdown files in `docusaurus/book/docs/`.
- [ ] T009 In `rag-backend/ingest.py`, implement text chunking for the markdown files.
- [ ] T010 In `rag-backend/ingest.py`, connect to Qdrant and OpenAI to generate and store embeddings (`text-embedding-3-small`) for each chunk. Ensure `source_file` and `chapter_title` metadata is included.
- [ ] T011 Create the `POST /ingest` endpoint in `rag-backend/main.py` that triggers the ingestion script.
- [ ] T012 Run the ingestion process to populate the Qdrant database.

**Checkpoint**: Foundation ready - Qdrant is populated with book content. User story implementation can now begin.

---

## Phase 3: User Story 1 - Ask a Question (US-4-01) 🎯 MVP

**Goal**: A student can ask a question and get a specific answer from the book content.

**Independent Test**: Send a `POST` request to `/chat` with a question like "How do I install Webots?". Expect a JSON response containing an answer derived from the book.

### Implementation for User Story 1

- [ ] T013 [P] In a new file `rag-backend/tools.py`, define the function tool `search_book_content(query: str)` that queries Qdrant for relevant chunks (FR-4-002).
- [ ] T014 [P] In a new file `rag-backend/agents.py`, create the `Book Expert Agent` with instructions to answer strictly from the provided context and give it the `search_book_content` tool (FR-4-003).
- [ ] T015 In `rag-backend/agents.py`, create the `Triage Agent` to determine user intent (Book Question vs. General Chit-chat) and hand off to the `Book Expert Agent` if appropriate (FR-4-003).
- [ ] T016 In `rag-backend/main.py`, implement the `POST /chat` endpoint (FR-4-005).
- [ ] T017 In the `/chat` endpoint, initialize the `AgentRunner` from the `openai-agents` SDK and pass it the `Triage Agent`.

**Checkpoint**: At this point, User Story 1 should be fully functional and testable via an API client.

---

## Phase 4: User Story 2 - Contextual Follow-up (US-4-02)

**Goal**: The chatbot remembers previous turns in the conversation.

**Independent Test**: 1) Ask "How do I install Webots?". 2) In a subsequent request with the same `session_id`, ask "What about Windows?". The bot should understand the context and answer about installing Webots on Windows.

### Implementation for User Story 2

- [ ] T018 In `rag-backend/database.py`, define the SQL schema for conversation history (`schema.sql`) and create a table using `init_db.py`.
- [ ] T019 In `rag-backend/main.py`, modify the `/chat` endpoint to accept an optional `session_id`.
- [ ] T020 In `rag-backend/main.py`, instantiate `SQLAlchemySession` from the SDK, passing the session manager to the `AgentRunner`.
- [ ] T021 Ensure the `session_id` is used to load and save conversation history, enabling contextual follow-up questions (FR-4-004).

**Checkpoint**: At this point, the chatbot should support multi-turn conversations.

---

## Phase 5: Frontend Integration

**Goal**: Embed the chatbot into the Docusaurus frontend.

**Independent Test**: Open the Docusaurus site, see the chat widget, ask a question, and receive an answer from the backend.

- [ ] T022 Create a new React component for the chat interface in `docusaurus/book/src/theme/ChatWidget.js`.
- [ ] T023 Style the `ChatWidget` component with a text input, a submit button, and a message display area.
- [ ] T024 In `ChatWidget.js`, implement the logic to call the `POST /chat` endpoint on the FastAPI backend.
- [ ] T025 Manage chat state (messages, loading status, session ID) within the React component.
- [ ] T026 "Swizzle" the Docusaurus root component or wrap the layout to include the `ChatWidget` on all pages.

---

## Phase N: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories.

- [ ] T027 [P] Add robust error handling to the `/chat` and `/ingest` endpoints.
- [ ] T028 [P] Implement logging throughout the `rag-backend` application to trace requests and agent execution.
- [ ] T029 Review and add documentation (docstrings, README) for the backend service.

---

## Dependencies & Execution Order

### Phase Dependencies
- **Setup (Phase 1)**: Can start immediately.
- **Foundational (Phase 2)**: Depends on Setup completion. BLOCKS all user stories.
- **User Stories (Phase 3 & 4)**: Depend on Foundational phase completion.
- **Frontend (Phase 5)**: Depends on User Story 1 completion.

### User Story Dependencies
- **US-4-01 (Ask a Question)**: Can start after Foundational (Phase 2).
- **US-4-02 (Contextual Follow-up)**: Depends on US-4-01. The chat endpoint must exist before session management can be added.
