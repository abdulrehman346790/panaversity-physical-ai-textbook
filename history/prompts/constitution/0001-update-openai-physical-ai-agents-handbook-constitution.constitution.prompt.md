---
id: 0001
title: Update OpenAI Physical AI & Agents Handbook Constitution
stage: constitution
date: 2025-11-29
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: unknown
command: /sp.phr
labels: [constitution, update, project-setup]
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

/sp.constitution

Our project is a comprehensive educational book and documentation website titled **“The OpenAI Physical AI & Agents Handbook”**. Its purpose is to teach students, developers, and beginners how to understand, build, and use AI Agents, Physical AI concepts, RAG systems, and the OpenAI ecosystem in simple, structured, and highly practical language.

### 📘 Core Purpose
To produce a clear, complete, beginner-friendly book (in Markdown format) that will later be integrated into a Docusaurus documentation website and enhanced with an embedded RAG-based chatbot.

### 🧭 Core Principles
1. **Clarity over Complexity**
   All explanations must be simple, beginner-readable, and practical. No unnecessary theory.

2. **Chapter-wise Development**
   Each chapter will be generated using an iterative cycle:
   `/sp.specify → /sp.plan → /sp.tasks → /sp.implement`.

3. **Consistency & Modularity**
   Every chapter must follow the same structure:
   - Introduction
   - Key Concepts
   - Examples
   - Visuals (described in text)
   - Summary
   - Practice Questions

4. **Markdown-First Philosophy**
   All final content should be clean `.md` format and ready to be moved into `/docs` for Docusaurus.

5. **RAG-Ready Writing**
   Content must be structured in a way that is easy for a vector database (Qdrant) to index later.

6. **Beginner Accessibility**
   Assume the reader is a college student or beginner developer with basic Python knowledge.

7. **Accuracy & Practicality**
   Content must reflect the latest OpenAI Agents SDK, modern LLM concepts, and real-world examples.

### 🎯 Deliverables
1. A full book containing:
   - Introduction to AI & Physical AI
   - OpenAI Agents SDK
   - RAG systems
   - Vector Databases
   - Embeddings
   - Chatbot construction
   - FastAPI backend
   - Deployment and best practices

2. Modular Markdown files for each chapter.

3. A structure suitable for:
   - Docusaurus documentation
   - RAG-based chatbot indexing
   - GitHub Pages deployment

### 🔄 Workflow Rules
- Every chapter must follow the Spec-Kit Plus 4-step process.
- All technical examples should be runnable.
- Explanations must be written in simple, human-like English.
- Maintain consistency in formatting across the entire book.

### 🚀 Goal
To finish the entire book quickly and cleanly so it can be plugged directly into a Docusaurus site and extended with a fully functional RAG + chatbot system for the hackathon submission.


## Response snapshot

The project constitution has been updated with "The OpenAI Physical AI & Agents Handbook" content.

## Outcome

- ✅ Impact: Constitution for "The OpenAI Physical AI & Agents Handbook" updated successfully.
- 🧪 Tests: none
- 📁 Files: 1 file modified (.specify/memory/constitution.md)
- 🔁 Next prompts: Review dependent templates (plan, spec, tasks) for consistency.
- 🧠 Reflection: Ensure all placeholders in PHR template are consistently handled.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): none
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Automate consistency checks for dependent templates.
