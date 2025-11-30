---
name: urdu-translation-plan-complete
agent: agent
description: Implementation plan prepared for Urdu translation feature; includes phases and tasks.
model: Raptor mini (Preview)
tools: []

---

## Prompt

Create an implementation plan for the Urdu translation feature, resolving clarifications and outlining phases, tasks, and deliverables.

## Response snapshot

Created `plan.md`, research, data model, openapi contract for `/api/translate`, and quickstart for dev environment. Resolved OQ decisions: PDF export deferred; machine translation labeled with subtle indicator and feedback; hybrid mode allowed as advanced toggle.

## Outcome

- ✅ Impact: Plan enables pipeline and phases for translation automation and Docs + Chatbot integration.
- 🧪 Tests: Quickstart + translations pipeline; CI required for automated PR creation.
- 📁 Files: `plan.md`, `research.md`, `data-model.md`, `translate-openapi.yaml`, `quickstart.md`.
- 🔁 Next prompts: `/sp.tasks` to generate task-level jobs for the implementation.
- 🧠 Reflection: Keep heavy simulator or full-translation jobs optional in CI; rely on caching and glossary to reduce errors.
