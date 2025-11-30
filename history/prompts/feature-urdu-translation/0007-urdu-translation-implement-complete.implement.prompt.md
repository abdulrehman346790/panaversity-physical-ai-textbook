---
name: urdu-translation-implement-complete
agent: agent
description: Completed core MVP tasks for Urdu translation feature; ready for review and PR creation.
model: Raptor mini (Preview)
tools: []

---

## Prompt

Summarize the implementation of the Urdu translation MVP and prepare PR-ready artifacts.

## Response snapshot

- Implemented Docusaurus i18n with Urdu locale; translation microservice skeleton; translation pipeline; glossary & preservation tests; Chatbot translation integration; UI toggle & persistence; feedback flow; and tests + CI.

## Outcome

- ✅ Impact: MVP feature is functional in mock mode (no paid APIs required) and ready for further work.
- 🧪 Tests: Unit & integration tests added. E2E checks added for basic Urdu page presence.
- 📁 Files: Updated and created various files under `services/translate`, `docusaurus`, `rag-backend`, `specs/feature-urdu-translation`.
- 🔁 Next prompts: `/sp.implement` to finalize PRs, or `/sp.tasks` to generate sub-tasks for final polishing.
- 🧠 Reflection: Implementation is intentionally mock-first. Add cost-control & LLM live mode before production deployment.
