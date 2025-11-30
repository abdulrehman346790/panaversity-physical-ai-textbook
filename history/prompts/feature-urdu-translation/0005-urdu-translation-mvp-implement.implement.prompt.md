---
name: urdu-translation-mvp-implement
agent: agent
description: Implemented MVP features for Urdu translation: UI toggle, Chatbot integration with translation microservice, user preferences, and initial tests/CI skeleton.
model: Raptor mini (Preview)
tools: []

---

## Prompt

Implement MVP for the Urdu translation feature: translation microservice skeleton, Docusaurus i18n and language toggle, Chatbot backend integration, preference endpoints, glossary and translation rules, initial tests and CI.

## Response snapshot

- Implemented FastAPI translation microservice skeleton, caching module, Dockerfile, and tests.
- Added Docusaurus i18n config for `ur`, created a language toggle, and integrated UI components to use `TranslationContext`.
- Added in-memory personalization endpoints to set/get user language preference.
- Wired `rag-backend` chat endpoint to consult user language and call translation microservice to translate output into Urdu if requested.
- Added tests for translation endpoint and chat translation flow; added Github Actions CI for translation service tests.

## Outcome

- ✅ Impact: MVP toggle and Chatbot translation integration functional.
- 🧪 Tests: Tests added for translation service and basic chat translation flow.
- 📁 Files: `services/translate/*`, `docusaurus/*` components, `rag-backend` changes, tasks updated.
- 🔁 Next prompts: `/sp.implement` to finish T401..T405, T301..T306 and additional E2E tests.
- 🧠 Reflection: Consolidate multiple language contexts into a single TranslationContext to reduce duplication.
