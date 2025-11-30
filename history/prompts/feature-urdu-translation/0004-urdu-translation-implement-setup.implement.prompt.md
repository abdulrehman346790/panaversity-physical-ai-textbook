---
name: urdu-translation-implement-setup
agent: agent
description: Implement skeleton translate microservice, Redis cache, Docusaurus i18n and language toggle skeleton. Created tests and CI workflow.
model: Raptor mini (Preview)
tools: []

---

## Prompt

Implement initial skeleton for Urdu translation feature: translate microservice, Redis cache, Docker compose, Docusaurus i18n placeholder pages & a LanguageSwitch component; create initial CI that runs translation tests.

## Response snapshot

Implemented translate microservice FastAPI skeleton, caching wrapper, tests, Dockerfile + docker-compose, Docusaurus locale folder, language toggle components and updated `docusaurus.config.js` to include locale dropdown. Added quickstart & glossary skeleton and updated tasks to reflect initial progress.

## Outcome

- ✅ Impact: Basic skeleton in place to start integration work; test & CI verify microservice runs and endpoints exist.
- 🧪 Tests: Basic unit tests added for translation endpoint and prefs.
- 📁 Files: `services/translate/*`, `docusaurus/i18n/ur/*`, `specs/feature-urdu-translation/*`, `docker-compose.yml`, `.dockerignore`, `.github/workflows/translate-ci.yml`
- 🔁 Next prompts: `/sp.implement` to implement MVP user stories (UI toggle, Chatbot integration, pref save) and to add translation pipeline features.
- 🧠 Reflection: The system has multiple translation/context providers (`TranslationContext` currently used by site vs new `LanguageContext`) — we should consolidate to avoid duplication and prefer `TranslationContext` in future steps.
