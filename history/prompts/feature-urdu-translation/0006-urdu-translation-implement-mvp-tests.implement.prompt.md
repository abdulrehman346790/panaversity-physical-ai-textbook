---
name: urdu-translation-mvp-tests
agent: agent
description: Added backend chat integration, translation microservice tests, and CI for rag-backend and translation-service. Also implemented frontend toggle, language persistence, and chat label for auto-translated replies.
model: Raptor mini (Preview)
tools: []

---

## Prompt

Implement tests and CI for the MVP features: translation service, chat translation flow, and basic frontend toggle and context tests.

## Response snapshot

- Added `services/translate/tests/test_translate.py` with translation endpoint tests.
- Added `rag-backend/tests/test_chat_translation.py` to verify chat flow with preferences set.
- Added `e2e/check_ur_pages.js` for a baseline E2E site pages check.
- Created GitHub Actions for `services/translate` and `rag-backend` to run tests on push/PR.

## Outcome

- ✅ Impact: Basic tests ensure translation microservice and chat translation pipeline remain functional.
- 🧪 Tests: pytest used for microservice & backend tests; a basic Node script checks Urdu i18n pages presence.
- 📁 Files: `services/translate/tests`, `rag-backend/tests/test_chat_translation.py`, `.github/workflows/translate-ci.yml`, `.github/workflows/rag-backend-ci.yml`.
- 🔁 Next prompts: `/sp.implement` to proceed on T401..T405 (feedback) and T601..T604 (Markdown preservation pipeline).
- 🧠 Reflection: Next iteration should focus on Markdown-aware translation logic + glossary enforcement, then expand E2E tests with Playwright.
