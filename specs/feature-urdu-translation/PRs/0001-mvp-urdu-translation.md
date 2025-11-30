PR: MVP — Urdu translation (Docs + Chatbot + UI)

Summary:
- Add Docusaurus i18n config with Urdu locale
- Add a translation microservice skeleton (FastAPI) with mock mode and tests
- Add translation pipeline that preserves code blocks and glossary-controlled terms
- Integrate Chatbot backend to translate responses when user preference is Urdu
- Add a UI language toggle and language persistence (localStorage + backend preference)
- Add feedback UI and backend feedback endpoint
- Add unit and integration tests for translation and chat flows

Files changed:
- `docusaurus` config and components
- `services/translate` microservice and tests
- `rag-backend` integration + tests
- `specs/feature-urdu-translation` (spec, tasks, research, plan)

Next steps:
- Complete `translate_pipeline` and integrate real LLM (OpenAI) in `LIVE` mode
- Add CI auto-translation PR job and translation audit
- Expand E2E Playwright test coverage for UI and Chatbot translation flows
