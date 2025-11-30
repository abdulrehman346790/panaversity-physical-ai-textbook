---
description: "Task list for Urdu Translation Feature (Docs + Chatbot + UI)"
---

# Tasks: Urdu Translation Feature

**Input**: Design docs from `/specs/feature-urdu-translation/`  
**Prerequisites**: `plan.md`, `spec.md`, `research.md`, `data-model.md`, `contracts/translate-openapi.yaml`, `quickstart.md`

## Format: `- [ ] T### [P?] [US?] Description (file path)`

- Tasks are grouped by phase and user story from the spec (US-UR-01..US-UR-04).
- Tasks are numbered sequentially starting at T201.

---

## Phase 1: Setup (Project & Environment) — Shared Infrastructure

- [x] T201 Initialize feature branch and add README for the feature (file: `specs/feature-urdu-translation/README.md`).
- [x] T202 [P] Set up Docusaurus i18n in `docusaurus.config.js` and enable Urdu locale `ur` (file: `docusaurus/docusaurus.config.js`).
- [x] T203 [P] Create `i18n/ur/` folder structure and scaffold placeholder pages (file: `docusaurus/i18n/ur/docusaurus-plugin-content-docs/current/`).
- [x] T204 [P] Add a simple translation microservice skeleton in `services/translate/` with `translate_service.py` and `Dockerfile` (file: `services/translate/translate_service.py`).
- [x] T205 [P] Add Redis config and Docker compose entry to the dev environment to host cache (file: `docker-compose.yml`) or document fallback (file: `specs/feature-urdu-translation/quickstart.md`).

---

## Phase 2: Foundational (Blocking Prerequisites)

- [x] T206 Create OpenAPI translation contract and server implementation stubs per `specs/feature-urdu-translation/contracts/translate-openapi.yaml` (file: `services/translate/api.py`).
- [x] T207 [P] Implement caching strategy skeleton: Redis + fallback logic + purge/eviction policy (file: `services/translate/cache.py`).
- [x] T208 [P] Create a translation glossary structure and a seed file with initial robotics/ROS terms (file: `specs/feature-urdu-translation/glossary/terms.yaml`).
- [x] T209 Create translation rules docs for technical content (how to preserve code blocks, YAML frontmatter, tables) (file: `specs/feature-urdu-translation/translation-rules.md`).
- [x] T210 [P] Add server-side user preference endpoint stubs if needed (file: `services/translate/user_pref.py`).
- [x] T211 [P] Add CI validation tests for the translate API contract (OpenAPI contract check - file: `.github/workflows/translate-ci.yml`).

---

## Phase 3: US-UR-01 — UI Toggle (Priority: P1) 🎯 MVP

**Goal**: Add a top-right language toggle that switches the UI (English ↔ Urdu) instantly and preserves page context.

- [x] T301 [US-UR-01] Add a React Context provider for language state (file: `docusaurus/src/components/Translation/TranslationContext.js`).
- [x] T302 [US-UR-01] Implement the navbar language toggle UI (file: `docusaurus/src/theme/NavbarItem/LanguageSwitcher.js`).
- [x] T303 [US-UR-01] Implement language state persistence to localStorage and user pref API (file: `docusaurus/src/components/Translation/TranslationContext.js`, `services/translate/user_pref.py`).
- [x] T304 [US-UR-01] Implement client-side detection and 'first-time hint' to toggle language based on browser locale preferences (file: `docusaurus/src/theme/HelloBar/` or plugin). 
- [x] T305 [US-UR-01] Add unit tests for TranslationContext and toggle behavior (file: `docusaurus/__tests__/TranslationContext.test.js`).
- [x] T306 [US-UR-01] Add E2E Playwright/Cypress test to assert toggle switches site to Urdu and back to English, preserving page context (file: `e2e/check_ur_pages.js`).

**Acceptance**: Clicking the toggle switches all visible UI to Urdu; the page does not refresh; the URL slug remains in the same position but in 'ur' locale path where applicable.

---

## Phase 4: US-UR-02 — Chatbot Urdu Mode (Priority: P1)

**Goal**: Chatbot must return Urdu responses when Urdu mode is active; allow English responses if user asks in English.

- [x] T401 [US-UR-02] Update Chatbot backend to consult user language preference (file: `rag-backend/agent.py` or `services/chatbot/handler.py`).
- [x] T402 [US-UR-02] Implement service to pass `target_locale` to the `/api/translate` call when generating responses (file: `services/chatbot/translate_integration.py`).
- [x] T403 [US-UR-02] Add tests verifying that when language pref = Urdu, responses are returned in Urdu and the chatbot warns if translation looks uncertain (file: `rag-backend/tests/test_chat_translation.py`).
- [x] T404 [US-UR-02] Add small icon/label on chat responses (UI) to mark "Machine-translated" when the response is auto-translated (file: `docusaurus/src/components/Chat/Message.js`).
- [ ] T405 [US-UR-02] Add a user feedback flow to report poor translations (file: `docusaurus/src/components/TranslationFeedback/`).

**Acceptance**: Chatbot replies are in Urdu when Urdu mode is selected; if the user asks in English, replies may be in English; a machine-translated label is visible with feedback link.

---

## Phase 5: US-UR-03 — Save User Language Preference (Priority: P2)

**Goal**: Save user language preference to Better Auth and local storage; use it to auto-configure the UI and the chatbot.

- [x] T501 [US-UR-03] Add API endpoint to set and get user language preferences (file: `services/translate/user_pref.py`).
- [x] T502 [US-UR-03] Integrate Better Auth user preference save in the sign-in flow and user profile (file: `rag-backend/personalization.py`).
- [x] T503 [US-UR-03] Add UI for selecting default language in user profile settings (file: `docusaurus/src/components/User/ProfileLanguageSettings.js`).
- [x] T504 [US-UR-03] Add integration & unit tests to confirm pref persistence across sessions and login (file: `rag-backend/tests/test_chat_translation.py`).

**Acceptance**: Language preference persists across login and is used to set UI and chatbot language on next visit.

---

## Phase 6: US-UR-04 — Preserve Markdown & Formatting (Priority: P1)

**Goal**: Ensure the translation microservice preserves Markdown structure and code blocks, tables, YAML frontmatter, and special tokens such as ROS topics or code snippets.

- [ ] T601 [US-UR-04] Implement a Markdown-aware translation pipeline that separates code blocks and metadata before translation (file: `services/translate/translate_pipeline.py`).
- [ ] T602 [US-UR-04] Add tests to ensure code blocks are preserved and not translated (file: `services/translate/tests/test_markdown_preservation.py`).
 - [x] T603 [US-UR-04] Add sample content and checksum preservation tests for table renderings (file: `specs/feature-urdu-translation/tests/markdown-preservation-samples.md`).
 - [x] T604 [US-UR-04] Add rule-based transformations for ROS/AI terminology to be referenced from `specs/feature-urdu-translation/glossary/`.

**Acceptance**: Generated Urdu Markdown renders correctly in Docusaurus with code blocks, tables, YAML frontmatter and ROS topics preserved and visible in the same format.

---

## Phase 7: Translate Existing Chapters (1–3) — Bulk Translation

**Goal**: Use the translation pipeline to translate Chapter 1–3 and prepare PRs for review.

- [ ] T701 [US-UR-03] Run translation script for each chapter and create PRs (file: `scripts/translate/translate_chapters.py`).
- [ ] T702 [US-UR-03] Human/agent review of translated pages; annotate issues and accept/override (file: `specs/feature-urdu-translation/review/`).
- [ ] T703 [US-UR-03] Add SEO-friendly slugs and meta tags for translated pages (file: `docusaurus/i18n/ur/docusaurus-plugin-content-docs/current/*`).
- [ ] T704 [US-UR-03] Add E2E tests to confirm that the generated Urdu pages render and link correctly in the locale-specific path (file: `e2e/ur-pages.spec.js`).

**Acceptance**: Chapters 1–3 appear in Urdu with matching structure and SEO metadata; translations are human-reviewed before merge.

---

## Phase 8: Frontend UX Improvements (Polish)

- [ ] T801 [P] Add `dir='rtl'` injection and consistent font usage for Urdu pages (file: `docusaurus/src/theme/rtl.css`).
- [ ] T802 [P] Add typography for Urdu and ensure font fallback is robust on mobile devices (file: `docusaurus/static/fonts/`).
- [ ] T803 [P] Fix layout issues for tables, lists, and code tabs in RTL mode (file: `docusaurus/src/theme/**` changes).
- [ ] T804 [P] Add unit and E2E tests confirming that RTL switching does not break the layout (file: `e2e/rtl-switch.spec.js`).

**Acceptance**: RTL pages render correctly across devices and page switching remains consistent.

---

## Phase 9: Automation & Maintenance Workflow (CI & PRs)

- [ ] T901 Create a GitHub Action that detects updates to English docs and creates a translation job/PR (file: `.github/workflows/auto-translate.yml`).
- [ ] T902 [P] Add manual override tag and review checklist to the auto-translation workflow to flag pages requiring human review (file: `.github/.github/translate-review-checklist.md`).
- [ ] T903 [P] Add rate-limit and cost controls to CI translation actions (ex: only translate changed sections, or limit to minor PRs) (file: `.github/workflows/auto-translate.yml`).
- [ ] T904 [P] Add a translation audit job that runs weekly and samples translations for manual review (file: `.github/workflows/translation-audit.yml`).

**Acceptance**: Auto-translation PRs created on content changes and maintainers can review/accept/override translations.

---

## Final Acceptance & Tasks
- [ ] T999 Finalize documentation and merge PRs for the feature; ensure all checklist items in `specs/feature-urdu-translation/checklists/requirements.md` PASS.

---

## Dependencies & Execution Order
- Phase 1 and Phase 2 must be complete before heavy translation tasks and US1–US3 implementation.
- US-UR-01 (UI) and US-UR-02 (Chatbot) should be prioritized as P1 for the MVP.
- Bulk translation (Phase 7) should only run after the translation pipeline and caching are validated.

---

## Summary
- Total tasks: 47 (T201–T211, T301–T306, T401–T405, T501–T504, T601–T604, T701–T704, T801–T804, T901–T904, T999)
- Per-story distribution:
  - Setup / Research / Shared: 11 tasks (T201–T211)
  - US-UR-01 (Toggle) / P1: 6 tasks (T301–T306)
  - US-UR-02 (Chatbot) / P1: 5 tasks (T401–T405)
  - US-UR-03 (User Preference) / P2: 4 tasks (T501–T504)
  - US-UR-04 (Markup preservation) / P1: 4 tasks (T601–T604)
  - Bulk Translation (Chapters 1–3): 4 tasks (T701–T704)
  - UX Improvements / Polish: 4 tasks (T801–T804)
  - Automation & Maintenance (CI): 4 tasks (T901–T904)
  - Finalize: 1 task (T999)

---

**Parallel Opportunities**
- Many tasks marked `[P]` can run in parallel such as skeleton creation, glossary, caching, tests, and CI setup.
- UI toggle and Chatbot changes can run in parallel once foundational APIs exist.

**MVP Recommendation**
- Deliver UI Toggle (US-UR-01), Chatbot Urdu Mode (US-UR-02), and saving user preference (US-UR-03) as MVP — implement formatting preservation and Glossary to improve translation accuracy.

**Test Coverage**
- Unit tests for translation pipeline + policy rules (T206, T601, T602)
- E2E tests for toggle and page rendering (T306, T704, T804)
- Contract tests for `translate` endpoints (T211)


---

**File path**: `specs/feature-urdu-translation/tasks.md`

