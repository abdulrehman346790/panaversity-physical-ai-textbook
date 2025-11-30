# Implementation Plan: Urdu Translation Feature

**Branch**: `feature-urdu-translation` | **Date**: 2025-11-30 | **Spec**: `specs/feature-urdu-translation/spec.md`
**Input**: Feature specification from `/specs/feature-urdu-translation/spec.md` and the user-provided plan outline

## Summary
Implement a dynamic translation system for Urdu that integrates with Docusaurus docs, the chatbot backend, and UI toggling. Emphasize a RAG/LLM-based translation microservice, caching, glossary management, and CSS-based RTL support. The feature will initially support dynamic on-demand translations with caching and automated PR generation.

## Technical Context
**Language/Version**: Node 18+ (Docusaurus) for frontend; Python 3.11+ (translation microservice) recommended. LLM: OpenAI models (GPT-4o/GPT-5) — top available model will be used and configurable.

**Primary Dependencies**:
- Docusaurus (vX) — i18n support
- Redis (caching)
- OpenAI API for translations
- GitHub Actions (CI)
- Better Auth (user settings)

**Testing**: Unit tests (node/pytest), E2E (Playwright/Cypress) for RTL and toggle behavior, contract tests for translate endpoint.

**Target Platform**: Linux-based CI, local dev via Docker, optional multi-platform testing for browsers.

**Performance Goal**: Page-language switch under 1s for cached items; translation time under 1.5s for chatbot responses.

**Constraints**: Ensure low-cost options for caches and limit LLM queries via caching and translations+glossary; maintain security and rate limits on translation endpoints.

## Constitution Check
- The feature aligns with project accessibility goals and technical constraints.
- Gate: Checklist must PASS before pushing to implementation; current plan resolves open clarifications.

## Project Structure
```
specs/feature-urdu-translation/
├── spec.md
├── plan.md
├── research.md
├── tasks.md (TBD)
├── data-model.md
├── quickstart.md
├── contracts/
│   └── translate-openapi.yaml
└── checklists/requirements.md

services/translate/
├── translate_service.py
├── requirements.txt
└── Dockerfile

examples/docusaurus/
├── docusaurus.config.js
├── i18n/
└── plugin-translation.js
```

## Phase 0: Research & Environment
- Resolve open questions (PDF export, machine-translated label, hybrid mode) — Done.
- Decision: Delay PDF export; label machine translations (subtle) with a feedback link; hybrid mode allowed as adv. toggle (default: single-language).

## Phase 1: Enable Docusaurus Multilingual Support
- Implement Docusaurus i18n configs and locale folder structure; add Urdu as `ur` and ensure `dir='rtl'` for pages.
- Deliverables: `docusaurus.config.js` updated, navbar switcher, placeholder i18n/ur pages.

## Phase 2: Build Translation Automation System
- Implement translation microservice: parse MD -> identify code blocks -> translate content portions -> preserve code and metadata.
- Implement glossary & translation rules.
- Deliverables: translation script, API, glossary, caching implementation.

## Phase 3: Translate Existing Chapters (1–3)
- Use translation pipeline to translate existing docs; generate PRs and apply a review process.
- Deliverables: translated chapters 1–3 + SEO metadata and slugs.

## Phase 4: Frontend UX Improvements for Urdu
- Add RTL support, fonts, ensure switching maintains page context, fix table/ code tab behavior.

## Phase 5: Automation & Maintenance Workflow
- Implement GitHub Actions to detect new changes and generate translation PRs; add manual override tags and a review checklist.

## Acceptance Criteria
- Toggle functional across website; translations for chapters 1–3; code blocks preserved; automation handles 90% of translation workload.

## Risks & Mitigations
- R-UR-01: Use glossary + manual overrides.
- R-UR-02: Add testing and component-level CSS.

## Next Steps
- Create `tasks.md` for the plan and implement skeleton translation microservice and i18n support.
- Add CI workflows and automated PR generation for docs.

