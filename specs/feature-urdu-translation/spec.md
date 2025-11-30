# Feature Specification: Urdu Translation System (Docs + Chatbot + UI)

**Feature ID**: feature-urdu-translation
**Branch**: `feature-urdu-translation`
**Created**: 2025-11-30
**Status**: Draft
**Owner**: panaversity-physical-ai
**Contributors**: Abdul Rehman, AI Agent (SpecifyPlus)

## Overview

This feature adds Urdu language support across the platform including Docusaurus documentation, chatbot responses, and UI components with an emphasis on preserving Markdown structure, correct layout (RTL) for Urdu readability, and caching to reduce cost. The goal is to make robotics, AI, and ROS 2 education accessible to Urdu-speaking students.

## Motivation

- Urdu translations increase accessibility for many students.
- A dual-language UI (Docs + Chatbot) improves comprehension and adoption.
- This aligns with the platform's mission of inclusive education.

## Feature Goals

- Full English ↔ Urdu translation toggle in the UI.
- Chatbot replies in Urdu when userdata selects Urdu mode.
- Docusaurus docs support Urdu mode dynamically with Markdown structure intact.
- RTL styling applied when Urdu is selected.
- User language pref stored in Better Auth and persisted across sessions.

## Scope

### In Scope
- UI toggle for switching languages.
- Dynamic translation using an OpenAI-based API (runtime translation) — implementation detail moved to Recommended Environment.
- RTL layout integration for Urdu.
- Chatbot Urdu mode.
- Translation of headings, text blocks, tables, callouts, sidebar items, and code comments (optional).
- Caching translations to reduce cost and latency.
- Store user preference in Better Auth.

### Out of Scope
- Manual translation of all chapters (automated translation only).
- Multi-language support beyond Urdu.
- Offline translation engines.
- Urdu text-to-speech (TTS).
- Grammar correction for user inputs.

## Personas

- P1: Beginner Robotics Student — needs Urdu explanations for improved comprehension.
- P2: AI + ROS Learner — wants chatbot explanations in Urdu while reading code in English.
- P3: Instructor — wants bilingual lessons for delivery.

## User Stories

- US-UR-01: As a student, I want to toggle English ↔ Urdu so I can learn in my preferred language.
- US-UR-02: As a learner, I want the chatbot to respond in Urdu when I choose Urdu mode.
- US-UR-03: As a user, I want my language preference saved automatically.
- US-UR-04: As a beginner, I want translated text to keep the same formatting and structure.

## Functional Requirements

- FR-UR-01: Implement a UI language toggle in the top-right navbar that switches English ↔ Urdu instantly using state.
- FR-UR-02: Create backend translation endpoint `/api/translate` (accepts Markdown and returns Urdu preserving structure). The translation approach is detailed in Recommended Environment.
- FR-UR-03: Chatbot returns responses in Urdu when Urdu mode is active; English allowed if user asks in English.
- FR-UR-04: Docusaurus pages translate dynamically on toggle without full page refresh where possible.
- FR-UR-05: Apply RTL stylesheet only when Urdu is selected; headers, paragraphs, sidebars, and tables should flip layout.
- FR-UR-06: Cache translations on the server (Redis or file-based) to reduce cost/latency.
- FR-UR-07: Save user language preference with Better Auth (persisted in user settings/local storage).

## Non-Functional Requirements

- NFR-UR-01: Translations must feel natural and human-like.
- NFR-UR-02: Page should update language under 1 second for typical pages.
- NFR-UR-03: No layout-breaking during RTL switch.
- NFR-UR-04: Chatbot should not mix Urdu and English unnecessarily.
- NFR-UR-05: Support mobile, tablet, and desktop layouts.

## Acceptance Criteria

- AC-UR-01: Clicking toggle switches site to Urdu instantly.
- AC-UR-02: Chatbot returns full Urdu responses when Urdu mode is on.
- AC-UR-03: Markdown formatting (headers, lists, code fences) remains intact after translation.
- AC-UR-04: RTL layout renders correctly with no broken UI elements.
- AC-UR-05: Switching back to English restores original content.
- AC-UR-06: Language preference persists across logins.

## Architecture Notes

- Prefer dynamic translation over storing a full set of localized pages; provide an option for human-curated edits of the translation cache.
- Use RAG and a Markdown-aware parser for accurate translation of code blocks, tables, and callouts.
- Client-side toggle will trigger fetch → translation → cache retrieval.
- RTL CSS should be injected only in Urdu mode; keep CSS isolated to avoid global breakage.
- Provide a dedicated translation microservice backing `/api/translate` and a cache layer (Redis) for performance.

## Integration Points

- Docusaurus (frontend)
- Chatbot backend
- Translation microservice
- Better Auth (user settings)
- Redis cache for translations

## Open Questions (<= 3 critical)

- [NEEDS CLARIFICATION: OQ-UR-01] Should PDF export support Urdu mode as part of scope? (PDF flows might need server-side rendering changes.)
- [NEEDS CLARIFICATION: OQ-UR-02] Should we label content as "Machine-translated" or similar to signal to learners that the translation is auto-generated?
- [NEEDS CLARIFICATION: OQ-UR-04] Should we support a hybrid mode (Docs English + Chatbot Urdu), or must the entire UI be in a single language at any time?

## Assumptions

- Students prefer Urdu for understanding while using English for code blocks.
- OpenAI translation API is available for runtime translation (models noted in Recommended Environment only).
- Docusaurus allows custom React components, which we will use to implement toggling and RTL injection.
- Internet-based translation; no offline mode.

## Risks

- Translation inaccuracies may cause misunderstanding of technical concepts.
- RTL styling may break some custom components; careful CSS isolation is required.
- The translation API cost may be high without caching strategies.

## Performance Requirements

- Page translations should be under 800ms for typical page blocks.
- Chatbot Urdu responses under 1.5 seconds.
- Cache hit ratio > 70% within 2 weeks of feature release.

## Security Requirements

- No user text stored without consent; logging should not persist user input beyond session unless requested.
- Rate-limit translation endpoints.
- Prevent misuse of the translation API for non-educational or abusive content. Provide guardrails for user-submitted text.

## Testing Requirements

- Unit tests for translation formatting integrity and Markdown structure invariants.
- UI tests for RTL layout switching and persistence.
- E2E tests for English → Urdu toggle, chatbot Urdu responses, and preference persistence.

## Recommended Environment (implementation details & guidance)

- Translation API: OpenAI models (e.g., GPT-4o or GPT-5 when available) for high-quality results; default to top available model at time of deployment.
- Add server-side caching via Redis and provide local file-based cache fallback for dev environments.
- Provide Docker images for reproducible dev environment using a pinned Jazzy/Node/Python environment for local testing.

## Out of Scope

- Complete manual translation of all chapters (automated translation only is in scope)
- Multi-language support beyond Urdu in this phase
- Offline translation engines or TTS in this phase

## Deliverables

- UI language toggle in navbar
- `/api/translate` translation microservice
- Docusaurus integration components & placeholders for translated content
- Translation cache management
- RTL stylesheet injection and tests
- Chatbot Urdu mode
- Persisted language preference via Better Auth

## Next Steps

- Resolve Open Questions (OQ-UR-01, OQ-UR-02, OQ-UR-04) before `/sp.plan`.
- Create `plan.md` and tasks for implementation once clarifications are resolved.

---

*Document prepared with inputs from the user and structured following the project `spec-template.md`. If any clarification is missing, we will ask at most 3 critical questions next.*
