# Research Notes — Urdu Translation Feature

## Purpose
Gather and document decisions for the Urdu translation feature: PDF export, labeling strategy for machine translation, hybrid mode approach, and general technical choices.

## Decisions & Rationale

### OQ-UR-01: PDF Export Support
- Decision: **No** — PDF export for Urdu is out of scope for Phase 1 (deferred to later milestone).
- Rationale: Server-side PDF rendering for localized pages requires additional infra, and many users consult the live site. This keeps initial scope achievable.
- Alternatives considered: Support from start (higher cost), or provide minimal generated PDFs as a later improvement.

### OQ-UR-02: Machine-translated Labeling
- Decision: **Use a subtle indicator with feedback link (Option C).**
- Rationale: Transparency is valuable in educational contexts; a small label and a feedback option balances clarity and usability. We'll show a small icon or label (e.g., "Machine translated — report issue").

### OQ-UR-04: Hybrid Mode — Chatbot vs Docs
- Decision: **Default to single-language mode (English OR Urdu), allow Hybrid as an advanced toggle (Option C).**
- Rationale: Most learners will prefer consistent UI, but hybrid mode provides flexibility for advanced users — e.g., keep docs English but use chatbot Urdu. Save this in user profile preferences.

## Tech Choices & Best Practices
- Use OpenAI GPT-4o/5 models for translation (configurable via Recommended Environment); pipeline preserves Markdown and code fences using a parser/HTML-aware translation approach (RAG/PTR for code blocks).
- Use Redis as primary caching layer; fallback to file-based cache for dev environments.
- Use slug and metadata mapping for SEO; when a translated page is generated, attach meta fields `og:locale: ur_PK` etc.

## Alternatives & Trade-offs
- On-demand translation (no persisted translations) reduces storage but increases cost and latency; we prefer a cached-on-demand approach.
- Full manual translation offers highest quality but is beyond initial scope.

## Acceptance
- The research file resolves OQ-UR-01, OQ-UR-02, and OQ-UR-04.
- Next: generate `plan.md` and `tasks.md` entries and data modeling for the translation system.
