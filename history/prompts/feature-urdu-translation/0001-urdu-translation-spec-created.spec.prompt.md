---
name: urdu-translation-spec-created
agent: agent
description: "Spec created for Urdu translation across Docs, Chatbot, and UI. Includes clarifications for PDF, machine-translated labelling, and hybrid mode."
model: "Raptor mini (Preview)"
tools: []
---

## Prompt

Create a feature specification for Urdu support across the platform (Docusaurus, Chatbot, UI), with a toggle, translation microservice, caching, and RTL support. Include three critical clarifications: OQ-UR-01 (PDF export), OQ-UR-02 (machine-translated label), and OQ-UR-04 (hybrid mode).

## Response snapshot

Created a spec with goals, functional/non-functional requirements, acceptance criteria, architecture notes, and three critical clarifications (OQ-UR-01, OQ-UR-02, OQ-UR-04) to resolve before proceeding to plan.

## Outcome

- ✅ Impact: Adds Urdu accessibility across the platform.
- 🧪 Tests: Checklist created; three critical clarifications pending.
- 📁 Files: spec.md, requirements.md
- 🔁 Next prompts: `/sp.plan` after clarifications; `/sp.tasks` to create actionable tasks.
- 🧠 Reflection: Recommend using caching and periodic manual audits for cached translations to improve quality.

## Evaluation notes
- Failure modes observed: No major failures; clarifications required.
