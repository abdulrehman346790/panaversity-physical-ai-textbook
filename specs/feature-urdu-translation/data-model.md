# Data Model — Urdu Translation Feature

## Key Entities

### TranslationRecord
- id: UUID
- source_path: string (absolute repo path or doc path)
- locale: string (e.g., 'ur')
- content_hash: string (SHA-256 or similar of source content)
- translated_html: text or pointer to storage
- translated_markdown: text (optional)
- status: enum (pending, completed, reviewed, rejected)
- created_at: timestamp
- updated_at: timestamp
- model_used: string (e.g., gpt-4o)

### TranslationGlossaryEntry
- id: UUID
- term_en: string
- term_ur: string
- context: string (e.g., 'ROS topic', 'python code comment')
- created_by: user_id or system

### UserPreference
- user_id: identifier from Better Auth
- language: 'en' | 'ur' | 'hybrid-UR-chatbot'
- notify_on_translation_review: boolean
- last_update: timestamp

### TranslationJob
- job_id: UUID
- source_path: string
- requested_by: user_id or system
- status: pending | in_progress | completed | failed
- created_at, updated_at

## Relationships
- TranslationRecord.source_path references an existing documentation path.
- UserPreference belongs to a Better Auth user profile.
- TranslationGlossaryEntry can be referenced in TranslationRecord decisions.

## Validation Rules
- `content_hash` must match the current source content before reuse from cache.
- Glossary terms should be protected from automatic translation (i.e., code blocks or glossary map).
- Translated content must preserve code fences and do not alter YAML frontmatter.

## Storage Choices
- Translated content saved in Redis for short-term and persisted to s3 or repo for long-term (depending on policy).
- Glossary and user preferences stored in DB (Better Auth or existing DB).

## Notes
- Minimal storage footprint desirable; cache eviction policy should be configured.
