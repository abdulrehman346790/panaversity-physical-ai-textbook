# Translation Rules — Markdown Preservation

This document enumerates rules used by the translation pipeline to preserve Markdown structure and technical tokens.

1. Code blocks (fenced code) must be extracted and stored before sending content to translation API. The translator receives only plain text without code blocks; the translated text is re-combined with the code blocks unchanged.
2. YAML frontmatter keys are not translated (e.g., `title`, `slug`, `description`). Only human-facing fields are translated.
3. Inline code spans (backticks `) are preserved and not translated.
4. Tables are converted to serializable form (CSV or Markdown table) and translated cell contents while keeping table structure intact.
5. Glossary terms (from `glossary/terms.yaml`) are not translated; they are replaced with their Urdu mapping or preserved as English if recommended.
6. ROS topics and names match a known token pattern (e.g., `/[a-z_0-9/]+`) and are left unchanged.
7. Emojis and special characters are preserved.

# Example Flow
- Input: Markdown
- Parse: Extract frontmatter + code blocks + YAML
- Replace: Mark tokens and placeholders
- Translate: Send to LLM a cleaned version for translation
- Recompose: Re-insert placeholders and tokens into translated text

# Verification
Unit tests verify that:
- Code fences are unchanged
- YAML frontmatter remains identical (keys and semantics)
- Glossary mappings are respected
- Tables remain valid and render the same in Docusaurus
