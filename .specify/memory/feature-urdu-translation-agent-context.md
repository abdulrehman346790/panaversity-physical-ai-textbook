# Agent Context — feature-urdu-translation

Context additions for AI agents responsible for translation, content QA, and PR generation:

- SELinux/OS-level constraints for file writes are out of scope here but the agent should try to use repo-friendly paths.
- The agent should use the `translate-openapi.yaml` contract for generating translation API calls.
- The agent should respect rate-limits and cache decisions when possible.
