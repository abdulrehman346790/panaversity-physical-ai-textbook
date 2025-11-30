from dotenv import load_dotenv
import os

# Force reload from .env
load_dotenv(override=True)

openai_key = os.getenv("OPENAI_API_KEY")
qdrant_key = os.getenv("QDRANT_API_KEY")
qdrant_url = os.getenv("QDRANT_URL")
neon_url = os.getenv("NEON_DATABASE_URL")

print(f"OPENAI_API_KEY: {'[SET]' if openai_key and not openai_key.startswith('sk-...') else '[MISSING/PLACEHOLDER]'}")
print(f"QDRANT_API_KEY: {'[SET]' if qdrant_key else '[MISSING]'}")
print(f"QDRANT_URL:     {'[SET]' if qdrant_url else '[MISSING]'}")
print(f"NEON_DB_URL:    {'[SET]' if neon_url else '[MISSING]'}")

if openai_key and openai_key.startswith("sk-..."):
    print("\nWARNING: OPENAI_API_KEY appears to be a placeholder!")
