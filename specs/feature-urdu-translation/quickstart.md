# Quickstart — Urdu Translation Feature (Dev)

## Prerequisites
- Node 18+ and npm (for Docusaurus) or equivalent.
- Docker for reproducible environment.
- OpenAI API key (stored in secrets) for translation microservice.
- Redis running for caching translations (or local fallback).

## Developer Quickstart (Docker)
1. Build the example Docker image (this includes a dev environment for Jazzy + Node):
```bash
docker build -t panaversity-urdu-dev -f specs/feature-urdu-translation/Dockerfile .
```
2. Start Docker container and services (redis):
```bash
docker run --rm -it --network host -e OPENAI_API_KEY=$OPENAI_KEY panaversity-urdu-dev /bin/bash
```
3. Inside container, run:
```bash
# install node deps and build docs site
cd docusaurus
npm ci
npm run start
```
4. Start translation microservice (example):
```bash
cd services/translate
pip install -r requirements.txt
export OPENAI_API_KEY="<key>"
python translate_service.py
```

## Test translation quickly
Send a request to the translation endpoint with some Markdown:
```bash
curl -X POST https://localhost/api/translate -H "Content-Type: application/json" -d '{"markdown":"# Hello\n\nThis is a test.","target_locale":"ur"}'
```

## Notes
- The `translate` API will return both translated Markdown and a rendered HTML variant for the page. The Docusaurus plugin will consume the Markdown and render client-side or server-side translations.
- A subtle label (UI) is displayed on machine-translated pages, linking to a feedback PR or translation-fix form.
- Hybrid mode is supported and stored in user preferences.
