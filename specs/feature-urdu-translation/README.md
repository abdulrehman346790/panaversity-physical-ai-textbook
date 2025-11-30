# Urdu Translation Feature

This feature adds dynamic Urdu translation for docs and chatbot. The repo contains a translation microservice skeleton and Docusaurus i18n configuration.

## Local development
- Start Redis or ensure `REDIS_URL` is available.
- Start translate service with:
```bash
cd services/translate
pip install -r requirements.txt
uvicorn translate_service:app --reload
```
- Start Docusaurus dev server in the `docusaurus/book` folder per the README there.

## Notes
- This is an initial skeleton for the translation feature; follow tasks in `specs/feature-urdu-translation/tasks.md` to progress.
