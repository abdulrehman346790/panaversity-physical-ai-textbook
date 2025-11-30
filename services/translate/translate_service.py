from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional
import os
from translate_pipeline import translate_markdown

app = FastAPI(title="Panaversity Translate Service")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class TranslateRequest(BaseModel):
    markdown: str
    target_locale: str = "ur"
    source_path: Optional[str] = None
    model: Optional[str] = None

class TranslateResponse(BaseModel):
    translated_markdown: str
    translated_html: Optional[str] = None
    content_hash: Optional[str] = None
    cached: bool = False

@app.post("/api/translate", response_model=TranslateResponse)
async def translate(req: TranslateRequest):
    # Placeholder translation: echo the input; In a real implementation,
    # call OpenAI API and perform Markdown-aware transformations
    if not req.markdown:
        raise HTTPException(status_code=400, detail="Missing markdown content")

    # Mode switch: mock or live
    mode = os.getenv('TRANSLATE_MODE', 'mock')
    if mode == 'mock':
        # return input as "translated" for testing, and indicate cached=False
        return TranslateResponse(translated_markdown=req.markdown, translated_html=None, cached=False)

    # For real mode, use the translate_markdown pipeline to preserve code blocks
    try:
        translated = translate_markdown(req.markdown, lambda s: s)  # placeholder: use a live translator here
        return TranslateResponse(translated_markdown=translated, translated_html=None, cached=False)
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation pipeline error: {str(e)}")
@app.post('/api/translate/feedback')
async def feedback(user_id: Optional[str] = None, source_path: Optional[str] = None, comments: Optional[str] = None):
    # Simple feedback receiver. Persist to a file or DB for review in real impl.
    try:
        # Append to in-memory log for now or file
        with open('/tmp/translate_feedback.log', 'a', encoding='utf-8') as fh:
            fh.write(f"USER:{user_id} PATH:{source_path} COMMENTS:{comments}\n")
        return {"status": "ok", "message": "feedback received"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/api/health")
async def health():
    return {"status": "ok"}

class PreferenceRequest(BaseModel):
    user_id: Optional[str] = None
    language: str

@app.post("/api/user/preference")
async def set_user_preference(pref: PreferenceRequest):
    # For now, echo back preference. Real implementation saves to DB.
    return {"user_id": pref.user_id, "language": pref.language}

@app.get("/api/user/preference")
async def get_user_preference(user_id: str):
    # Return default English; in real impl, read from DB
    return {"user_id": user_id, "language": "en"}
