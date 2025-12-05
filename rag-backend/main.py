from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import Optional
from agents import Runner
from agent import triage_agent, config
from ingest import ingest_docs
from auth import router as auth_router
from personalization import router as personalization_router, get_personalization_context
from database import get_db
from agents import SQLiteSession
import asyncio
import os

app = FastAPI(title="Physical AI Textbook Chatbot")

# Configure CORS - allow both localhost and production URLs
allowed_origins = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")
app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include Auth Router
app.include_router(auth_router, prefix="/api")
app.include_router(personalization_router, prefix="/api")

# Include Translation Router
from translation import router as translation_router, translate_markdown
from feedback import router as feedback_router
app.include_router(translation_router, prefix="/api")
app.include_router(feedback_router, prefix="/api")


class ChatRequest(BaseModel):
    message: str
    session_id: str = "default_session"
    token: Optional[str] = None  # Optional user token for personalization

class ChatResponse(BaseModel):
    response: str
    translated: bool = False
    source_lang: Optional[str] = None

@app.get("/")
async def root():
    """Health check endpoint for deployment platforms"""
    return {"status": "ok", "message": "Physical AI Textbook API is running"}

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest, db = Depends(get_db)):
    try:
        # Get user personalization context if token provided
        personalization_context = ""
        if request.token:
            try:
                from personalization import get_user_personalization
                profile = await get_user_personalization(request.token, db)
                personalization_context = get_personalization_context(profile)
            except:
                pass  # Continue without personalization if token invalid
        
        # Prepend personalization context to message
        enhanced_message = request.message
        if personalization_context:
            enhanced_message = f"[USER CONTEXT: {personalization_context}]\n\nUser Question: {request.message}"
        
        # Run the agent workflow
        result = await Runner.run(triage_agent, enhanced_message, run_config=config)

        # Post-process: consult user language pref and translate if necessary
        user_lang = 'en'
        if request.token:
            try:
                from personalization import get_user_language
                pref = await get_user_language(request.token)
                user_lang = pref.get('language', 'en')
            except Exception:
                user_lang = 'en'

        final_text = result.final_output
        is_translated = False
        if user_lang == 'ur' and final_text:
            try:
                # Direct call to translation logic
                final_text = translate_markdown(final_text, 'ur')
                is_translated = True
            except Exception as e:
                # If translation fails, fallback to original final_text
                print('Translation error:', e)

        return ChatResponse(response=final_text, translated=is_translated, source_lang='en' if is_translated else None)
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"Error: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/ingest")
async def trigger_ingestion():
    try:
        # Run ingestion in a separate thread/process in production
        # For hackathon, running synchronously is acceptable but might timeout
        ingest_docs()
        return {"status": "Ingestion complete"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

if __name__ == "__main__":
    import uvicorn
    # Hugging Face Spaces defaults to port 7860
    port = int(os.getenv("PORT", 7860))
    uvicorn.run(app, host="0.0.0.0", port=port)
