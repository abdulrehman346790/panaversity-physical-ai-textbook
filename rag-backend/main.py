from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel
from agents import Runner
from agent import triage_agent, config
from ingest import ingest_docs
from fastapi.middleware.cors import CORSMiddleware
import asyncio

app = FastAPI(title="Physical AI Textbook Chatbot")

# CORS for Docusaurus frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify the Docusaurus URL
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class ChatRequest(BaseModel):
    message: str
    session_id: str = "default_session"

class ChatResponse(BaseModel):
    response: str

@app.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest):
    try:
        # Run the agent workflow
        result = await Runner.run(triage_agent, request.message, run_config=config)
        return ChatResponse(response=result.final_output)
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
    uvicorn.run(app, host="0.0.0.0", port=8001)
