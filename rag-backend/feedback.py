from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional
import sqlite3
import uuid
from datetime import datetime
import os

router = APIRouter()

# Simple SQLite setup for feedback
DB_PATH = os.path.join(os.path.dirname(__file__), "feedback.db")

def init_db():
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    c.execute('''CREATE TABLE IF NOT EXISTS feedback
                 (id TEXT PRIMARY KEY, 
                  user_id TEXT, 
                  source_path TEXT, 
                  original_text TEXT, 
                  translated_text TEXT, 
                  comments TEXT, 
                  rating INTEGER, 
                  timestamp TEXT)''')
    conn.commit()
    conn.close()

# Initialize on module load
init_db()

class FeedbackRequest(BaseModel):
    user_id: Optional[str] = "anon"
    source_path: str
    original_text: Optional[str] = None
    translated_text: Optional[str] = None
    comments: str
    rating: Optional[int] = None

@router.post("/translate/feedback")
async def submit_feedback(feedback: FeedbackRequest):
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        feedback_id = str(uuid.uuid4())
        timestamp = datetime.now().isoformat()
        
        c.execute("INSERT INTO feedback VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                  (feedback_id, feedback.user_id, feedback.source_path, 
                   feedback.original_text, feedback.translated_text, 
                   feedback.comments, feedback.rating, timestamp))
        
        conn.commit()
        conn.close()
        
        return {"status": "success", "id": feedback_id}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to save feedback: {str(e)}")

@router.get("/translate/feedback/stats")
async def get_feedback_stats():
    try:
        conn = sqlite3.connect(DB_PATH)
        c = conn.cursor()
        c.execute("SELECT COUNT(*) FROM feedback")
        total = c.fetchone()[0]
        conn.close()
        return {"total_feedback": total}
    except Exception as e:
        return {"error": str(e)}
