from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy import text
from database import get_db
from auth import verify_token

router = APIRouter()

# Simple in-memory language preference store for MVP
LANG_PREFS = {}

@router.get("/personalization/profile")
async def get_user_personalization(token: str, db = Depends(get_db)):
    """
    Get user's personalization profile for content customization
    """
    try:
        # Verify token
        payload = verify_token(token)
        user_id = payload.get("sub")
        
        # Get profile
        result = db.execute(
            text("""
                SELECT 
                    software_experience, programming_languages,
                    has_ros_experience, hardware_experience,
                    preferred_difficulty, show_beginner_notes,
                    show_advanced_tips
                FROM user_profiles
                WHERE user_id = :user_id
            """),
            {"user_id": user_id}
        )
        profile = result.fetchone()
        
        if not profile:
            return {
                "personalized": False,
                "message": "No profile found"
            }
        
        return {
            "personalized": True,
            "software_experience": profile[0],
            "programming_languages": profile[1],
            "has_ros_experience": profile[2],
            "hardware_experience": profile[3],
            "preferred_difficulty": profile[4],
            "show_beginner_notes": profile[5],
            "show_advanced_tips": profile[6]
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

def get_personalization_context(user_profile: dict) -> str:
    """
    Generate context string for AI based on user profile
    """
    if not user_profile or not user_profile.get("personalized"):
        return ""
    
    context_parts = []
    
    # Software experience
    if user_profile.get("software_experience") == "beginner":
        context_parts.append("The user is a beginner in programming. Explain concepts in simple terms with examples.")
    elif user_profile.get("software_experience") == "advanced":
        context_parts.append("The user is an advanced programmer. You can use technical terminology and skip basic explanations.")
    
    # Programming languages
    langs = user_profile.get("programming_languages", [])
    if langs:
        context_parts.append(f"The user knows: {', '.join(langs)}. Use examples in these languages when possible.")
    
    # ROS experience
    if user_profile.get("has_ros_experience"):
        context_parts.append("The user has ROS/ROS 2 experience. You can reference ROS concepts directly.")
    else:
        context_parts.append("The user is new to ROS. Explain ROS concepts from scratch.")
    
    # Hardware experience
    hw_exp = user_profile.get("hardware_experience", "none")
    if hw_exp == "none":
        context_parts.append("The user has no robotics hardware experience. Focus on simulation and theory.")
    elif hw_exp in ["intermediate", "advanced"]:
        context_parts.append("The user has hands-on robotics experience. You can discuss hardware integration.")
    
    return "\n".join(context_parts)


@router.post('/user/lang')
async def set_user_language(token: str, language: str):
    """Set language for user (in-memory for MVP)"""
    try:
        payload = verify_token(token)
        user_id = payload.get('sub')
        LANG_PREFS[user_id] = language
        return {'user_id': user_id, 'language': language}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get('/user/lang')
async def get_user_language(token: str):
    """Return user's language preference"""
    try:
        payload = verify_token(token)
        user_id = payload.get('sub')
        return {'user_id': user_id, 'language': LANG_PREFS.get(user_id, 'en')}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
