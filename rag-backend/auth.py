from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel, EmailStr
from typing import Optional, List
import bcrypt
import jwt
from datetime import datetime, timedelta
from sqlalchemy import text
from database import get_db
import os

router = APIRouter()

# JWT Configuration
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-this")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24 * 7  # 7 days

# Pydantic Models
class UserSignup(BaseModel):
    email: EmailStr
    password: str
    name: str
    
class UserProfile(BaseModel):
    software_experience: str  # 'beginner', 'intermediate', 'advanced'
    programming_languages: List[str]
    has_ros_experience: bool
    hardware_experience: str  # 'none', 'basic', 'intermediate', 'advanced'
    hardware_description: Optional[str] = ""
    has_robotics_hardware: bool
    learning_goal: str  # 'career_change', 'hobby', 'research', 'academic', 'professional'
    learning_goals_description: Optional[str] = ""

class UserSignupComplete(BaseModel):
    email: EmailStr
    password: str
    name: str
    profile: UserProfile

class UserLogin(BaseModel):
    email: EmailStr
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str
    user: dict

# Helper Functions
def hash_password(password: str) -> str:
    """Hash password using bcrypt"""
    salt = bcrypt.gensalt()
    return bcrypt.hashpw(password.encode('utf-8'), salt).decode('utf-8')

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify password against hash"""
    return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))

def create_access_token(data: dict) -> str:
    """Create JWT access token"""
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str) -> dict:
    """Verify JWT token"""
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token has expired")
    except jwt.JWTError:
        raise HTTPException(status_code=401, detail="Invalid token")

# Routes
@router.post("/auth/signup", response_model=Token)
async def signup(user_data: UserSignupComplete, db = Depends(get_db)):
    """
    Complete signup with user credentials and profile information
    """
    try:
        # Check if user already exists
        result = db.execute(
            text("SELECT id FROM users WHERE email = :email"),
            {"email": user_data.email}
        )
        if result.fetchone():
            raise HTTPException(status_code=400, detail="Email already registered")
        
        # Hash password
        password_hash = hash_password(user_data.password)
        
        # Insert user
        result = db.execute(
            text("""
                INSERT INTO users (email, password_hash, name)
                VALUES (:email, :password_hash, :name)
                RETURNING id, email, name, created_at
            """),
            {
                "email": user_data.email,
                "password_hash": password_hash,
                "name": user_data.name
            }
        )
        user = result.fetchone()
        user_id = user[0]
        
        # Insert user profile
        db.execute(
            text("""
                INSERT INTO user_profiles (
                    user_id, software_experience, programming_languages,
                    has_ros_experience, hardware_experience, hardware_description,
                    has_robotics_hardware, learning_goal, learning_goals_description,
                    preferred_difficulty, show_beginner_notes, show_advanced_tips
                )
                VALUES (
                    :user_id, :software_experience, :programming_languages,
                    :has_ros_experience, :hardware_experience, :hardware_description,
                    :has_robotics_hardware, :learning_goal, :learning_goals_description,
                    :preferred_difficulty, :show_beginner_notes, :show_advanced_tips
                )
            """),
            {
                "user_id": user_id,
                "software_experience": user_data.profile.software_experience,
                "programming_languages": user_data.profile.programming_languages,
                "has_ros_experience": user_data.profile.has_ros_experience,
                "hardware_experience": user_data.profile.hardware_experience,
                "hardware_description": user_data.profile.hardware_description,
                "has_robotics_hardware": user_data.profile.has_robotics_hardware,
                "learning_goal": user_data.profile.learning_goal,
                "learning_goals_description": user_data.profile.learning_goals_description,
                "preferred_difficulty": user_data.profile.software_experience,
                "show_beginner_notes": user_data.profile.software_experience == 'beginner',
                "show_advanced_tips": user_data.profile.software_experience == 'advanced'
            }
        )
        
        db.commit()
        
        # Create access token
        access_token = create_access_token({"sub": str(user_id), "email": user_data.email})
        
        return {
            "access_token": access_token,
            "token_type": "bearer",
            "user": {
                "id": str(user_id),
                "email": user[1],
                "name": user[2]
            }
        }
        
    except Exception as e:
        db.rollback()
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/auth/signin", response_model=Token)
async def signin(credentials: UserLogin, db = Depends(get_db)):
    """
    Sign in with email and password
    """
    try:
        # Get user
        result = db.execute(
            text("SELECT id, email, password_hash, name FROM users WHERE email = :email"),
            {"email": credentials.email}
        )
        user = result.fetchone()
        
        if not user:
            raise HTTPException(status_code=401, detail="Invalid credentials")
        
        # Verify password
        if not verify_password(credentials.password, user[2]):
            raise HTTPException(status_code=401, detail="Invalid credentials")
        
        # Create access token
        access_token = create_access_token({"sub": str(user[0]), "email": user[1]})
        
        return {
            "access_token": access_token,
            "token_type": "bearer",
            "user": {
                "id": str(user[0]),
                "email": user[1],
                "name": user[3]
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/auth/profile")
async def get_profile(token: str, db = Depends(get_db)):
    """
    Get user profile information
    """
    try:
        # Verify token
        payload = verify_token(token)
        user_id = payload.get("sub")
        
        # Get profile
        result = db.execute(
            text("""
                SELECT 
                    u.id, u.email, u.name,
                    p.software_experience, p.programming_languages,
                    p.has_ros_experience, p.hardware_experience,
                    p.hardware_description, p.has_robotics_hardware,
                    p.learning_goal, p.learning_goals_description,
                    p.preferred_difficulty, p.show_beginner_notes,
                    p.show_advanced_tips
                FROM users u
                LEFT JOIN user_profiles p ON u.id = p.user_id
                WHERE u.id = :user_id
            """),
            {"user_id": user_id}
        )
        profile = result.fetchone()
        
        if not profile:
            raise HTTPException(status_code=404, detail="User not found")
        
        return {
            "user": {
                "id": str(profile[0]),
                "email": profile[1],
                "name": profile[2]
            },
            "profile": {
                "software_experience": profile[3],
                "programming_languages": profile[4],
                "has_ros_experience": profile[5],
                "hardware_experience": profile[6],
                "hardware_description": profile[7],
                "has_robotics_hardware": profile[8],
                "learning_goal": profile[9],
                "learning_goals_description": profile[10],
                "preferred_difficulty": profile[11],
                "show_beginner_notes": profile[12],
                "show_advanced_tips": profile[13]
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
