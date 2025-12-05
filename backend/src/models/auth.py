from pydantic import BaseModel, EmailStr, Field
from typing import Optional
from datetime import datetime
from enum import Enum

class SkillLevel(str, Enum):
    """User skill level options"""
    BEGINNER = "Beginner"
    INTERMEDIATE = "Intermediate"
    ADVANCED = "Advanced"

class UserSignupRequest(BaseModel):
    """User signup request schema"""
    email: EmailStr
    password: str = Field(min_length=8, description="Password must be at least 8 characters")
    name: str = Field(min_length=2, max_length=100)
    software_level: SkillLevel
    hardware_level: SkillLevel
    learning_goals: Optional[str] = None

class UserLoginRequest(BaseModel):
    """User login request schema"""
    email: EmailStr
    password: str

class UserResponse(BaseModel):
    """User response schema (without password)"""
    id: str
    email: str
    name: str
    software_level: str
    hardware_level: str
    learning_goals: Optional[str]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True

class Token(BaseModel):
    """JWT token response"""
    access_token: str
    token_type: str = "bearer"
    user: UserResponse

class TokenData(BaseModel):
    """Data stored in JWT token"""
    user_id: str
    email: str
