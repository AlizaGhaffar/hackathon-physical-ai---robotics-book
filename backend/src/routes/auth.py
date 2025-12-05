from fastapi import APIRouter, HTTPException, status, Depends
from ..models.auth import UserSignupRequest, UserLoginRequest, Token, UserResponse
from ..services.auth_service import hash_password, verify_password, create_access_token
from ..database import get_db_connection
from ..auth.middleware import get_current_user_from_db
from datetime import datetime
import psycopg

router = APIRouter(prefix="/api/auth", tags=["authentication"])

@router.post("/signup", response_model=Token, status_code=status.HTTP_201_CREATED)
async def signup(user_data: UserSignupRequest):
    """
    Create a new user account
    """
    conn = get_db_connection()
    cursor = conn.cursor()

    try:
        # Check if user already exists
        cursor.execute("SELECT id FROM users WHERE email = %s", (user_data.email,))
        existing_user = cursor.fetchone()

        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email already registered"
            )

        # Hash password
        password_hash = hash_password(user_data.password)

        # Insert new user
        cursor.execute("""
            INSERT INTO users (email, password_hash, name, software_level, hardware_level, learning_goals)
            VALUES (%s, %s, %s, %s, %s, %s)
            RETURNING id, email, name, software_level, hardware_level, learning_goals, created_at, updated_at
        """, (
            user_data.email,
            password_hash,
            user_data.name,
            user_data.software_level.value,
            user_data.hardware_level.value,
            user_data.learning_goals
        ))

        user = cursor.fetchone()
        conn.commit()

        # Create access token
        access_token = create_access_token(data={
            "sub": str(user["id"]),
            "email": user["email"]
        })

        user_response = UserResponse(
            id=str(user["id"]),
            email=user["email"],
            name=user["name"],
            software_level=user["software_level"],
            hardware_level=user["hardware_level"],
            learning_goals=user["learning_goals"],
            created_at=user["created_at"],
            updated_at=user["updated_at"]
        )

        return Token(access_token=access_token, user=user_response)

    except psycopg.Error as e:
        conn.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Database error: {str(e)}"
        )
    finally:
        cursor.close()
        conn.close()

@router.post("/login", response_model=Token)
async def login(credentials: UserLoginRequest):
    """
    Login with email and password
    """
    conn = get_db_connection()
    cursor = conn.cursor()

    try:
        # Get user by email
        cursor.execute("""
            SELECT id, email, password_hash, name, software_level, hardware_level, learning_goals, created_at, updated_at
            FROM users WHERE email = %s
        """, (credentials.email,))

        user = cursor.fetchone()

        if not user:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )

        # Verify password
        if not verify_password(credentials.password, user["password_hash"]):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid email or password"
            )

        # Create access token
        access_token = create_access_token(data={
            "sub": str(user["id"]),
            "email": user["email"]
        })

        user_response = UserResponse(
            id=str(user["id"]),
            email=user["email"],
            name=user["name"],
            software_level=user["software_level"],
            hardware_level=user["hardware_level"],
            learning_goals=user["learning_goals"],
            created_at=user["created_at"],
            updated_at=user["updated_at"]
        )

        return Token(access_token=access_token, user=user_response)

    finally:
        cursor.close()
        conn.close()

@router.get("/me", response_model=UserResponse)
async def get_current_user(user: dict = Depends(get_current_user_from_db)):
    """
    Get current authenticated user details
    """
    return UserResponse(
        id=str(user["id"]),
        email=user["email"],
        name=user["name"],
        software_level=user["software_level"],
        hardware_level=user["hardware_level"],
        learning_goals=user["learning_goals"],
        created_at=user["created_at"],
        updated_at=user["updated_at"]
    )
