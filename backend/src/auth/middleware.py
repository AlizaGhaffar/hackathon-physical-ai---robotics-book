from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
from ..services.auth_service import decode_access_token
from ..models.auth import TokenData
from ..database import get_db_connection

security = HTTPBearer()

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> TokenData:
    """
    Dependency to get current authenticated user from JWT token
    """
    token = credentials.credentials

    token_data = decode_access_token(token)

    if token_data is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Invalid authentication credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )

    return token_data

async def get_current_user_from_db(
    token_data: TokenData = Depends(get_current_user)
) -> dict:
    """
    Get full user details from database
    """
    conn = get_db_connection()
    cursor = conn.cursor()

    cursor.execute(
        "SELECT id, email, name, software_level, hardware_level, learning_goals, created_at, updated_at FROM users WHERE id = %s",
        (token_data.user_id,)
    )

    user = cursor.fetchone()
    cursor.close()
    conn.close()

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="User not found"
        )

    return user
