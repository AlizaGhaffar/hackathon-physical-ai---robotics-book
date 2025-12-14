"""Global error handling middleware for FastAPI."""
from fastapi import FastAPI, Request, status
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException
import traceback
import uuid


def add_error_handlers(app: FastAPI):
    """Add global error handlers to FastAPI app."""

    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        """Handle HTTP exceptions."""
        request_id = str(uuid.uuid4())[:8]
        return JSONResponse(
            status_code=exc.status_code,
            content={
                "error": f"HTTP {exc.status_code}",
                "message": exc.detail,
                "request_id": f"req_{request_id}",
            },
        )

    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        """Handle request validation errors."""
        request_id = str(uuid.uuid4())[:8]
        return JSONResponse(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            content={
                "error": "Validation Error",
                "message": "Invalid request parameters",
                "details": exc.errors(),
                "request_id": f"req_{request_id}",
            },
        )

    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        """Handle all other exceptions."""
        request_id = str(uuid.uuid4())[:8]

        # Log the error (in production, use proper logging)
        print(f"❌ Error [{request_id}]: {str(exc)}")
        print(traceback.format_exc())

        return JSONResponse(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            content={
                "error": "Internal Server Error",
                "message": "An unexpected error occurred. Please try again later.",
                "request_id": f"req_{request_id}",
            },
        )
