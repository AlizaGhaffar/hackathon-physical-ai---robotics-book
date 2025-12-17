"""Rate limiting middleware for FastAPI."""
from fastapi import Request, HTTPException, status
from starlette.middleware.base import BaseHTTPMiddleware
from collections import defaultdict
from datetime import datetime, timedelta
import asyncio


class RateLimiterMiddleware(BaseHTTPMiddleware):
    """
    Simple in-memory rate limiter.
    Limits requests per IP address.

    For production, use Redis-based rate limiting.
    """

    def __init__(self, app, requests_per_minute: int = 100):
        super().__init__(app)
        self.requests_per_minute = requests_per_minute
        self.request_counts = defaultdict(list)
        self.cleanup_task = None

    async def dispatch(self, request: Request, call_next):
        """Process request with rate limiting."""

        # Skip rate limiting for health checks
        if request.url.path == "/api/health":
            return await call_next(request)

        # Get client IP
        client_ip = request.client.host

        # Clean old entries
        now = datetime.now()
        cutoff = now - timedelta(minutes=1)
        self.request_counts[client_ip] = [
            req_time for req_time in self.request_counts[client_ip]
            if req_time > cutoff
        ]

        # Check rate limit
        if len(self.request_counts[client_ip]) >= self.requests_per_minute:
            raise HTTPException(
                status_code=status.HTTP_429_TOO_MANY_REQUESTS,
                detail=f"Rate limit exceeded. Maximum {self.requests_per_minute} requests per minute.",
            )

        # Record this request
        self.request_counts[client_ip].append(now)

        # Process request
        response = await call_next(request)

        # Add rate limit headers
        remaining = self.requests_per_minute - len(self.request_counts[client_ip])
        response.headers["X-RateLimit-Limit"] = str(self.requests_per_minute)
        response.headers["X-RateLimit-Remaining"] = str(max(0, remaining))
        response.headers["X-RateLimit-Reset"] = str(int((now + timedelta(minutes=1)).timestamp()))

        return response
