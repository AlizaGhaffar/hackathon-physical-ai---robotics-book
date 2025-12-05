from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(
    title="Physical AI Textbook API",
    description="Backend for Chapter 1: ROS 2 Fundamentals",
    version="1.0.0"
)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://localhost:3001", "https://*.github.io", "https://*.vercel.app"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "Physical AI Textbook API - Chapter 1"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# Import routes
from .routes import auth_router, translate_router

# Include routers
app.include_router(auth_router)
app.include_router(translate_router)

# Chatbot endpoint (Simple - No RAG)
from pydantic import BaseModel
from typing import Optional, List, Dict
from .ai_client import chat_completion
from fastapi import Depends

class ChatRequest(BaseModel):
    message: str
    chapter: str = "chapter_1"
    use_rag: bool = True

class ChatResponse(BaseModel):
    response: str
    success: bool = True
    sources: Optional[List[Dict]] = None
    num_sources: Optional[int] = 0

@app.post("/api/chatbot/ask", response_model=ChatResponse)
async def chatbot_ask(request: ChatRequest):
    """
    Simple chatbot for answering questions about ROS 2 and Physical AI
    Powered by OpenAI GPT-3.5
    """
    try:
        # Detailed system context for Physical AI textbook
        system_context = """
        You are an expert AI assistant for a Physical AI and Humanoid Robotics textbook - Chapter 1: ROS 2 Fundamentals.

        About the Book:
        - Teaches Physical AI: AI systems that understand and interact with the physical world
        - Focuses on humanoid robots that can operate in human environments
        - Core technology: ROS 2 (Robot Operating System 2)

        ROS 2 Core Concepts:
        - Nodes: Independent processes that perform computation
        - Topics: Named buses for message passing (pub/sub pattern)
        - Services: Request/reply interactions between nodes
        - Actions: Long-running tasks with feedback
        - Parameters: Configuration values for nodes

        Key Topics in Chapter 1:
        1. What is ROS 2? Middleware for robot control and communication
        2. Why ROS 2? Industry standard, used by Boston Dynamics, NASA, Waymo
        3. Nodes: Building blocks of ROS 2 applications
        4. Topics: Asynchronous message passing for sensor data
        5. Services: Synchronous request-reply for commands
        6. URDF: Unified Robot Description Format for humanoid models

        Your Role:
        - Answer questions concisely and clearly
        - Provide Python code examples when relevant
        - Explain robotics concepts in simple terms
        - Focus on practical implementation
        - Help students understand Physical AI fundamentals

        Be helpful, encouraging, and focus on real-world robotics applications!
        """

        response_text = chat_completion(request.message, system_context)

        return ChatResponse(
            response=response_text,
            success=True,
            sources=[],
            num_sources=0
        )
    except Exception as e:
        return ChatResponse(
            response=f"Error: {str(e)}. Please check if OpenAI API key is configured.",
            success=False,
            sources=[],
            num_sources=0
        )

@app.post("/api/chatbot/personalized", response_model=ChatResponse)
async def chatbot_personalized(request: ChatRequest):
    """
    Personalized chatbot (simplified - same as regular for now)
    """
    # For now, just use the regular chatbot
    return await chatbot_ask(request)
