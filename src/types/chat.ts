/**
 * TypeScript type definitions for ChatBot feature
 * These types match the backend Pydantic models exactly
 */

// ===== Backend API Contract Types =====

/**
 * Request payload for /api/query endpoint
 * Matches backend QueryRequest Pydantic model
 */
export interface QueryRequest {
  query: string;
  chapter_id: string;
  session_id?: string;
  selected_text?: string;
}

/**
 * Retrieved document chunk from vector database
 * Matches backend RetrievedChunk Pydantic model
 */
export interface RetrievedChunk {
  chunk_id: string;
  content: string;
  score: number;
  source_file: string;
  chapter_id: string;
}

/**
 * Token usage statistics from OpenAI API
 */
export interface TokenUsage {
  prompt: number;
  completion: number;
  total: number;
}

/**
 * Response from /api/query endpoint
 * Matches backend QueryResponse Pydantic model
 */
export interface QueryResponse {
  request_id: string;
  answer: string;
  session_id: string;
  sources: RetrievedChunk[];
  token_usage?: TokenUsage;
  response_time_ms?: number;
}

// ===== Frontend-Specific Types =====

/**
 * Client-side chat message representation
 */
export interface Message {
  id: string;                    // UUID for React key
  role: 'user' | 'assistant';    // Message sender
  content: string;               // Message text content
  timestamp: Date;               // When message was created
  sources?: RetrievedChunk[];    // Source citations (assistant only)
  isError?: boolean;             // Error message flag
}

/**
 * Chat session state
 */
export interface ChatSession {
  sessionId: string;             // Persistent session ID from sessionStorage
  chapterId: string;             // Current chapter context from URL
  messages: Message[];           // Conversation history
  isLoading: boolean;            // Waiting for backend response
}
