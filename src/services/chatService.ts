/**
 * API service layer for ChatBot feature
 * Handles communication with FastAPI RAG backend
 */

import type { QueryRequest, QueryResponse } from '../types/chat';

// Get API URL from Docusaurus config or environment variable
const API_BASE_URL = 'http://localhost:8000';


/**
 * Custom error class for API communication failures
 */
export class ChatServiceError extends Error {
  constructor(
    public type: 'network' | 'timeout' | 'server' | 'validation',
    message: string,
    public statusCode?: number
  ) {
    super(message);
    this.name = 'ChatServiceError';
  }
}

/**
 * Send query to RAG backend
 *
 * @param query - User's question
 * @param chapterId - Current chapter ID (e.g., "chapter-1")
 * @param sessionId - Optional session identifier
 * @returns Promise<QueryResponse> with answer and sources
 * @throws ChatServiceError on network, timeout, server, or validation errors
 */
export async function sendQuery(
  query: string,
  chapterId: string,
  sessionId?: string
): Promise<QueryResponse> {
  // Abort request after 30 seconds
  const controller = new AbortController();
  const timeoutId = setTimeout(() => controller.abort(), 30000);

  try {
    const response = await fetch(`${API_BASE_URL}/api/query`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({
        query: query.trim(),
        chapter_id: chapterId,
        session_id: sessionId,
      } as QueryRequest),
      signal: controller.signal,
    });

    clearTimeout(timeoutId);

    // Handle HTTP errors
    if (!response.ok) {
      if (response.status === 422) {
        throw new ChatServiceError(
          'validation',
          'Invalid question format. Please try rephrasing.',
          422
        );
      }
      if (response.status >= 500) {
        throw new ChatServiceError(
          'server',
          'The chatbot service is temporarily unavailable. Please try again later.',
          response.status
        );
      }
      // Other HTTP errors
      throw new ChatServiceError(
        'server',
        `Server returned error: ${response.status}`,
        response.status
      );
    }

    const data: QueryResponse = await response.json();
    return data;
  } catch (error: any) {
    clearTimeout(timeoutId);

    // Handle timeout
    if (error.name === 'AbortError') {
      throw new ChatServiceError('timeout', 'Request took too long. Please try again.');
    }

    // Re-throw ChatServiceError
    if (error instanceof ChatServiceError) {
      throw error;
    }

    // Network error
    throw new ChatServiceError(
      'network',
      'Unable to connect to the chatbot service. Please check your connection.'
    );
  }
}

/**
 * Get or create session ID from sessionStorage
 *
 * @returns Session ID (UUID)
 */
export function getOrCreateSessionId(): string {
  const STORAGE_KEY = 'chat_session_id';
  let sessionId = sessionStorage.getItem(STORAGE_KEY);

  if (!sessionId) {
    sessionId = crypto.randomUUID();
    sessionStorage.setItem(STORAGE_KEY, sessionId);
  }

  return sessionId;
}

/**
 * Extract chapter ID from current page URL
 *
 * @param pathname - Current page pathname (e.g., "/chapter-1/intro")
 * @returns Chapter ID (e.g., "chapter-1") or default "chapter-1"
 */
export function extractChapterId(pathname: string): string {
  const match = pathname.match(/\/chapter-(\d+)/);
  return match ? `chapter-${match[1]}` : 'chapter-1'; // Default fallback
}
