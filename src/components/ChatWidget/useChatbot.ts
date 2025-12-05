import { useState, useCallback } from 'react';

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Array<{
    chapter: number;
    section: string;
    chunk_id: string;
    similarity_score: number;
  }>;
}

interface UseChatbotReturn {
  messages: Message[];
  isLoading: boolean;
  error: string | null;
  sendMessage: (question: string, chapter: number) => Promise<void>;
  clearMessages: () => void;
}

export const useChatbot = (): UseChatbotReturn => {
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendMessage = useCallback(async (question: string, chapter: number) => {
    if (!question.trim()) return;

    setError(null);
    
    // Add user message
    const userMessage: Message = {
      role: 'user',
      content: question,
    };
    
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      // Use browser's window object for runtime env vars in Docusaurus
      const apiUrl = (typeof window !== 'undefined' && (window as any).ENV?.API_URL) ||
                     process.env.NEXT_PUBLIC_API_URL ||
                     'http://localhost:8000';

      const response = await fetch(`${apiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question,
          chapter,
          conversation_id: null,
          user_id: null,
          selected_text: null,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({}));
        throw new Error(errorData.detail || 'Failed to get response from chatbot');
      }

      const data = await response.json();

      // Add assistant message
      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (err) {
      const errorMsg = err instanceof Error ? err.message : 'An error occurred';
      setError(errorMsg);
      
      // Show error as assistant message
      const errorMessage: Message = {
        role: 'assistant',
        content: `I apologize, but I encountered an error: ${errorMsg}. Please try again.`,
      };
      
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  }, []);

  const clearMessages = useCallback(() => {
    setMessages([]);
    setError(null);
  }, []);

  return {
    messages,
    isLoading,
    error,
    sendMessage,
    clearMessages,
  };
};
