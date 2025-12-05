import React, { useState } from 'react';
import ChatWindow from './ChatWindow';
import InputBox from './InputBox';
import './ChatWidget.css';

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

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const handleSendMessage = async (message: string) => {
    if (!message.trim()) return;

    // Add user message to chat
    const userMessage: Message = {
      role: 'user',
      content: message,
    };
    
    setMessages(prev => [...prev, userMessage]);
    setIsLoading(true);

    try {
      const apiUrl = process.env.REACT_APP_API_URL || 'http://localhost:8000';
      
      // Get current chapter from URL
      const pathMatch = window.location.pathname.match(/chapter-(\d+)/);
      const currentChapter = pathMatch ? parseInt(pathMatch[1]) : 1;

      const response = await fetch(`${apiUrl}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: message,
          chapter: currentChapter,
          conversation_id: null,
          user_id: null,
          selected_text: null,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response from chatbot');
      }

      const data = await response.json();

      // Add assistant message to chat
      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      
      // Show error message
      const errorMessage: Message = {
        role: 'assistant',
        content: 'I apologize, but I encountered an error processing your question. Please try again.',
      };
      
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <>
      {/* Floating chat button */}
      {!isOpen && (
        <button className="chat-widget-button" onClick={toggleChat} aria-label="Open chat">
          💬
        </button>
      )}

      {/* Chat window */}
      {isOpen && (
        <div className="chat-widget-container">
          <div className="chat-widget-header">
            <h3>AI Textbook Assistant</h3>
            <button className="chat-widget-close" onClick={toggleChat} aria-label="Close chat">
              ✕
            </button>
          </div>
          
          <ChatWindow messages={messages} isLoading={isLoading} />
          
          <InputBox onSendMessage={handleSendMessage} disabled={isLoading} />
        </div>
      )}
    </>
  );
};

export default ChatWidget;
