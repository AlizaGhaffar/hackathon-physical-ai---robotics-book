import React, { useEffect, useRef } from 'react';
import Message from './Message';

interface ChatWindowProps {
  messages: Array<{
    role: 'user' | 'assistant';
    content: string;
    citations?: Array<{
      chapter: number;
      section: string;
      chunk_id: string;
      similarity_score: number;
    }>;
  }>;
  isLoading: boolean;
}

const ChatWindow: React.FC<ChatWindowProps> = ({ messages, isLoading }) => {
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [messages]);

  return (
    <div className="chat-window">
      {messages.length === 0 ? (
        <div className="chat-empty-state">
          <p>👋 Hello! I'm your AI textbook assistant.</p>
          <p>Ask me anything about the Physical AI content!</p>
        </div>
      ) : (
        <>
          {messages.map((message, index) => (
            <Message key={index} message={message} />
          ))}
          
          {isLoading && (
            <div className="loading-indicator">
              <div className="loading-dots">
                <span>.</span>
                <span>.</span>
                <span>.</span>
              </div>
              <span className="loading-text">Thinking...</span>
            </div>
          )}
        </>
      )}
      
      <div ref={messagesEndRef} />
    </div>
  );
};

export default ChatWindow;
