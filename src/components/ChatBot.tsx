/**
 * ChatBot Component
 * Interactive chat interface for RAG-based question answering
 */

import React, { useState, useEffect, useRef } from 'react';
import { useLocation } from '@docusaurus/router';
import {
  MainContainer,
  ChatContainer,
  MessageList,
  Message as ChatMessage,
  MessageInput,
  TypingIndicator,
} from '@chatscope/chat-ui-kit-react';
import '@chatscope/chat-ui-kit-styles/dist/default/styles.min.css';
import ReactMarkdown from 'react-markdown';
import { Prism as SyntaxHighlighter } from 'react-syntax-highlighter';
import { oneDark } from 'react-syntax-highlighter/dist/esm/styles/prism';

import {
  sendQuery,
  getOrCreateSessionId,
  extractChapterId,
  ChatServiceError,
} from '../services/chatService';
import type { Message } from '../types/chat';
import styles from './ChatBot.module.css';

export default function ChatBot() {
  const location = useLocation();

  // Component state
  const [messages, setMessages] = useState<Message[]>([]);
  const [isLoading, setIsLoading] = useState(false);
  const [sessionId] = useState(() => getOrCreateSessionId());
  const [chapterId] = useState(() => extractChapterId(location.pathname));

  // Ref for auto-scroll
  const messageListRef = useRef<any>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    if (messageListRef.current) {
      messageListRef.current.scrollToBottom();
    }
  }, [messages]);

  /**
   * Handle sending a message to the backend
   */
  const handleSend = async (messageText: string) => {
    const trimmedMessage = messageText.trim();

    // Validate input (FR-009: prevent empty messages)
    if (!trimmedMessage || isLoading) return;

    // Add user message to chat (FR-002: display user messages immediately)
    const userMessage: Message = {
      id: crypto.randomUUID(),
      role: 'user',
      content: trimmedMessage,
      timestamp: new Date(),
    };
    setMessages((prev) => [...prev, userMessage]);

    // Call backend API
    setIsLoading(true); // FR-003: show loading indicator
    try {
      const response = await sendQuery(trimmedMessage, chapterId, sessionId);

      // Add assistant message (FR-004: display AI answers)
      const assistantMessage: Message = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content: response.answer,
        timestamp: new Date(),
        sources: response.sources, // FR-007: include source citations
      };
      
      setMessages((prev) => [...prev, assistantMessage]);

      // Log performance metrics
      if (response.response_time_ms) {
        console.log(
          `[ChatBot] Response time: ${(response.response_time_ms / 1000).toFixed(2)}s`
        );
      }
    } catch (error) {
      console.error('[ChatBot] Error:', error);

      // FR-008: handle and display error messages
      const errorMessage: Message = {
        id: crypto.randomUUID(),
        role: 'assistant',
        content:
          error instanceof ChatServiceError
            ? error.message
            : 'An unexpected error occurred. Please try again.',
        timestamp: new Date(),
        isError: true,
      };
      setMessages((prev) => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.chatContainer}>
      <MainContainer>
        <ChatContainer>
          <MessageList
            ref={messageListRef}
            typingIndicator={isLoading && <TypingIndicator content="AI is thinking..." />}
          >
            {messages.map((msg) => (
              <ChatMessage
                key={msg.id}
                model={{
                  message: msg.content,
                  sender: msg.role === 'user' ? 'You' : 'AI Assistant',
                  direction: msg.role === 'user' ? 'outgoing' : 'incoming',
                  position: 'single',
                }}
                className={msg.isError ? styles.errorMessage : ''}
              >
                <ChatMessage.CustomContent>
                  <div>
                    {/* FR-012: Format AI responses with markdown */}
                    <ReactMarkdown
                      components={{
                        code({ node, inline, className, children, ...props }) {
                          const match = /language-(\w+)/.exec(className || '');
                          return !inline && match ? (
                            <SyntaxHighlighter
                              language={match[1]}
                              style={oneDark}
                              PreTag="div"
                            >
                              {String(children).replace(/\n$/, '')}
                            </SyntaxHighlighter>
                          ) : (
                            <code className={className} {...props}>
                              {children}
                            </code>
                          );
                        },
                      }}
                    >
                      {msg.content}
                    </ReactMarkdown>
                    {/* FR-007: Display source citations */}
                    {msg.sources && msg.sources.length > 0 && (
                      <details className={styles.sources}>
                        <summary>📚 Sources ({msg.sources.length})</summary>
                        <ul>
                          {msg.sources.map((source) => (
                            <li key={source.chunk_id}>
                              <strong>{source.source_file}</strong> (relevance:{' '}
                              {(source.score * 100).toFixed(0)}%)
                              <p className={styles.sourceContent}>
                                {source.content.substring(0, 200)}...
                              </p>
                            </li>
                          ))}
                        </ul>
                      </details>
                    )}
                  </div>
                </ChatMessage.CustomContent>
              </ChatMessage>
            ))}
          </MessageList>
          <MessageInput
            placeholder={isLoading ? 'Please wait...' : 'Ask a question about this chapter...'}
            onSend={handleSend}
            disabled={isLoading}
            attachButton={false}
          />
        </ChatContainer>
      </MainContainer>
    </div>
  );
}
