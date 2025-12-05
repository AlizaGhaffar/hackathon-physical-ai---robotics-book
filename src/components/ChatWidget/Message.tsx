import React from 'react';

interface MessageProps {
  message: {
    role: 'user' | 'assistant';
    content: string;
    citations?: Array<{
      chapter: number;
      section: string;
      chunk_id: string;
      similarity_score: number;
    }>;
  };
}

const Message: React.FC<MessageProps> = ({ message }) => {
  const { role, content, citations } = message;

  return (
    <div className={`message message-${role}`}>
      <div className="message-bubble">
        <div className="message-content">
          {content}
        </div>
        
        {citations && citations.length > 0 && (
          <div className="message-citations">
            <div className="citations-header">📚 Sources:</div>
            {citations.map((citation, index) => (
              <div key={index} className="citation-item">
                Chapter {citation.chapter}, {citation.section}
                <span className="citation-score">
                  {Math.round(citation.similarity_score * 100)}% match
                </span>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
};

export default Message;
