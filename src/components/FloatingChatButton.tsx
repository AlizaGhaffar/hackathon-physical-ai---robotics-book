/**
 * FloatingChatButton Component
 * Floating button that opens the chatbot modal
 */

import React, { useState } from 'react';
import ChatModal from './ChatModal';
import styles from './FloatingChatButton.module.css';

export default function FloatingChatButton() {
  const [isModalOpen, setIsModalOpen] = useState(false);

  const openModal = () => setIsModalOpen(true);
  const closeModal = () => setIsModalOpen(false);

  return (
    <>
      <button
        className={styles.floatingButton}
        onClick={openModal}
        aria-label="Chat with us"
        title="Chat with us"
      >
        {/* Chat Icon (SVG) */}
        <svg
          xmlns="http://www.w3.org/2000/svg"
          viewBox="0 0 24 24"
          fill="currentColor"
          className={styles.chatIcon}
        >
          <path d="M20 2H4c-1.1 0-2 .9-2 2v18l4-4h14c1.1 0 2-.9 2-2V4c0-1.1-.9-2-2-2zm0 14H6l-2 2V4h16v12z" />
          <circle cx="8" cy="10" r="1.5" />
          <circle cx="12" cy="10" r="1.5" />
          <circle cx="16" cy="10" r="1.5" />
        </svg>

        {/* Tooltip */}
        <span className={styles.tooltip}>Chat with us</span>
      </button>

      <ChatModal isOpen={isModalOpen} onClose={closeModal} />
    </>
  );
}
