/**
 * ChatModal Component
 * Modal overlay wrapper for the ChatBot component
 */

import React, { useEffect } from 'react';
import ChatBot from './ChatBot';
import styles from './ChatModal.module.css';

interface ChatModalProps {
  isOpen: boolean;
  onClose: () => void;
}

export default function ChatModal({ isOpen, onClose }: ChatModalProps) {
  // Close modal on Escape key press
  useEffect(() => {
    const handleEscape = (e: KeyboardEvent) => {
      if (e.key === 'Escape' && isOpen) {
        onClose();
      }
    };

    document.addEventListener('keydown', handleEscape);
    return () => document.removeEventListener('keydown', handleEscape);
  }, [isOpen, onClose]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (isOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = 'unset';
    }

    return () => {
      document.body.style.overflow = 'unset';
    };
  }, [isOpen]);

  if (!isOpen) return null;

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <div className={styles.modalHeader}>
          <h3 className={styles.modalTitle}>AI Chatbot</h3>
          <button
            className={styles.closeButton}
            onClick={onClose}
            aria-label="Close chatbot"
          >
            ✕
          </button>
        </div>
        <div className={styles.modalBody}>
          <ChatBot />
        </div>
      </div>
    </div>
  );
}
