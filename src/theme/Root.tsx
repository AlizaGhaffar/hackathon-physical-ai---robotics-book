import React from 'react';
import FloatingChatButton from '@site/src/components/FloatingChatButton';

// This Root component wraps all pages in Docusaurus
export default function Root({ children }) {
  return (
    <>
      {children}
      <FloatingChatButton />
    </>
  );
}
