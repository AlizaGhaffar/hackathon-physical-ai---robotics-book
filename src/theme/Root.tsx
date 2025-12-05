import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

// This Root component wraps all pages in Docusaurus
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
}
