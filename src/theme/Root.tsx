import React from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

interface RootProps {
  children: React.ReactNode;
}

// This Root component wraps all pages in Docusaurus
export default function Root({ children }: RootProps) {
  return (
    <>
      {children}
      <BrowserOnly>
        {() => {
          const FloatingChatButton = require('@site/src/components/FloatingChatButton').default;
          return <FloatingChatButton />;
        }}
      </BrowserOnly>
    </>
  );
}
