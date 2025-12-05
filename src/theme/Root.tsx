import React, { useEffect } from 'react';
import { AuthButton, AuthGuard } from '../components/Auth';

// Default Root component wrapper for Docusaurus
export default function Root({ children }: { children: React.ReactNode }) {
  useEffect(() => {
    // Add custom navbar items
    const addNavbarItems = () => {
      const navbar = document.querySelector('.navbar__items--right');

      if (navbar && !document.getElementById('custom-navbar-items')) {
        const container = document.createElement('div');
        container.id = 'custom-navbar-items';
        container.style.display = 'flex';
        container.style.alignItems = 'center';
        container.style.gap = '12px';
        container.style.marginLeft = '12px';

        // Insert before GitHub link
        const githubLink = navbar.querySelector('a[href*="github"]');
        if (githubLink) {
          navbar.insertBefore(container, githubLink);
        } else {
          navbar.appendChild(container);
        }

        // Render React components into the container
        const ReactDOM = require('react-dom/client');
        const root = ReactDOM.createRoot(container);

        root.render(
          <React.StrictMode>
            <AuthButton />
          </React.StrictMode>
        );
      }
    };

    // Try multiple times as navbar might not be ready immediately
    const intervals = [0, 100, 500, 1000];
    intervals.forEach(delay => {
      setTimeout(addNavbarItems, delay);
    });
  }, []);

  return (
    <AuthGuard>
      {children}
    </AuthGuard>
  );
}
