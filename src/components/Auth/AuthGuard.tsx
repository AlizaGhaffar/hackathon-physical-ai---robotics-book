import React, { useEffect, useState } from 'react';
import { useLocation } from '@docusaurus/router';
import styles from './AuthGuard.module.css';

interface AuthGuardProps {
  children: React.ReactNode;
}

const AuthGuard: React.FC<AuthGuardProps> = ({ children }) => {
  const [isAuthenticated, setIsAuthenticated] = useState<boolean | null>(null);
  const [showLoginPrompt, setShowLoginPrompt] = useState(false);
  const location = useLocation();

  // Routes that don't require authentication - intro page is public
  const publicRoutes = ['/', '/blog', '/chapter-1/intro'];

  useEffect(() => {
    const checkAuth = () => {
      const token = localStorage.getItem('auth_token');
      const userData = localStorage.getItem('user_data');

      // Check if current path is a public route
      const isPublicRoute = publicRoutes.some(route =>
        location.pathname === route || location.pathname === `/book${route}`
      );

      if (isPublicRoute) {
        setIsAuthenticated(true);
        return;
      }

      // For all doc pages (chapters), require authentication
      if (token && userData) {
        setIsAuthenticated(true);
      } else {
        setIsAuthenticated(false);
        setShowLoginPrompt(true);
      }
    };

    checkAuth();
  }, [location.pathname]);

  const handleLoginClick = () => {
    // Trigger the login button in the navbar
    const loginButton = document.querySelector('[class*="loginButton"]') as HTMLButtonElement;
    if (loginButton) {
      loginButton.click();
    }
    setShowLoginPrompt(false);
  };

  if (isAuthenticated === null) {
    return (
      <div className={styles.loading}>
        <div className={styles.spinner}></div>
        <p>Loading...</p>
      </div>
    );
  }

  if (!isAuthenticated && showLoginPrompt) {
    return (
      <div className={styles.authPromptContainer}>
        <div className={styles.authPrompt}>
          <div className={styles.lockIcon}>🔒</div>
          <h2>Authentication Required</h2>
          <p>Please sign in to access the book content and start your learning journey.</p>
          <button className={styles.loginButton} onClick={handleLoginClick}>
            Sign In / Sign Up
          </button>
          <p className={styles.helpText}>
            Don't have an account? Click the button above to create one!
          </p>
        </div>
      </div>
    );
  }

  return <>{children}</>;
};

export default AuthGuard;
