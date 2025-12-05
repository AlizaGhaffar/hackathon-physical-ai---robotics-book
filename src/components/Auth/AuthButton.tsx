import React, { useState, useEffect } from 'react';
import styles from './AuthButton.module.css';

interface User {
  id: string;
  email: string;
  name: string;
  software_level: string;
  hardware_level: string;
}

const AuthButton: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [user, setUser] = useState<User | null>(null);
  const [showLogin, setShowLogin] = useState(false);
  const [showSignup, setShowSignup] = useState(false);

  // Login form state
  const [loginEmail, setLoginEmail] = useState('');
  const [loginPassword, setLoginPassword] = useState('');

  // Signup form state
  const [signupEmail, setSignupEmail] = useState('');
  const [signupPassword, setSignupPassword] = useState('');
  const [signupName, setSignupName] = useState('');
  const [softwareLevel, setSoftwareLevel] = useState('Beginner');
  const [hardwareLevel, setHardwareLevel] = useState('Beginner');
  const [learningGoals, setLearningGoals] = useState('');

  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  useEffect(() => {
    // Check if user is logged in
    const token = localStorage.getItem('auth_token');
    const userData = localStorage.getItem('user_data');

    if (token && userData) {
      try {
        setUser(JSON.parse(userData));
      } catch (e) {
        console.error('Failed to parse user data');
      }
    }
  }, []);

  const handleLogin = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email: loginEmail,
          password: loginPassword,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        setError(data.detail || 'Invalid email or password');
        setLoading(false);
        return;
      }

      const data = await response.json();
      localStorage.setItem('auth_token', data.access_token);
      localStorage.setItem('user_data', JSON.stringify(data.user));
      setUser(data.user);
      setShowLogin(false);
      setLoginEmail('');
      setLoginPassword('');

      // Reload page to apply auth guard
      window.location.reload();
    } catch (err) {
      console.error('Login error:', err);
      setError('Unable to connect to server. Please try again in a moment.');
    } finally {
      setLoading(false);
    }
  };

  const handleSignup = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    try {
      const response = await fetch('http://localhost:8000/api/auth/signup', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          email: signupEmail,
          password: signupPassword,
          name: signupName,
          software_level: softwareLevel,
          hardware_level: hardwareLevel,
          learning_goals: learningGoals,
        }),
      });

      if (!response.ok) {
        const data = await response.json();
        setError(data.detail || 'Signup failed. Email may already be registered.');
        setLoading(false);
        return;
      }

      const data = await response.json();
      localStorage.setItem('auth_token', data.access_token);
      localStorage.setItem('user_data', JSON.stringify(data.user));
      setUser(data.user);
      setShowSignup(false);

      // Reset form
      setSignupEmail('');
      setSignupPassword('');
      setSignupName('');
      setLearningGoals('');

      // Reload page to apply auth guard
      window.location.reload();
    } catch (err) {
      console.error('Signup error:', err);
      setError('Unable to connect to server. Please try again in a moment.');
    } finally {
      setLoading(false);
    }
  };

  const handleLogout = () => {
    localStorage.removeItem('auth_token');
    localStorage.removeItem('user_data');
    setUser(null);
    setIsOpen(false);
  };

  if (user) {
    return (
      <div className={styles.authContainer}>
        <button
          className={styles.userButton}
          onClick={() => setIsOpen(!isOpen)}
        >
          <span className={styles.userAvatar}>{user.name.charAt(0).toUpperCase()}</span>
          <span className={styles.userName}>{user.name}</span>
          <svg width="12" height="12" viewBox="0 0 12 12" fill="currentColor">
            <path d="M2 4l4 4 4-4" stroke="currentColor" strokeWidth="2" fill="none" />
          </svg>
        </button>

        {isOpen && (
          <div className={styles.dropdown}>
            <div className={styles.userInfo}>
              <div className={styles.userEmail}>{user.email}</div>
              <div className={styles.userLevels}>
                <span className={styles.badge}>💻 {user.software_level}</span>
                <span className={styles.badge}>🔧 {user.hardware_level}</span>
              </div>
            </div>
            <div className={styles.divider} />
            <button className={styles.menuItem} onClick={() => setIsOpen(false)}>
              Profile Settings
            </button>
            <button className={styles.menuItem} onClick={() => setIsOpen(false)}>
              Learning Progress
            </button>
            <div className={styles.divider} />
            <button className={styles.logoutButton} onClick={handleLogout}>
              Logout
            </button>
          </div>
        )}
      </div>
    );
  }

  return (
    <>
      <button
        className={styles.loginButton}
        onClick={() => setShowLogin(true)}
      >
        🔐 Login / Sign Up
      </button>

      {/* Login Modal */}
      {showLogin && (
        <div className={styles.modalOverlay} onClick={() => setShowLogin(false)}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalHeader}>
              <h2>Login</h2>
              <button
                className={styles.closeButton}
                onClick={() => setShowLogin(false)}
              >
                ×
              </button>
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <form onSubmit={handleLogin}>
              <div className={styles.formGroup}>
                <label>Email</label>
                <input
                  type="email"
                  value={loginEmail}
                  onChange={(e) => setLoginEmail(e.target.value)}
                  required
                  placeholder="your@email.com"
                />
              </div>

              <div className={styles.formGroup}>
                <label>Password</label>
                <input
                  type="password"
                  value={loginPassword}
                  onChange={(e) => setLoginPassword(e.target.value)}
                  required
                  placeholder="••••••••"
                />
              </div>

              <button
                type="submit"
                className={styles.submitButton}
                disabled={loading}
              >
                {loading ? 'Logging in...' : 'Login'}
              </button>
            </form>

            <div className={styles.switchAuth}>
              Don't have an account?{' '}
              <button
                onClick={() => {
                  setShowLogin(false);
                  setShowSignup(true);
                }}
              >
                Sign Up
              </button>
            </div>
          </div>
        </div>
      )}

      {/* Signup Modal */}
      {showSignup && (
        <div className={styles.modalOverlay} onClick={() => setShowSignup(false)}>
          <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalHeader}>
              <h2>Sign Up</h2>
              <button
                className={styles.closeButton}
                onClick={() => setShowSignup(false)}
              >
                ×
              </button>
            </div>

            {error && <div className={styles.error}>{error}</div>}

            <form onSubmit={handleSignup}>
              <div className={styles.formGroup}>
                <label>Name</label>
                <input
                  type="text"
                  value={signupName}
                  onChange={(e) => setSignupName(e.target.value)}
                  required
                  placeholder="Your Name"
                />
              </div>

              <div className={styles.formGroup}>
                <label>Email</label>
                <input
                  type="email"
                  value={signupEmail}
                  onChange={(e) => setSignupEmail(e.target.value)}
                  required
                  placeholder="your@email.com"
                />
              </div>

              <div className={styles.formGroup}>
                <label>Password</label>
                <input
                  type="password"
                  value={signupPassword}
                  onChange={(e) => setSignupPassword(e.target.value)}
                  required
                  placeholder="••••••••"
                  minLength={8}
                />
              </div>

              <div className={styles.formRow}>
                <div className={styles.formGroup}>
                  <label>Software Level</label>
                  <select
                    value={softwareLevel}
                    onChange={(e) => setSoftwareLevel(e.target.value)}
                  >
                    <option value="Beginner">Beginner</option>
                    <option value="Intermediate">Intermediate</option>
                    <option value="Advanced">Advanced</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label>Hardware Level</label>
                  <select
                    value={hardwareLevel}
                    onChange={(e) => setHardwareLevel(e.target.value)}
                  >
                    <option value="Beginner">Beginner</option>
                    <option value="Intermediate">Intermediate</option>
                    <option value="Advanced">Advanced</option>
                  </select>
                </div>
              </div>

              <div className={styles.formGroup}>
                <label>Learning Goals (Optional)</label>
                <textarea
                  value={learningGoals}
                  onChange={(e) => setLearningGoals(e.target.value)}
                  placeholder="What do you want to learn?"
                  rows={3}
                />
              </div>

              <button
                type="submit"
                className={styles.submitButton}
                disabled={loading}
              >
                {loading ? 'Creating account...' : 'Sign Up'}
              </button>
            </form>

            <div className={styles.switchAuth}>
              Already have an account?{' '}
              <button
                onClick={() => {
                  setShowSignup(false);
                  setShowLogin(true);
                }}
              >
                Login
              </button>
            </div>
          </div>
        </div>
      )}
    </>
  );
};

export default AuthButton;
