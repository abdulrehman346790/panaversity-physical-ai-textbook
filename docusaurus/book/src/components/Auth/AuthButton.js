import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import Signin from './Signin';
import Signup from './Signup';
import UserProfile from './UserProfile';
import styles from './Auth.module.css';

const AuthButton = () => {
    const auth = useAuth();
    const user = auth ? auth.user : null;
    const [showSignin, setShowSignin] = useState(false);
    const [showSignup, setShowSignup] = useState(false);

    if (user) {
        return <UserProfile />;
    }

    return (
        <>
            <button
                className={styles.navAuthButton}
                onClick={() => setShowSignin(true)}
            >
                <span>🔐</span> Sign In
            </button>

            {showSignin && (
                <Signin
                    onClose={() => setShowSignin(false)}
                    onSwitchToSignup={() => {
                        setShowSignin(false);
                        setShowSignup(true);
                    }}
                />
            )}

            {showSignup && (
                <Signup
                    onClose={() => setShowSignup(false)}
                />
            )}
        </>
    );
};

export default AuthButton;
