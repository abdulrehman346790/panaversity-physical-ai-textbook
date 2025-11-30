import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './Auth.module.css';

const UserProfile = () => {
    const { user, logout } = useAuth();
    const [showMenu, setShowMenu] = useState(false);

    if (!user) return null;

    const initials = user.name
        ? user.name.split(' ').map(n => n[0]).join('').toUpperCase().substring(0, 2)
        : 'U';

    return (
        <div style={{ position: 'relative' }}>
            <div className={styles.userProfile} onClick={() => setShowMenu(!showMenu)}>
                <div className={styles.avatar}>{initials}</div>
                <span>{user.name}</span>
            </div>

            {showMenu && (
                <div style={{
                    position: 'absolute',
                    top: '100%',
                    right: 0,
                    background: 'var(--ifm-background-surface-color)',
                    border: '1px solid var(--ifm-color-emphasis-200)',
                    borderRadius: '8px',
                    padding: '0.5rem',
                    boxShadow: '0 5px 15px rgba(0,0,0,0.1)',
                    minWidth: '150px',
                    zIndex: 100
                }}>
                    <div style={{ padding: '0.5rem', borderBottom: '1px solid var(--ifm-color-emphasis-200)', fontSize: '0.8rem', color: 'var(--ifm-color-emphasis-600)' }}>
                        {user.email}
                    </div>
                    <button
                        onClick={logout}
                        style={{
                            width: '100%',
                            padding: '0.5rem',
                            background: 'none',
                            border: 'none',
                            textAlign: 'left',
                            cursor: 'pointer',
                            color: '#ff4d4f',
                            marginTop: '0.5rem'
                        }}
                    >
                        Sign Out
                    </button>
                </div>
            )}
        </div>
    );
};

export default UserProfile;
