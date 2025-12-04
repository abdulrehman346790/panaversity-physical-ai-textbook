import React, { createContext, useContext, useState, useEffect } from 'react';

const AuthContext = createContext(null);

export const AuthProvider = ({ children }) => {
    const [user, setUser] = useState(null);
    const [token, setToken] = useState(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        // Check local storage for token (with error handling for blocked storage)
        try {
            const storedToken = localStorage.getItem('auth_token');
            const storedUser = localStorage.getItem('auth_user');

            if (storedToken && storedUser) {
                setToken(storedToken);
                setUser(JSON.parse(storedUser));
            }
        } catch (error) {
            console.warn('localStorage access blocked:', error);
            // Continue without stored auth - user can still use the app
        }
        setLoading(false);
    }, []);

    const login = (userData, authToken) => {
        setUser(userData);
        setToken(authToken);
        try {
            localStorage.setItem('auth_token', authToken);
            localStorage.setItem('auth_user', JSON.stringify(userData));
        } catch (error) {
            console.warn('localStorage save failed:', error);
            // Auth still works, just won't persist across page reloads
        }
    };

    const logout = () => {
        setUser(null);
        setToken(null);
        try {
            localStorage.removeItem('auth_token');
            localStorage.removeItem('auth_user');
        } catch (error) {
            console.warn('localStorage clear failed:', error);
        }
    };

    return (
        <AuthContext.Provider value={{ user, token, login, logout, loading }}>
            {children}
        </AuthContext.Provider>
    );
};

export const useAuth = () => useContext(AuthContext);
