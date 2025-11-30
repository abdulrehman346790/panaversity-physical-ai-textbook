import React, { useState } from 'react';
import ReactDOM from 'react-dom';
import { useAuth } from './AuthContext';
import styles from './Auth.module.css';
import API_BASE_URL from '@site/src/config/api';

const Signin = ({ onClose, onSwitchToSignup }) => {
    const { login } = useAuth();
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const [formData, setFormData] = useState({
        email: '',
        password: ''
    });

    const handleInputChange = (e) => {
        const { name, value } = e.target;
        setFormData(prev => ({ ...prev, [name]: value }));
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        setLoading(true);
        setError('');

        try {
            const response = await fetch(`${API_BASE_URL}/api/auth/signin`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify(formData)
            });

            const data = await response.json();

            if (!response.ok) {
                throw new Error(data.detail || 'Signin failed');
            }

            login(data.user, data.access_token);
            onClose();
        } catch (err) {
            setError(err.message);
        } finally {
            setLoading(false);
        }
    };

    return ReactDOM.createPortal(
        <div className={styles.authContainer}>
            <div className={styles.authCard}>
                <button className={styles.closeButton} onClick={onClose}>×</button>

                <form onSubmit={handleSubmit}>
                    <h2>Welcome Back</h2>
                    {error && <div className={styles.error}>{error}</div>}

                    <div className={styles.formGroup}>
                        <label>Email</label>
                        <input required type="email" name="email" value={formData.email} onChange={handleInputChange} />
                    </div>

                    <div className={styles.formGroup}>
                        <label>Password</label>
                        <input required type="password" name="password" value={formData.password} onChange={handleInputChange} />
                    </div>

                    <button type="submit" disabled={loading} className={styles.submitButton}>
                        {loading ? 'Signing In...' : 'Sign In'}
                    </button>

                    <div className={styles.switchAuth}>
                        Don't have an account? <button type="button" onClick={onSwitchToSignup}>Create one</button>
                    </div>
                </form>
            </div>
        </div>,
        document.body
    );
};

export default Signin;
