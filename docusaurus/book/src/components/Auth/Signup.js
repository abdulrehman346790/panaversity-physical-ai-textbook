import React, { useState } from 'react';
import ReactDOM from 'react-dom';
import { useAuth } from './AuthContext';
import styles from './Auth.module.css';
import API_BASE_URL from '@site/src/config/api';

const Signup = ({ onClose }) => {
    const { login } = useAuth();
    const [step, setStep] = useState(1);
    const [loading, setLoading] = useState(false);
    const [error, setError] = useState('');

    const [formData, setFormData] = useState({
        email: '',
        password: '',
        name: '',
        software_experience: 'beginner',
        programming_languages: [],
        has_ros_experience: false,
        hardware_experience: 'none',
        hardware_description: '',
        has_robotics_hardware: false,
        learning_goal: 'career_change',
        learning_goals_description: ''
    });

    const handleInputChange = (e) => {
        const { name, value, type, checked } = e.target;
        setFormData(prev => ({
            ...prev,
            [name]: type === 'checkbox' ? checked : value
        }));
    };

    const handleLanguageChange = (lang) => {
        setFormData(prev => {
            const langs = prev.programming_languages.includes(lang)
                ? prev.programming_languages.filter(l => l !== lang)
                : [...prev.programming_languages, lang];
            return { ...prev, programming_languages: langs };
        });
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        setLoading(true);
        setError('');

        try {
            const response = await fetch(`${API_BASE_URL}/api/auth/signup`, {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({
                    email: formData.email,
                    password: formData.password,
                    name: formData.name,
                    profile: {
                        software_experience: formData.software_experience,
                        programming_languages: formData.programming_languages,
                        has_ros_experience: formData.has_ros_experience,
                        hardware_experience: formData.hardware_experience,
                        hardware_description: formData.hardware_description,
                        has_robotics_hardware: formData.has_robotics_hardware,
                        learning_goal: formData.learning_goal,
                        learning_goals_description: formData.learning_goals_description
                    }
                })
            });

            const data = await response.json();

            if (!response.ok) {
                throw new Error(data.detail || 'Signup failed');
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

                {step === 1 ? (
                    <form onSubmit={(e) => { e.preventDefault(); setStep(2); }}>
                        <h2>Create Account</h2>
                        {error && <div className={styles.error}>{error}</div>}

                        <div className={styles.formGroup}>
                            <label>Name</label>
                            <input required name="name" value={formData.name} onChange={handleInputChange} />
                        </div>

                        <div className={styles.formGroup}>
                            <label>Email</label>
                            <input required type="email" name="email" value={formData.email} onChange={handleInputChange} />
                        </div>

                        <div className={styles.formGroup}>
                            <label>Password</label>
                            <input required type="password" name="password" value={formData.password} onChange={handleInputChange} />
                        </div>

                        <button type="submit" className={styles.submitButton}>Next: Personalization</button>
                    </form>
                ) : (
                    <form onSubmit={handleSubmit}>
                        <h2>Personalize Your Learning</h2>
                        <p className={styles.subtitle}>Help us tailor the content to your background</p>

                        <div className={styles.formGroup}>
                            <label>Software Experience</label>
                            <select name="software_experience" value={formData.software_experience} onChange={handleInputChange}>
                                <option value="beginner">Beginner (New to coding)</option>
                                <option value="intermediate">Intermediate (Some experience)</option>
                                <option value="advanced">Advanced (Professional)</option>
                            </select>
                        </div>

                        <div className={styles.formGroup}>
                            <label>Programming Languages</label>
                            <div className={styles.checkboxGroup}>
                                {['Python', 'C++', 'JavaScript', 'Rust'].map(lang => (
                                    <label key={lang} className={styles.checkboxLabel}>
                                        <input
                                            type="checkbox"
                                            checked={formData.programming_languages.includes(lang)}
                                            onChange={() => handleLanguageChange(lang)}
                                        />
                                        {lang}
                                    </label>
                                ))}
                            </div>
                        </div>

                        <div className={styles.formGroup}>
                            <label className={styles.checkboxLabel}>
                                <input
                                    type="checkbox"
                                    name="has_ros_experience"
                                    checked={formData.has_ros_experience}
                                    onChange={handleInputChange}
                                />
                                I have experience with ROS/ROS 2
                            </label>
                        </div>

                        <div className={styles.formGroup}>
                            <label>Hardware Experience</label>
                            <select name="hardware_experience" value={formData.hardware_experience} onChange={handleInputChange}>
                                <option value="none">None</option>
                                <option value="basic">Basic (Arduino/Raspberry Pi)</option>
                                <option value="intermediate">Intermediate (Built robots)</option>
                                <option value="advanced">Advanced (Professional robotics)</option>
                            </select>
                        </div>

                        <button type="submit" disabled={loading} className={styles.submitButton}>
                            {loading ? 'Creating Account...' : 'Start Learning'}
                        </button>
                        <button type="button" className={styles.backButton} onClick={() => setStep(1)}>Back</button>
                    </form>
                )}
            </div>
        </div>,
        document.body
    );
};

export default Signup;
