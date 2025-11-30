import React, { createContext, useContext, useState, useEffect } from 'react';
import { useAuth } from '../Auth/AuthContext';

const TranslationContext = createContext(null);

export const TranslationProvider = ({ children }) => {
    const [language, setLanguage] = useState('en'); // 'en' or 'ur'
    
    useEffect(() => {
        try {
            const stored = typeof window !== 'undefined' && localStorage.getItem('panaversity_language');
            if (stored) {
                setLanguage(stored);
            } else if (typeof navigator !== 'undefined') {
                const lang = navigator.language || navigator.userLanguage || 'en';
                if (lang.startsWith('ur')) setLanguage('ur');
            }
        } catch (err) {
            // ignore
        }
    }, []);
    const [translationCache, setTranslationCache] = useState({});

    const translateText = async (text) => {
        if (language === 'en') return text;

        // Check cache first
        if (translationCache[text]) {
            return translationCache[text];
        }

        try {
            const response = await fetch('http://localhost:8000/api/translate', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ markdown: text, target_locale: 'ur' })
            });

            const data = await response.json();
            const translated = data.translated_text;

            // Cache the translation
            setTranslationCache(prev => ({ ...prev, [text]: translated }));

            return translated;
        } catch (error) {
            console.error('Translation error:', error);
            return text; // Fallback to original text
        }
    };

    const { token } = useAuth();
    const toggleLanguage = async () => {
        setLanguage(prev => {
            const next = prev === 'en' ? 'ur' : 'en';
            if (typeof window !== 'undefined') {
                try { localStorage.setItem('panaversity_language', next); } catch (e) {}
            }
            return next;
        });
        // Persist to server if token is present
        if (token) {
            try {
                const nextLang = language === 'en' ? 'ur' : 'en';
                await fetch(`http://localhost:8001/api/user/lang?token=${encodeURIComponent(token)}&language=${encodeURIComponent(nextLang)}`, { method: 'POST' });
            } catch (err) { /* ignore */ }
        }
    };

    return (
        <TranslationContext.Provider value={{ language, translateText, toggleLanguage }}>
            {children}
        </TranslationContext.Provider>
    );
};

export const useTranslation = () => useContext(TranslationContext);
