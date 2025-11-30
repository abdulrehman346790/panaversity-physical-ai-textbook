import React, { useEffect } from 'react';
import { AuthProvider } from '@site/src/components/Auth/AuthContext';
import { TranslationProvider, useTranslation } from '@site/src/components/Translation/TranslationContext';
import Chatbot from '@site/src/components/Chatbot';
import '@site/src/theme/rtl.css';

// Inner component to access translation context
function RootContent({ children }) {
    const { language } = useTranslation();

    useEffect(() => {
        // Set document direction based on language
        document.documentElement.setAttribute('dir', language === 'ur' ? 'rtl' : 'ltr');
    }, [language]);

    return (
        <>
            {children}
            {/* Chatbot */}
            <Chatbot />
        </>
    );
}

// Default implementation, that you can customize
export default function Root({ children }) {
    return (
        <AuthProvider>
            <TranslationProvider>
                <RootContent>{children}</RootContent>
            </TranslationProvider>
        </AuthProvider>
    );
}
