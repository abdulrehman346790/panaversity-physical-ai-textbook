import React, { createContext, useContext, useEffect, useState } from 'react';

const LanguageContext = createContext({
  language: 'en',
  setLanguage: (lang) => {},
});

export const LanguageProvider = ({ children }) => {
  const [language, setLanguage] = useState('en');

  useEffect(() => {
    try {
      const stored = typeof window !== 'undefined' && localStorage.getItem('panaversity_language');
      if (stored) setLanguage(stored);
    } catch (err) {
      // ignore
    }
  }, []);

  useEffect(() => {
    if (typeof window !== 'undefined') {
      try {
        localStorage.setItem('panaversity_language', language);
        // toggle direction
        document.documentElement.dir = language === 'ur' ? 'rtl' : 'ltr';
      } catch (err) {}
    }
  }, [language]);

  return (
    <LanguageContext.Provider value={{ language, setLanguage }}>
      {children}
    </LanguageContext.Provider>
  );
};

export const useLanguage = () => useContext(LanguageContext);

export default LanguageContext;
