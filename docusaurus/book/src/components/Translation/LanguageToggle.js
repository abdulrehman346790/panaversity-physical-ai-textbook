import React from 'react';
import { useTranslation } from './TranslationContext';
import styles from './Translation.module.css';

const LanguageToggle = () => {
    const { language, toggleLanguage } = useTranslation();

    return (
        <button
            className={styles.languageToggle}
            onClick={toggleLanguage}
            title={language === 'en' ? 'اردو میں دیکھیں' : 'View in English'}
        >
            {language === 'en' ? '🇵🇰 اردو' : '🇬🇧 English'}
        </button>
    );
};

export default LanguageToggle;
