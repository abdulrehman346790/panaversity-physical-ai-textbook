import React from 'react';
import { useTranslation } from '@site/src/components/Translation/TranslationContext';

const ProfileLanguageSettings = () => {
  const { language, toggleLanguage } = useTranslation();
  return (
    <div>
      <h3>Language preference</h3>
      <button onClick={toggleLanguage}>{language === 'en' ? 'Switch to Urdu' : 'Switch to English'}</button>
    </div>
  );
};

export default ProfileLanguageSettings;
