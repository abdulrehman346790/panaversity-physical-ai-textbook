import React from 'react';
import Content from '@theme-original/DocItem/Content';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import { useLocation } from '@docusaurus/router';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

function LanguageSwitcher() {
    const { i18n } = useDocusaurusContext();
    const location = useLocation();
    const currentLocale = i18n.currentLocale;

    // Calculate target path
    let targetPath;
    if (currentLocale === 'en') {
        // Switch to Urdu: Add /ur prefix
        // Ensure we don't double add if somehow already there (though locale check prevents this)
        targetPath = `/ur${location.pathname}`;
    } else {
        // Switch to English: Remove /ur prefix
        targetPath = location.pathname.replace(/^\/ur/, '') || '/';
    }

    const isUrdu = currentLocale === 'ur';

    return (
        <div className={styles.languageSwitcherContainer}>
            <Link
                to={targetPath}
                className={`${styles.languageButton} ${isUrdu ? styles.urduButton : styles.englishButton}`}
            >
                {isUrdu ? (
                    <>
                        <span style={{ marginRight: '8px' }}>🇬🇧</span>
                        Read in English
                    </>
                ) : (
                    <>
                        <span style={{ marginRight: '8px' }}>🇵🇰</span>
                        اردو میں پڑھیں
                    </>
                )}
            </Link>
        </div>
    );
}

export default function ContentWrapper(props) {
    return (
        <>
            <LanguageSwitcher />
            <Content {...props} />
        </>
    );
}
