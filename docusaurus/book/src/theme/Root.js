import React from 'react';
import Head from '@docusaurus/Head';
import { AuthProvider } from '../components/Auth/AuthContext';
import { TranslationProvider } from '../components/Translation/TranslationContext';

// Default implementation, that you can customize
export default function Root({ children }) {
  return (
    <AuthProvider>
      <TranslationProvider>
        <Head>
          <script>
            {`
            (function() {
              try {
                var x = '__storage_test__';
                localStorage.setItem(x, x);
                localStorage.removeItem(x);
              } catch (e) {
                console.warn('localStorage access blocked. Mocking it to prevent application crash.');
                var data = {};
                var storageMock = {
                  getItem: function(k) { return data[k] || null; },
                  setItem: function(k, v) { data[k] = String(v); },
                  removeItem: function(k) { delete data[k]; },
                  clear: function() { data = {}; },
                  key: function(i) { return Object.keys(data)[i] || null; },
                  get length() { return Object.keys(data).length; }
                };
                try {
                  Object.defineProperty(window, 'localStorage', { value: storageMock, writable: true, configurable: true });
                  Object.defineProperty(window, 'sessionStorage', { value: storageMock, writable: true, configurable: true });
                } catch (defErr) { console.error('Failed to mock localStorage:', defErr); }
              }
            })();
          `}
          </script>
        </Head>
        {children}
      </TranslationProvider>
    </AuthProvider>
  );
}
