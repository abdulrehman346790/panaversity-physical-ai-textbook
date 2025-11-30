import React from 'react';
import { render, screen, fireEvent } from '@testing-library/react';
import { TranslationProvider, useTranslation } from '../src/components/Translation/TranslationContext';

const TestComponent = () => {
  const { language, toggleLanguage } = useTranslation();
  return (
    <div>
      <span data-testid="lang">{language}</span>
      <button onClick={toggleLanguage}>Toggle</button>
    </div>
  );
};

test('TranslationProvider default and toggle', () => {
  render(
    <TranslationProvider>
      <TestComponent />
    </TranslationProvider>
  );

  const lang = screen.getByTestId('lang');
  expect(lang.textContent).toBe('en');
  fireEvent.click(screen.getByText('Toggle'));
  expect(lang.textContent).toBe('ur');
});
