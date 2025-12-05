import { useEffect, useState } from 'react';
import { useTranslation } from '../contexts/TranslationContext';
import { getPageTranslation, PageTranslation } from '../i18n/contentTranslations';
import { useLocation } from '@docusaurus/router';

/**
 * Hook for getting content translations for the current page
 */
export function useContentTranslation() {
  const { language } = useTranslation();
  const location = useLocation();
  const [pageTranslation, setPageTranslation] = useState<PageTranslation | null>(null);

  useEffect(() => {
    const translation = getPageTranslation(location.pathname, language);
    setPageTranslation(translation);
  }, [location.pathname, language]);

  return {
    pageTranslation,
    language,
  };
}

/**
 * Hook for translating specific content by key
 */
export function useContentTranslate() {
  const { language } = useTranslation();
  const location = useLocation();

  const tc = (key: string, fallback?: string): string => {
    const translation = getPageTranslation(location.pathname, language);
    if (translation?.content && translation.content[key]) {
      return translation.content[key];
    }
    return fallback || key;
  };

  return { tc, language };
}
