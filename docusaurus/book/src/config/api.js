// API Configuration
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// For use in React components
export const useApiUrl = () => {
    const { siteConfig } = useDocusaurusContext();
    return siteConfig.customFields.API_URL || 'http://localhost:8001';
};

// For use outside React components (direct import)
const API_BASE_URL = 'https://ar2107927-panaversity-rag-backend.hf.space';

export default API_BASE_URL;
