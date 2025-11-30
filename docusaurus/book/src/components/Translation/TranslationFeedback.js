import React, { useState } from 'react';

const TranslationFeedback = ({sourcePath, userId}) => {
  const [comments, setComments] = useState('');
  const [status, setStatus] = useState(null);

  const submitFeedback = async () => {
    try {
      const params = new URLSearchParams({ user_id: userId || 'anon', source_path: sourcePath || '/', comments });
      const resp = await fetch(`/api/translate/feedback?${params.toString()}`, { method: 'POST' });
      if (resp.ok) {
        setStatus('Thank you — feedback received');
        setComments('');
      } else {
        setStatus('Error: could not submit feedback');
      }
    } catch (err) {
      setStatus('Network error');
    }
  };

  return (
    <div>
      <textarea value={comments} onChange={(e) => setComments(e.target.value)} placeholder="Report an issue with this translation..." />
      <div>
        <button onClick={submitFeedback}>Report</button>
        {status && <span>{status}</span>}
      </div>
    </div>
  );
};

export default TranslationFeedback;