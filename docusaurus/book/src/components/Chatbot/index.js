import React, { useState, useEffect } from 'react';
import { useAuth } from '../Auth/AuthContext';
import styles from './Chatbot.module.css';
import TranslationFeedback from '@site/src/components/Translation/TranslationFeedback';

const Chatbot = () => {
    const { token } = useAuth(); // Get token from AuthContext
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([
        { role: 'assistant', content: 'Hi! I am your AI Tutor. Ask me anything about the book!' }
    ]);
    const [feedbackOpen, setFeedbackOpen] = useState(false);
    const [feedbackMessageData, setFeedbackMessageData] = useState(null);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');

    const toggleChat = () => setIsOpen(!isOpen);

    // Handle text selection from the page
    useEffect(() => {
        const handleTextSelection = () => {
            const selection = window.getSelection().toString().trim();
            if (selection && selection.length > 0 && selection.length < 500) {
                setSelectedText(selection);
                setInput(`Explain this: "${selection}"`);
                setIsOpen(true); // Auto-open chat when text is selected
            }
        };

        document.addEventListener('mouseup', handleTextSelection);
        return () => document.removeEventListener('mouseup', handleTextSelection);
    }, []);

    const sendMessage = async () => {
        if (!input.trim()) return;

        const userMessage = { role: 'user', content: input };
        setMessages(prev => [...prev, userMessage]);
        setInput('');
        setSelectedText(''); // Clear selected text after sending
        setIsLoading(true);

        try {
            const response = await fetch('http://localhost:8001/chat', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: input,
                    ...(token && { token }) // Only send token if it exists
                }),
            });

            const data = await response.json();
            const botMessage = { role: 'assistant', content: data.response, translated: data.translated };
            setMessages(prev => [...prev, botMessage]);
            setFeedbackMessageData(null);
        } catch (error) {
            console.error('Error sending message:', error);
            setMessages(prev => [...prev, { role: 'assistant', content: 'Sorry, I encountered an error. Please check if the backend is running.' }]);
        } finally {
            setIsLoading(false);
        }
    };

    const handleOpenFeedback = (msg) => {
        setFeedbackMessageData({ content: msg.content, source_path: window.location.pathname });
        setFeedbackOpen(true);
    };
    const handleCloseFeedback = () => setFeedbackOpen(false);

    return (
        <div className={styles.chatbotContainer}>
            {/* Floating Chat Button */}
            <button className={styles.chatButton} onClick={toggleChat}>
                💬
            </button>

            {/* Chat Window */}
            {isOpen && (
                <div className={styles.chatWindow}>
                    <div className={styles.chatHeader}>
                        <h3>AI Tutor</h3>
                        <button className={styles.closeButton} onClick={toggleChat}>×</button>
                    </div>

                    <div className={styles.chatMessages}>
                        {messages.map((msg, idx) => (
                            <div key={idx} className={msg.role === 'user' ? styles.userMessage : styles.botMessage}>
                                <div className={styles.messageContent}>
                                    <span>{msg.content}</span>
                                    {msg.translated && (
                                        <div className={styles.translationMeta}>
                                            <span className={styles.translatedLabel}>Auto-translated</span>
                                            <button className={styles.reportButton} onClick={() => handleOpenFeedback(msg)}>
                                                Report Issue
                                            </button>
                                        </div>
                                    )}
                                </div>
                            </div>
                        ))}
                        {isLoading && <div className={styles.botMessage}>Thinking...</div>}
                    </div>

                    <div className={styles.chatInput}>
                        {selectedText && (
                            <div className={styles.selectedTextIndicator}>
                                Selected: "{selectedText.substring(0, 50)}..."
                            </div>
                        )}
                        <input
                            type="text"
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                            placeholder="Ask about the textbook..."
                        />
                        <button onClick={sendMessage}>Send</button>
                    </div>

                    {feedbackOpen && (
                        <div className={styles.feedbackModalOverlay}>
                            <div className={styles.feedbackModal}>
                                <h4>Report Translation Issue</h4>
                                <TranslationFeedback
                                    sourcePath={feedbackMessageData?.source_path}
                                    userId={token ? 'user' : 'anon'}
                                />
                                <button className={styles.closeModalButton} onClick={handleCloseFeedback}>Close</button>
                            </div>
                        </div>
                    )}
                </div>
            )}
        </div>
    );
};

export default Chatbot;
