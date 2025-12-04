import React, { useState, useEffect, useRef } from 'react';
import { useAuth } from '../Auth/AuthContext';
import styles from './Chatbot.module.css';
import TranslationFeedback from '@site/src/components/Translation/TranslationFeedback';
import API_BASE_URL from '@site/src/config/api';

const Chatbot = () => {
    const { token } = useAuth();
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([
        { role: 'assistant', content: 'Hi! I am your AI Tutor. Ask me anything about the book!' }
    ]);
    const [feedbackOpen, setFeedbackOpen] = useState(false);
    const [feedbackMessageData, setFeedbackMessageData] = useState(null);
    const [input, setInput] = useState('');
    const [isLoading, setIsLoading] = useState(false);
    const [selectedText, setSelectedText] = useState('');
    const [tooltipPosition, setTooltipPosition] = useState(null);
    const [showTooltip, setShowTooltip] = useState(false);
    const tooltipRef = useRef(null);

    const toggleChat = () => setIsOpen(!isOpen);

    // Handle text selection with floating tooltip
    useEffect(() => {
        const handleTextSelection = () => {
            const selection = window.getSelection();
            const text = selection.toString().trim();

            if (text && text.length > 0 && text.length < 500) {
                const range = selection.getRangeAt(0);
                const rect = range.getBoundingClientRect();

                setSelectedText(text);
                setTooltipPosition({
                    top: rect.top + window.scrollY - 60,
                    left: rect.left + window.scrollX + (rect.width / 2)
                });
                setShowTooltip(true);
            } else {
                setShowTooltip(false);
            }
        };

        const handleClickOutside = (e) => {
            if (tooltipRef.current && !tooltipRef.current.contains(e.target)) {
                setShowTooltip(false);
            }
        };

        document.addEventListener('mouseup', handleTextSelection);
        document.addEventListener('click', handleClickOutside);

        return () => {
            document.removeEventListener('mouseup', handleTextSelection);
            document.removeEventListener('click', handleClickOutside);
        };
    }, []);

    // Keyboard shortcut: Ctrl+Enter to send selected text
    useEffect(() => {
        const handleKeyboard = (e) => {
            if (e.ctrlKey && e.key === 'Enter' && selectedText) {
                handleQuickAction('explain');
            }
        };

        document.addEventListener('keydown', handleKeyboard);
        return () => document.removeEventListener('keydown', handleKeyboard);
    }, [selectedText]);

    const handleQuickAction = (action) => {
        if (!selectedText) return;

        let prompt = '';
        switch (action) {
            case 'explain':
                prompt = `Explain this: "${selectedText}"`;
                break;
            case 'summarize':
                prompt = `Summarize this: "${selectedText}"`;
                break;
            case 'translate':
                prompt = `Translate this to Urdu: "${selectedText}"`;
                break;
            case 'define':
                prompt = `Define: "${selectedText}"`;
                break;
            default:
                prompt = selectedText;
        }

        setInput(prompt);
        setIsOpen(true);
        setShowTooltip(false);

        // Auto-focus input after opening
        setTimeout(() => {
            const inputElement = document.querySelector(`.${styles.chatInput} input`);
            if (inputElement) inputElement.focus();
        }, 100);
    };

    const sendMessage = async () => {
        if (!input.trim()) return;

        const userMessage = { role: 'user', content: input };
        setMessages(prev => [...prev, userMessage]);
        setInput('');
        setSelectedText('');
        setIsLoading(true);

        try {
            const response = await fetch(`${API_BASE_URL}/chat`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    message: input,
                    ...(token && { token })
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
            {/* Floating Tooltip for Text Selection */}
            {showTooltip && tooltipPosition && (
                <div
                    ref={tooltipRef}
                    className={styles.selectionTooltip}
                    style={{
                        top: `${tooltipPosition.top}px`,
                        left: `${tooltipPosition.left}px`
                    }}
                >
                    <div className={styles.tooltipArrow}></div>
                    <div className={styles.tooltipButtons}>
                        <button
                            onClick={() => handleQuickAction('explain')}
                            title="Explain (Ctrl+Enter)"
                        >
                            💡 Explain
                        </button>
                        <button onClick={() => handleQuickAction('summarize')}>
                            📝 Summarize
                        </button>
                        <button onClick={() => handleQuickAction('translate')}>
                            🌐 Translate
                        </button>
                        <button onClick={() => handleQuickAction('define')}>
                            📖 Define
                        </button>
                    </div>
                    <div className={styles.tooltipHint}>
                        Press <kbd>Ctrl+Enter</kbd> to explain
                    </div>
                </div>
            )}

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
                                ✨ Selected: "{selectedText.substring(0, 50)}{selectedText.length > 50 ? '...' : ''}"
                            </div>
                        )}
                        <div className={styles.inputRow}>
                            <input
                                type="text"
                                value={input}
                                onChange={(e) => setInput(e.target.value)}
                                onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                                placeholder="Ask about the textbook..."
                            />
                            <button onClick={sendMessage}>Send</button>
                        </div>
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
