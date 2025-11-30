import React, { useState, useEffect } from 'react';
import styles from './Chatbot.module.css';

const Chatbot = () => {
    const [isOpen, setIsOpen] = useState(false);
    const [messages, setMessages] = useState([
        { role: 'assistant', content: 'Hi! I am your AI Tutor. Ask me anything about the book!' }
    ]);
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
        setSelectedText('');
        setIsLoading(true);

        try {
            const response = await fetch('http://localhost:8001/chat', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ message: input }),
            });

            const data = await response.json();
            const botMessage = { role: 'assistant', content: data.response };
            setMessages(prev => [...prev, botMessage]);
        } catch (error) {
            console.error('Error sending message:', error);
            setMessages(prev => [...prev, { role: 'assistant', content: 'Sorry, I encountered an error. Please check if the backend is running.' }]);
        } finally {
            setIsLoading(false);
        }
    };

    return (
        <div className={styles.chatbotContainer}>
            {!isOpen && (
                <button className={styles.chatButton} onClick={toggleChat}>
                    🤖 Chat
                </button>
            )}

            {isOpen && (
                <div className={styles.chatWindow}>
                    <div className={styles.chatHeader}>
                        <h3>AI Tutor</h3>
                        <button onClick={toggleChat}>×</button>
                    </div>
                    <div className={styles.chatMessages}>
                        {messages.map((msg, index) => (
                            <div key={index} className={`${styles.message} ${styles[msg.role]}`}>
                                {msg.content}
                            </div>
                        ))}
                        {isLoading && <div className={styles.loading}>Thinking...</div>}
                    </div>
                    <div className={styles.chatInput}>
                        {selectedText && (
                            <div className={styles.selectedTextIndicator}>
                                📝 Selected: "{selectedText.substring(0, 50)}..."
                            </div>
                        )}
                        <input
                            type="text"
                            value={input}
                            onChange={(e) => setInput(e.target.value)}
                            onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
                            placeholder="Ask a question or select text from the page..."
                        />
                        <button onClick={sendMessage}>Send</button>
                    </div>
                </div>
            )}
        </div>
    );
};

export default Chatbot;
