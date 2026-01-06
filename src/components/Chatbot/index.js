import React, { useState, useEffect, useRef } from 'react';
import { Send, X, Copy, Bot, User } from 'lucide-react';
import clsx from 'clsx';
import styles from './Chatbot.module.css';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const Chatbot = () => {
  const { siteConfig } = useDocusaurusContext();
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [showSelectionBadge, setShowSelectionBadge] = useState(false);
  const [isClient, setIsClient] = useState(false);
  const messagesEndRef = useRef(null);
  const selectionTimeoutRef = useRef(null);

  // Get the API URL from site config
  const chatbotApiUrl = siteConfig.customFields?.CHATBOT_API_URL || 'http://localhost:8000';

  // Check if running in browser environment
  useEffect(() => {
    setIsClient(true);
  }, []);

  // Track text selection (only in browser)
  useEffect(() => {
    if (!isClient) return;

    const handleSelection = () => {
      const selection = window.getSelection ? window.getSelection() : null;
      const selectedText = selection ? selection.toString().trim() : '';

      if (selectedText) {
        setSelectedText(selectedText);
        setShowSelectionBadge(true);

        // Clear any existing timeout
        if (selectionTimeoutRef.current) {
          clearTimeout(selectionTimeoutRef.current);
        }

        // Hide the badge after 5 seconds
        selectionTimeoutRef.current = setTimeout(() => {
          setShowSelectionBadge(false);
        }, 5000);
      }
    };

    document.addEventListener('mouseup', handleSelection);

    return () => {
      document.removeEventListener('mouseup', handleSelection);
      if (selectionTimeoutRef.current) {
        clearTimeout(selectionTimeoutRef.current);
      }
    };
  }, [isClient]);

  // Scroll to bottom of messages (only in browser)
  useEffect(() => {
    if (isClient && messagesEndRef.current) {
      messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
    }
  }, [messages, isClient]);

  const sendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    const userMessage = {
      id: Date.now(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Add the selected text if available
      const response = await fetch(`${chatbotApiUrl}/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_query: inputValue,
          user_id: 'user_' + Date.now(),
          selected_text: selectedText || null
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      const botMessage = {
        id: Date.now() + 1,
        text: data.answer,
        sender: 'bot',
        sources: data.sources,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, botMessage]);

      // Clear the selected text after using it
      setSelectedText('');
      setShowSelectionBadge(false);
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage = {
        id: Date.now() + 1,
        text: 'Error: Could not connect to the chat service. Please check if the backend is running.',
        sender: 'bot',
        error: true,
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const clearChat = () => {
    setMessages([]);
    setSelectedText('');
    setShowSelectionBadge(false);
  };

  return (
    <>
      {/* Floating Neural Link Button - only render if client-side */}
      {isClient && (
        <div className={styles.neuralLinkButton} onClick={toggleChat}>
          <div className={styles.neuralPulse}></div>
          <div className={styles.neuralIcon}>🔗</div>
        </div>
      )}

      {/* Chatbot Window - only render if client-side and open */}
      {isClient && isOpen && (
        <div className={styles.chatbotWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.chatTitle}>
              <Bot size={20} className={styles.botIcon} />
              <span>Neural Link Assistant</span>
            </div>
            <div className={styles.chatControls}>
              <button
                onClick={clearChat}
                className={styles.clearButton}
                title="Clear chat"
              >
                ✕
              </button>
              <button
                onClick={toggleChat}
                className={styles.closeButton}
              >
                <X size={16} />
              </button>
            </div>
          </div>

          <div className={styles.chatBody}>
            {/* Selection Badge */}
            {showSelectionBadge && (
              <div className={clsx(styles.selectionBadge, styles.glow)}>
                <span>IGNAL CAPTURED</span>
                <div className={styles.badgePulse}></div>
              </div>
            )}

            {/* Messages Container */}
            <div className={styles.messagesContainer}>
              {messages.length === 0 ? (
                <div className={styles.welcomeMessage}>
                  <p>Neural Link initialized. Ready for input.</p>
                  <p className={styles.welcomeSubtext}>Select text on the page to provide context</p>
                </div>
              ) : (
                messages.map((message) => (
                  <div
                    key={message.id}
                    className={clsx(
                      styles.message,
                      message.sender === 'user' ? styles.userMessage : styles.botMessage
                    )}
                  >
                    <div className={styles.messageContent}>
                      {message.sender === 'user' ? (
                        <User size={16} className={styles.senderIcon} />
                      ) : (
                        <Bot size={16} className={styles.senderIcon} />
                      )}
                      <div className={styles.messageText}>
                        <p>{message.text}</p>
                        {message.sources && message.sources.length > 0 && (
                          <div className={styles.sources}>
                            <details className={styles.sourcesDetails}>
                              <summary>Sources</summary>
                              {message.sources.map((source, index) => (
                                <div key={index} className={styles.sourceItem}>
                                  <span className={styles.sourceFile}>📄 {source.source_file}</span>
                                  <span className={styles.sourceScore}>Score: {source.score?.toFixed(3)}</span>
                                </div>
                              ))}
                            </details>
                          </div>
                        )}
                      </div>
                    </div>
                  </div>
                ))
              )}

              {isLoading && (
                <div className={styles.message}>
                  <div className={styles.messageContent}>
                    <Bot size={16} className={styles.senderIcon} />
                    <div className={styles.loadingMessage}>
                      <div className={clsx(styles.typingIndicator, styles.glitch)}>
                        <span className={styles.processingText}>ECRYPTING</span>
                        <span className={styles.dots}>
                          <span>.</span>
                          <span>.</span>
                          <span>.</span>
                        </span>
                      </div>
                    </div>
                  </div>
                </div>
              )}
              <div ref={messagesEndRef} />
            </div>
          </div>

          <div className={styles.chatInput}>
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Enter your query..."
              className={styles.inputField}
              rows={1}
            />
            <button
              onClick={sendMessage}
              disabled={!inputValue.trim() || isLoading}
              className={clsx(
                styles.sendButton,
                (!inputValue.trim() || isLoading) && styles.sendButtonDisabled
              )}
            >
              <Send size={16} />
            </button>
          </div>
        </div>
      )}
    </>
  );
};

export default Chatbot;