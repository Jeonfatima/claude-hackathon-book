import React, { useState, useEffect, useRef } from 'react';
import { useChat } from '../../contexts/ChatContext';
import { apiService } from '../../services/api';
import { ChatMessage } from '../../types/chat';
import styles from './Chatbot.module.css';
import TextSelectionPopup from './TextSelectionPopup';
import { FiX, FiSend, FiMessageSquare, FiLoader } from 'react-icons/fi';

const Chatbot: React.FC = () => {
  const {
    state,
    setInputText,
    setLoading,
    setError,
    addMessage,
    setOpen,
    setSelectedText
  } = useChat();

  const [mounted, setMounted] = useState(false);
  const [localSelectedText, setLocalSelectedText] = useState<string>('');
  const [showTextPopup, setShowTextPopup] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // 1. Prevent Hydration Mismatch (The "Vanishing" fix)
  useEffect(() => {
    setMounted(true);
  }, []);

  // 2. Capture selected text and show popup
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection()?.toString().trim();
      if (selectedText && selectedText.length > 0) {
        setLocalSelectedText(selectedText);
        setShowTextPopup(true);
      } else {
        setShowTextPopup(false);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      setShowTextPopup(false);
    };
  }, []);

  // 3. Auto-scroll messages
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [state.messages]);

  const handleSendMessage = async () => {
    if (!state.inputText.trim() && !localSelectedText) return;

    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      sender: 'user',
      content: state.inputText || `Question about: ${localSelectedText}`,
      timestamp: new Date().toISOString(),
    };

    addMessage(userMessage);
    const currentInput = state.inputText;
    setInputText('');
    setError(undefined); // Clear any previous errors when sending a new message

    try {
      setLoading(true);

      // If there's no user input, create a question about the selected text
      const question = currentInput || `Explain the following text: ${localSelectedText.substring(0, 200)}${localSelectedText.length > 200 ? '...' : ''}`;

      const response = await apiService.query({
        question: question,
        selected_text: localSelectedText || undefined,
      });

      addMessage({
        id: `resp_${Date.now()}`,
        sender: 'assistant',
        content: response.answer,
        timestamp: new Date().toISOString(),
      });
    } catch (error) {
      setError(error instanceof Error ? error.message : 'Connection failed');
      console.error('Error sending message:', error);
    } finally {
      setLoading(false);
      // Clear the selected text after sending
      setLocalSelectedText('');
      setSelectedText('');
    }
  };

  const handlePopupClose = () => {
    setShowTextPopup(false);
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  // Don't render anything until the browser is ready
  if (!mounted) return null;

  return (
    <>
      {/* TEXT SELECTION POPUP */}
      <TextSelectionPopup
        selectedText={localSelectedText}
        isOpen={showTextPopup}
        onClose={handlePopupClose}
      />

      {/* FLOATING BUTTON: Only shows when chat is closed */}
      {!state.isOpen && (
        <button
          className={styles.floatingButton}
          onClick={() => setOpen(true)}
          aria-label="Open chat"
        >
          <FiMessageSquare />
        </button>
      )}

      {/* CHAT WINDOW */}
      <div
        className={`${styles.chatContainer} ${state.isOpen ? styles.open : styles.closed}`}
        role="complementary"
      >
        {/* Header */}
        <div className={styles.chatHeader}>
          <div className={styles.headerTitle}>
            <FiMessageSquare className={styles.headerIcon} />
            <span className={styles.assistantName}>AI Assistant</span>
          </div>
          <button className={styles.closeButton} onClick={() => setOpen(false)}>
            <FiX />
          </button>
        </div>

        {/* Messages */}
        <div className={styles.chatMessages}>
          {state.messages.map((message) => (
            <div key={message.id} className={`${styles.message} ${styles[message.sender]}`}>
              <div className={styles.messageContent}>{message.content}</div>
              <div className={styles.messageTimestamp}>
                {new Date(message.timestamp).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
              </div>
            </div>
          ))}
          {state.error && (
            <div className={`${styles.message} ${styles.assistant}`}>
              <div className={styles.messageContent}>
                <span style={{color: 'red'}}>Error: {state.error}</span>
              </div>
            </div>
          )}
          {state.isLoading && (
            <div className={`${styles.message} ${styles.assistant}`}>
              <FiLoader className={styles.spinning} />
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        {/* Input Area */}
        <div className={styles.inputArea}>
          {localSelectedText && (
            <div className={styles.selectedTextPreview}>
              <span>Selected text active</span>
              <button onClick={() => {
                setLocalSelectedText('');
                setSelectedText('');
              }}>Clear</button>
            </div>
          )}
          <div className={styles.inputContainer}>
            <textarea
              className={styles.messageInput}
              value={state.inputText}
              onChange={(e) => setInputText(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              rows={1}
            />
            <button
              className={styles.sendButton}
              onClick={handleSendMessage}
              disabled={state.isLoading}
            >
              <FiSend />
            </button>
          </div>
        </div>
      </div>
    </>
  );
};

export default Chatbot;