import React, { useState, useEffect } from 'react';
import OriginalLayout from '@theme-original/Layout';
import { ChatProvider } from '../contexts/ChatContext';
import Chatbot from '../components/Chatbot/Chatbot';
import styles from '../components/Chatbot/Chatbot.module.css'; // Import the styles
import { FiMessageSquare } from 'react-icons/fi';

type LayoutProps = {
  children: React.ReactNode;
  [key: string]: any;
};

const LayoutWrapper: React.FC<LayoutProps> = (props) => {
  const [isChatOpen, setIsChatOpen] = useState(false);

  // Check if chat was previously open in localStorage
  useEffect(() => {
    const savedChatState = localStorage.getItem('chatbot-open');
    if (savedChatState) {
      setIsChatOpen(JSON.parse(savedChatState));
    }
  }, []);

  // Save chat state to localStorage
  useEffect(() => {
    localStorage.setItem('chatbot-open', JSON.stringify(isChatOpen));
  }, [isChatOpen]);

  return (
    <ChatProvider>
      <OriginalLayout {...props}>
        {props.children}

        {/* Floating chat button when chat is closed */}
        {!isChatOpen && (
          <button
            className={styles.floatingButton}
            onClick={() => setIsChatOpen(true)}
            aria-label="Open AI assistant chat"
            title="Open AI assistant"
          >
            <FiMessageSquare aria-hidden="true" />
          </button>
        )}

        {/* Chatbot component */}
        <div style={{ display: isChatOpen ? 'block' : 'none' }}>
          <Chatbot />
        </div>
      </OriginalLayout>
    </ChatProvider>
  );
};

export default LayoutWrapper;