import React, { useState, useEffect, useRef } from 'react';
import { useChat } from '../../contexts/ChatContext';
import styles from './TextSelectionPopup.module.css';

interface TextSelectionPopupProps {
  selectedText: string;
  isOpen: boolean;
  onClose: () => void;
}

const TextSelectionPopup: React.FC<TextSelectionPopupProps> = ({
  selectedText,
  isOpen,
  onClose
}) => {
  const { setOpen: setChatOpen, setSelectedText } = useChat();
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [positionClass, setPositionClass] = useState('');
  const popupRef = useRef<HTMLDivElement>(null);

  // Update position when text is selected
  useEffect(() => {
    if (isOpen && selectedText) {
      const selection = window.getSelection();
      if (selection && selection.rangeCount > 0) {
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        // Calculate viewport boundaries
        const viewportWidth = window.innerWidth;
        const popupWidth = 160; // Approximate width of the popup
        const buffer = 10; // Buffer space from edges

        // Calculate base position (above the selection)
        let x = rect.left + window.scrollX + rect.width / 2; // Center over the selection
        let y = rect.top + window.scrollY - 50; // 50px above the selection

        // Adjust x position to stay within viewport
        if (x - popupWidth / 2 < buffer) {
          // Too close to left edge
          x = buffer + popupWidth / 2;
          setPositionClass(styles.leftEdge);
        } else if (x + popupWidth / 2 > viewportWidth - buffer) {
          // Too close to right edge
          x = viewportWidth - buffer - popupWidth / 2;
          setPositionClass(styles.rightEdge);
        } else {
          setPositionClass('');
        }

        // Adjust y position if needed
        if (y < 20) {
          // If too close to top, position below the selection instead
          y = rect.bottom + window.scrollY + 10;
        }

        setPosition({
          x,
          y
        });
      }
    }
  }, [isOpen, selectedText]);

  const handleAskClick = () => {
    // Set the selected text in the chat context
    setSelectedText(selectedText);
    // Close the popup
    onClose();
    // Open the chat window
    setChatOpen(true);
  };

  if (!isOpen || !selectedText) {
    return null;
  }

  return (
    <div
      ref={popupRef}
      className={`${styles.popup} ${positionClass}`}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
      role="dialog"
      aria-label="Text selection popup"
    >
      <button
        className={styles.popupButton}
        onClick={handleAskClick}
        aria-label="Ask chatbot about selected text"
      >
        Ask chatbot about this
      </button>
    </div>
  );
};

export default TextSelectionPopup;