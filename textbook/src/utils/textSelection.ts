/**
 * Utility functions for capturing text selection on web pages
 */

/**
 * Get the currently selected text from the page
 * @returns The selected text as a string, or empty string if no text is selected
 */
export const getSelectedText = (): string => {
  return window.getSelection()?.toString().trim() || '';
};

/**
 * Get the currently selected text along with its context information
 * @returns Object containing the selected text and its context
 */
export const getSelectedTextWithContext = (): {
  text: string;
  element?: HTMLElement;
  container?: HTMLElement;
} => {
  const selection = window.getSelection();
  if (!selection || selection.toString().trim() === '') {
    return { text: '' };
  }

  const selectedText = selection.toString().trim();
  let element: HTMLElement | undefined;
  let container: HTMLElement | undefined;

  // Get the anchor and focus nodes to identify the selected elements
  if (selection.anchorNode) {
    element = (selection.anchorNode.nodeType === Node.ELEMENT_NODE
      ? selection.anchorNode as HTMLElement
      : selection.anchorNode.parentElement) || undefined;
  }

  // Find a reasonable container for context
  if (element) {
    // Look for a containing element that might provide context
    container = element.closest('p, div, section, article, .markdown') as HTMLElement || element.parentElement;
  }

  return {
    text: selectedText,
    element,
    container
  };
};

/**
 * Add event listeners for text selection
 * @param callback Function to call when text is selected
 * @returns Function to remove the event listeners
 */
export const addTextSelectionListener = (callback: (selectedText: string) => void): (() => void) => {
  const handleSelection = () => {
    const selectedText = getSelectedText();
    if (selectedText) {
      callback(selectedText);
    }
  };

  // Add event listeners for different selection events
  document.addEventListener('mouseup', handleSelection);
  document.addEventListener('keyup', (e) => {
    if (e.key === 'Escape') {
      callback(''); // Clear selection when Escape is pressed
    }
  });

  // Return a function to remove the event listeners
  return () => {
    document.removeEventListener('mouseup', handleSelection);
    document.removeEventListener('keyup', (e) => {
      if (e.key === 'Escape') {
        callback(''); // Clear selection when Escape is pressed
      }
    });
  };
};

/**
 * Validate selected text according to the requirements
 * @param text The text to validate
 * @returns True if the text is valid, false otherwise
 */
export const validateSelectedText = (text: string): boolean => {
  // Check if text length is within the required range (0-2000 characters)
  return text.length >= 0 && text.length <= 2000;
};

/**
 * Sanitize selected text to remove potentially harmful content
 * @param text The text to sanitize
 * @returns Sanitized text
 */
export const sanitizeSelectedText = (text: string): string => {
  // Remove extra whitespace and normalize the text
  return text.replace(/\s+/g, ' ').trim();
};