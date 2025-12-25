# Text Selection Popup Feature

## Overview

The "Ask Chatbot About Selected Text" feature allows users to select text within the textbook content and ask the AI assistant about it directly. When text is selected, a small popup button appears near the selected text with the option to "Ask chatbot about this".

## How It Works

1. **Text Selection**: User selects text within any textbook content
2. **Popup Appearance**: A small popup button appears near the selected text
3. **Ask Action**: User clicks the popup button to ask the chatbot about the selected text
4. **Chat Window**: The chat window opens with the selected text context available
5. **Response**: The AI assistant responds based on the selected text

## Implementation Details

### Components

- `TextSelectionPopup.tsx`: The popup component that appears when text is selected
- `TextSelectionPopup.module.css`: Styles for the popup component
- `Chatbot.tsx`: Updated to include popup functionality and improved text selection handling

### Key Features

- **Positioning**: Popup appears above the selected text with proper viewport boundary handling
- **Context**: Selected text is passed to the chatbot as context for more accurate responses
- **UI**: Clean, modern design that fits with the existing chatbot theme
- **Accessibility**: Proper ARIA labels and keyboard navigation support

### Technical Details

- The popup uses `window.getSelection()` to detect selected text
- Positioning is calculated using `getBoundingClientRect()` for accurate placement
- Viewport boundaries are respected to ensure the popup remains visible
- The selected text is stored in the chat context and sent to the backend API

## API Integration

The selected text is sent to the backend API using the existing `/query` endpoint with the `selected_text` parameter:

```typescript
const response = await apiService.query({
  question: currentInput || 'Explain the selected text',
  selected_text: localSelectedText || undefined,
});
```

## Styling

The popup features:
- Modern gradient button design
- Smooth animations and transitions
- Responsive positioning that adapts to viewport boundaries
- Consistent styling with the existing chatbot theme

## User Experience

1. Select any text in the textbook content
2. A "Ask chatbot about this" button will appear above the selected text
3. Click the button to open the chatbot with the selected text as context
4. Ask questions about the selected content
5. The chatbot will provide answers based on the selected text context