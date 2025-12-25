# Chatbot Integration Guide

This document provides an overview of the chatbot integration in the Docusaurus textbook, named "Fatima Salman".

## Overview

The chatbot integration provides users with an AI assistant that can answer questions about the book content. The assistant is named "Fatima Salman" and can be accessed via a floating chat button on all pages.

## Features

### 1. Basic Chat Functionality
- Ask questions about the book content
- Receive responses from the AI assistant
- View conversation history with timestamps

### 2. Text Selection Feature
- Select text on any page by highlighting it
- Ask questions specifically about the selected text
- The selected text will be shown as context in the chat

### 3. Assistant Identity
- The assistant is consistently identified as "Fatima Salman"
- All responses are clearly attributed to the assistant

## Technical Architecture

### Frontend Components
- **Chatbot Component** (`src/components/Chatbot/Chatbot.tsx`): Main UI component
- **Chat Context** (`src/contexts/ChatContext.tsx`): State management
- **API Service** (`src/services/api.ts`): Backend communication
- **Type Definitions** (`src/types/chat.ts`): Type interfaces
- **Text Selection Utility** (`src/utils/textSelection.ts`): Text selection capture

### Backend Integration
- Communicates with the backend via the `/query` endpoint
- Supports optional `selected_text` parameter for context-specific queries
- Uses environment variable `REACT_APP_BACKEND_URL` for API endpoint configuration

## How to Use

### Opening the Chat
1. Look for the floating chat button in the bottom-right corner of any page
2. Click the button to open the chat interface

### Asking Questions
1. Type your question in the input field at the bottom of the chat
2. Press Enter or click the send button to submit
3. Wait for the response from "Fatima Salman"

### Using Text Selection
1. Highlight text on the current page
2. The selected text will appear in the chat interface
3. Type a question about the selected text
4. The context will be sent to the backend along with your question

## Configuration

### Environment Variables
The following environment variables should be set in your `.env` file:

```env
REACT_APP_BACKEND_URL=http://localhost:8000  # Backend API URL
REACT_APP_ASSISTANT_NAME=Fatima Salman       # Assistant display name
```

## API Communication

The chatbot communicates with the backend using the following API contract:

### Request
- **Endpoint**: `POST /query`
- **Headers**:
  - `Content-Type: application/json`
  - `Authorization: Bearer <api_key>` (if required)
- **Body**:
```json
{
  "question": "string (1-1000 characters)",
  "selected_text": "string (0-2000 characters, optional)"
}
```

### Response
- **Success (200 OK)**:
```json
{
  "answer": "string (50-5000 characters)",
  "sources": [
    {
      "source_url": "string",
      "document_title": "string",
      "excerpt": "string",
      "similarity_score": "number (0-1)"
    }
  ],
  "query_id": "string",
  "response_timestamp": "string (ISO date format)",
  "confidence_score": "number (0-1, optional)"
}
```

## Error Handling

The chatbot handles various error scenarios:
- Network errors with user-friendly messages
- API errors with appropriate status code information
- Validation errors for input parameters
- Loading states during API requests

## Accessibility

The chat interface includes:
- Proper ARIA labels for screen readers
- Keyboard navigation support
- Sufficient color contrast
- Semantic HTML structure

## Development

### Running Locally
1. Start the backend server: `cd backend && python -m uvicorn main:app --reload`
2. Start the Docusaurus frontend: `cd textbook && npm run start`
3. The chatbot will automatically connect to the backend at `http://localhost:8000`

### Testing
The chatbot functionality can be tested by:
1. Opening the chat interface on any page
2. Asking questions about the book content
3. Verifying responses are attributed to "Fatima Salman"
4. Testing the text selection feature
5. Verifying error handling works correctly

## Troubleshooting

### Common Issues
- **Chat not connecting to backend**: Verify the backend server is running and the URL in `.env` is correct
- **Text selection not working**: Check browser console for JavaScript errors
- **CORS errors**: Ensure the backend has proper CORS configuration for frontend requests