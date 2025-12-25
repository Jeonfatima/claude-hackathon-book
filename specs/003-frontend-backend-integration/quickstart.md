# Quickstart Guide: Frontend-Backend Chat Integration

## Prerequisites
- Node.js 18+ for Docusaurus frontend
- Python 3.9+ for FastAPI backend
- Access to required API keys (Cohere, Qdrant, etc.)

## Backend Setup

1. **Start the backend server:**
   ```bash
   cd backend
   python -m uvicorn main:app --host 0.0.0.0 --port 8000 --reload
   ```

2. **Verify the backend is running:**
   - Navigate to `http://localhost:8000/` to see the API root message
   - Test the health endpoint at `http://localhost:8000/health`
   - Verify the query endpoint at `http://localhost:8000/query` (POST request)

## Frontend Setup

1. **Install dependencies:**
   ```bash
   cd textbook
   npm install
   ```

2. **Configure backend API URL:**
   - Create or update `.env` file in the textbook directory:
   ```
   REACT_APP_BACKEND_URL=http://localhost:8000
   ```

3. **Start the Docusaurus development server:**
   ```bash
   npm run start
   ```

## Integration Steps

1. **Create the chatbot component:**
   - Create a new React component for the chat interface
   - Place it in `textbook/src/components/Chatbot/`

2. **Add text selection functionality:**
   - Implement the text selection capture using `window.getSelection()`
   - Store selected text in component state

3. **Connect to the backend API:**
   - Use fetch or axios to make requests to the `/query` endpoint
   - Handle loading states and error conditions

4. **Integrate with Docusaurus layout:**
   - Add the chatbot component to the Docusaurus layout
   - Ensure it appears on all book pages without breaking existing functionality

## Testing the Integration

1. **Verify API communication:**
   - Open browser developer tools
   - Make a test query through the chat interface
   - Check the Network tab to confirm API requests/responses

2. **Test assistant name display:**
   - Verify that responses appear with "Fatima Salman" as the assistant name

3. **Test text selection feature:**
   - Select text on a book page
   - Initiate a chat with the selected text
   - Verify the context is passed to the backend

## Environment Configuration

### Frontend Environment Variables
```
REACT_APP_BACKEND_URL=http://localhost:8000  # Backend API URL
REACT_APP_ASSISTANT_NAME=Fatima Salman       # Assistant display name
```

### Backend Environment Variables
Ensure your backend `.env` file has:
```
COHERE_API_KEY=your_cohere_api_key
QDRANT_HOST=your_qdrant_host
QDRANT_API_KEY=your_qdrant_api_key
QDRANT_PORT=6333
```

## Troubleshooting

### Common Issues

**CORS Errors:**
- Ensure backend is configured with correct origins
- Check that frontend and backend are running on compatible ports

**API Key Issues:**
- Verify all required API keys are set in the backend `.env` file
- Confirm API keys have appropriate permissions

**Text Selection Not Working:**
- Verify the text selection JavaScript is properly attached to page elements
- Check for JavaScript errors in browser console

### Verification Steps

1. **Backend health check:**
   ```bash
   curl http://localhost:8000/health
   ```

2. **Direct API test:**
   ```bash
   curl -X POST http://localhost:8000/query \
        -H "Content-Type: application/json" \
        -d '{"question": "What is ROS 2?"}'
   ```

3. **Frontend console:**
   - Open browser developer tools
   - Check for any JavaScript errors
   - Verify network requests to backend API

## Next Steps

1. Implement the full chatbot UI with "Fatima Salman" branding
2. Add advanced features like conversation history
3. Optimize for production deployment
4. Add analytics and usage tracking