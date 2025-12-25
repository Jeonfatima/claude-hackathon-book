# Implementation Plan: Frontend-Backend Integration (Chatbot)

## Technical Context

- **Frontend**: Docusaurus book in `/textbook` directory with React-based components
- **Backend**: FastAPI RAG chatbot in `/backend` directory with existing `/query` endpoint
- **Current API**: POST `/query` accepts `question` and optional `selected_text` fields
- **Assistant Name**: Must be "Fatima Salman" as specified
- **Spec Status**: Specs 1-3 are complete, no modifications allowed to existing logic

## Constitution Check

Based on the project constitution, this plan aligns with:
- **Modular Architecture**: Adding chatbot as a modular component
- **Test-First**: Implementation will include proper testing
- **Performance & Scalability**: API rate limiting already implemented (5/minute)
- **Security-First**: API key authentication already in place

## Implementation Plan

### Phase 1: Backend Preparation

**1.1 Verify CORS Configuration**
- Confirm that CORS is properly configured to allow requests from the Docusaurus frontend
- Current CORS settings in `main.py:51-58` use `allow_origins=["*"]` which should work for development

**1.2 Verify API Endpoint Compatibility**
- Confirm that existing `/query` endpoint supports the frontend requirements
- Endpoint already accepts `question` and `selected_text` fields as needed
- Response format is already compatible with frontend display needs

### Phase 2: Frontend Implementation

**2.1 Create Chatbot UI Component**
- Create a React component for the chatbot interface
- Implement "Fatima Salman" as the assistant name in the UI
- Add floating widget or sidebar integration with Docusaurus
- Include loading states and error handling

**2.2 Implement Text Selection Capture**
- Add JavaScript to capture user-selected text on the page
- Create UI elements to initiate chat with selected text context
- Store selected text in component state for context

**2.3 Create API Communication Layer**
- Implement fetch API calls to communicate with backend
- Create request payload with question and selected text
- Handle loading states during API requests
- Implement error handling for failed requests

**2.4 Integrate with Docusaurus Layout**
- Add chatbot component to the Docusaurus layout
- Ensure it doesn't break existing book page functionality
- Make it accessible without interfering with reading experience

### Phase 3: Integration & Testing

**3.1 Frontend-Backend Integration**
- Connect frontend chatbot UI to backend `/query` endpoint
- Test end-to-end functionality with various questions
- Verify selected text context is properly passed to backend

**3.2 User Experience Testing**
- Test chatbot functionality across different book pages
- Verify "Fatima Salman" appears correctly as assistant name
- Test error scenarios and loading states
- Validate that existing book functionality remains intact

**3.3 Performance Validation**
- Ensure page load times don't exceed 10% increase as per spec
- Test API response times under normal conditions
- Verify 95% success rate for chat requests

## Data Model

**Frontend State:**
- `messages`: Array of chat messages (user and assistant)
- `inputText`: Current user input
- `isLoading`: Boolean for API request state
- `error`: Error message if request fails
- `selectedText`: Text selected by user on current page

**API Request:**
```
{
  "question": string,
  "selected_text": string (optional)
}
```

**API Response:**
```
{
  "answer": string,
  "sources": Array of Source objects,
  "query_id": string,
  "response_timestamp": string,
  "confidence_score": number (optional)
}
```

## API Contracts

**Frontend → Backend:**
- Endpoint: `POST /query`
- Headers: `Content-Type: application/json`, `Authorization: Bearer <api_key>` (if required)
- Request body: `{ "question": string, "selected_text": string }`
- Response: `200 OK` with QueryResponse object or appropriate error codes

## Quickstart Guide

1. **Backend Setup**: Ensure backend is running and accessible
2. **Frontend Integration**: Add chatbot component to Docusaurus
3. **Configuration**: Set backend API URL in frontend environment
4. **Testing**: Verify chat functionality and assistant name display

## Agent Context Update

Add the following technologies to the agent context:
- Docusaurus chatbot integration patterns
- React chat UI component best practices
- CORS configuration for frontend-backend communication
- Text selection API usage
- FastAPI client integration in React applications