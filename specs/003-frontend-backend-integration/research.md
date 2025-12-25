# Research: Frontend-Backend Integration (Chatbot)

## Decision: CORS Configuration
- **Rationale**: Backend already has CORS middleware configured with `allow_origins=["*"]` which allows requests from any origin. This is suitable for development but should be restricted in production.
- **Implementation**: No changes needed to backend CORS configuration for basic integration.

## Decision: API Endpoint Compatibility
- **Rationale**: The existing `/query` endpoint in `main.py` already supports the required functionality with `question` and optional `selected_text` fields in the `QueryRequest` model.
- **Implementation**: Frontend will send requests to POST `/query` with the appropriate payload structure.

## Decision: Assistant Name Display
- **Rationale**: The assistant name "Fatima Salman" needs to be displayed in the frontend UI, not necessarily returned by the backend.
- **Implementation**: Hardcode the assistant name in the frontend chat component to ensure it always appears as "Fatima Salman".

## Decision: Chatbot UI Positioning
- **Rationale**: Docusaurus supports custom components that can be integrated as floating widgets or sidebars without breaking existing functionality.
- **Alternatives Considered**:
  1. Floating widget (recommended) - appears as a chat bubble that expands when clicked
  2. Sidebar integration - permanent panel on the side of content
  3. Bottom docked panel - always visible at bottom of screen
- **Choice**: Floating widget approach for minimal visual impact on existing content.

## Decision: Text Selection Implementation
- **Rationale**: Modern browsers provide the `window.getSelection()` API to capture user-selected text.
- **Implementation**: Add event listeners to capture text selection and store in component state for use in chat context.

## Decision: Error Handling Strategy
- **Rationale**: Network requests can fail, and users need appropriate feedback.
- **Implementation**: Implement proper error states in UI with user-friendly messages when backend requests fail.

## Decision: Loading States
- **Rationale**: API requests take time, and users need visual feedback during processing.
- **Implementation**: Show loading indicators while waiting for backend responses to improve user experience.