# Data Model: Frontend-Backend Integration (Chatbot)

## Chat Message Entity

**Definition**: Represents a communication between user and assistant in the chat interface

**Fields**:
- `id`: Unique identifier for the message (string, required)
- `sender`: Either "user" or "assistant" (string, required)
- `content`: The text content of the message (string, required)
- `timestamp`: When the message was created (ISO date string, required)
- `status`: Message status ("sent", "delivered", "error") (string, optional)

**Validation Rules**:
- Content must be 1-2000 characters
- Sender must be one of the allowed values
- Timestamp must be in valid ISO format

## Chat Session Entity

**Definition**: Represents a conversation session between user and the chatbot

**Fields**:
- `sessionId`: Unique identifier for the session (string, required)
- `messages`: Array of ChatMessage entities (array of ChatMessage, required)
- `createdAt`: When the session was created (ISO date string, required)
- `lastActiveAt`: When the session was last active (ISO date string, required)

**Validation Rules**:
- SessionId must be unique
- Messages array must not exceed 100 messages
- createdAt must be before lastActiveAt

## Query Request Entity

**Definition**: The request payload sent from frontend to backend

**Fields**:
- `question`: The user's question to be answered (string, required)
- `selected_text`: Optional text selected by user on the page (string, optional)
- `sessionId`: Optional session identifier (string, optional)

**Validation Rules**:
- Question must be 1-1000 characters
- Selected text must be 0-2000 characters
- Question and selected_text cannot both be empty

## Query Response Entity

**Definition**: The response payload received from backend

**Fields**:
- `answer`: The generated answer based on retrieved content (string, required)
- `sources`: List of sources used to generate the answer (array of Source objects, required)
- `query_id`: Unique identifier for the query (string, required)
- `response_timestamp`: Timestamp when the response was generated (ISO date string, required)
- `confidence_score`: Optional confidence level in the answer (number 0-1, optional)

**Validation Rules**:
- Answer must be 50-5000 characters
- Sources array must have at least 1 item
- Confidence score must be between 0 and 1 if provided

## Source Entity

**Definition**: Represents a source document used to generate the answer

**Fields**:
- `source_url`: URL of the source document (string, required)
- `document_title`: Title of the source document (string, required)
- `excerpt`: Relevant excerpt from the source (string, required)
- `similarity_score`: Relevance score of this source (number 0-1, required)

**Validation Rules**:
- All fields are required
- Similarity score must be between 0 and 1
- URLs must be valid

## Frontend State Entity

**Definition**: Represents the state of the chatbot UI component

**Fields**:
- `messages`: Array of ChatMessage entities (array of ChatMessage, required)
- `inputText`: Current text in the input field (string, required)
- `isLoading`: Whether an API request is in progress (boolean, required)
- `error`: Error message if any (string, optional)
- `selectedText`: Text currently selected on the page (string, optional)
- `isOpen`: Whether the chat widget is open (boolean, required)

**Validation Rules**:
- InputText must be 0-1000 characters
- Error message must be provided when error state is true