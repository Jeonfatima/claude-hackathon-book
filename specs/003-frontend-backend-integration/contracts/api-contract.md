# API Contract: Frontend-Backend Chat Integration

## Overview
This document defines the API contract between the Docusaurus frontend and the FastAPI backend for the chatbot functionality.

## Base URL
The backend API is accessible at a configurable base URL (e.g., `http://localhost:8000` during development).

## Authentication
The API uses API key authentication via the `Authorization` header:
```
Authorization: Bearer <api_key>
```

## Endpoints

### POST /query
Submit a question to the RAG system and receive an answer based on book content.

#### Request
**Headers:**
- `Content-Type: application/json`
- `Authorization: Bearer <api_key>` (if required by deployment)

**Body:**
```json
{
  "question": "string (1-1000 characters)",
  "selected_text": "string (0-2000 characters, optional)"
}
```

#### Response
**Success (200 OK):**
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

**Error Responses:**
- `400 Bad Request`: Invalid input (question too short, etc.)
- `422 Unprocessable Entity`: Validation error
- `429 Too Many Requests`: Rate limit exceeded
- `500 Internal Server Error`: Server error
- `503 Service Unavailable`: External service connection error

#### Example Request
```json
{
  "question": "What is ROS 2?",
  "selected_text": "This section discusses the Robot Operating System version 2"
}
```

#### Example Response
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is a set of software libraries and tools that help you build robot applications. It provides hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more...",
  "sources": [
    {
      "source_url": "https://hackathon-claude-textbook.vercel.app/module-1/ros2-introduction",
      "document_title": "ROS 2 Introduction",
      "excerpt": "The Robot Operating System (ROS) is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior...",
      "similarity_score": 0.92
    }
  ],
  "query_id": "query_12345",
  "response_timestamp": "2025-12-18T10:30:00.123Z",
  "confidence_score": 0.85
}
```

## Frontend-to-Backend Communication Flow

1. User types question in chat UI
2. If user has selected text, capture that text as context
3. Frontend sends POST request to `/query` with question and optional selected_text
4. Backend processes query using RAG system
5. Backend returns response with answer and sources
6. Frontend displays response in chat UI

## Error Handling

### Frontend Error Handling
- Network errors: Display user-friendly message
- API errors: Show appropriate error based on status code
- Validation errors: Show specific validation messages

### Backend Error Responses
- `QUERY_TOO_SHORT`: Question is too short
- `NO_RELEVANT_CONTENT`: No relevant content found in knowledge base
- `LLM_ERROR`: Error with language model
- `VECTOR_DB_ERROR`: Error with vector database
- `SERVICE_UNAVAILABLE`: External service unavailable
- `INVALID_INPUT`: Invalid input parameters
- `RATE_LIMIT_EXCEEDED`: Rate limit exceeded

## Rate Limiting
- Default: 5 requests per minute per IP address
- Applied to the `/query` endpoint
- Returns 429 status code when exceeded

## CORS Policy
- Origins: `*` (for development) - should be restricted in production
- Methods: All HTTP methods allowed
- Headers: All headers allowed
- Credentials: Allowed