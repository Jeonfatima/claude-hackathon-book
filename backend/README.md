# RAG Agent API

This is a Retrieval-Augmented Generation (RAG) agent that answers questions based on book content stored in a vector database.

## Overview

The RAG Agent API allows users to ask questions about book content and receive accurate answers based on retrieved information from a vector database, with proper source attribution.

## Features

- Question answering based on retrieved content
- Source attribution with URLs and metadata
- API key authentication
- Rate limiting
- Health check endpoint
- Error handling with standardized responses

## Setup

1. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Create a `.env` file with the following variables:
   ```env
   GEMINI_API_KEY=your_gemini_api_key_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   COHERE_API_KEY=your_cohere_api_key_here
   REQUIRED_API_KEY=your_required_api_key_here  # Optional, for custom API key validation
   ```

3. Run the application:
   ```bash
   uvicorn main:app --reload --host 0.0.0.0 --port 8000
   ```

## API Endpoints

### POST /query

Submit a question to the RAG agent.

**Headers:**
- `X-API-Key`: Your API key for authentication

**Request Body:**
```json
{
  "question": "Your question here"
}
```

**Response:**
```json
{
  "answer": "The generated answer",
  "sources": [
    {
      "source_url": "URL of the source",
      "document_title": "Title of the document",
      "excerpt": "Relevant excerpt from the source",
      "similarity_score": 0.92
    }
  ],
  "query_id": "Unique query identifier",
  "response_timestamp": "2025-12-17T10:30:00Z",
  "confidence_score": 0.85
}
```

### GET /health

Check the health status of the service.

**Response:**
```json
{
  "status": "healthy",
  "details": {
    "timestamp": "2025-12-17T10:30:00Z",
    "services": {
      "retrieval_service": true,
      "generation_service": true
    }
  }
}
```

## Rate Limiting

The API is rate-limited to 5 requests per minute per IP address.

## Error Handling

The API returns standardized error responses:

```json
{
  "error_code": "ERROR_CODE",
  "message": "Error message",
  "timestamp": "2025-12-17T10:30:00Z",
  "details": {}
}
```

## Architecture

The system consists of:
- **Retrieval Service**: Connects to Qdrant Cloud and performs vector similarity search
- **Generation Service**: Interfaces with Google Gemini API for answer generation
- **RAG Service**: Coordinates retrieval and generation with proper grounding in retrieved content
- **FastAPI Application**: Provides the REST API interface