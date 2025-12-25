# Quickstart Guide: RAG Agent Backend Implementation

## Prerequisites

- Python 3.11+
- pip package manager
- Git
- Access to Google Gemini API key
- Access to Qdrant Cloud API key and URL
- Access to Cohere API key (for embedding consistency)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd hackathon-qwen
```

### 2. Create Virtual Environment
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### 3. Install Dependencies
```bash
pip install fastapi uvicorn python-dotenv qdrant-client google-generativeai pydantic
```

### 4. Create Environment File
Create a `.env` file in the backend directory with the following:
```env
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
COHERE_API_KEY=your_cohere_api_key_here
ENVIRONMENT=development  # or production
LOG_LEVEL=INFO
```

### 5. Project Structure
```
backend/
├── main.py              # FastAPI application entry point
├── requirements.txt     # Python dependencies
├── .env                 # Environment variables
├── services/
│   ├── __init__.py
│   ├── rag_service.py   # Main RAG orchestration
│   ├── retrieval_service.py  # Vector database operations
│   └── generation_service.py # LLM interaction
├── models/
│   ├── __init__.py
│   ├── request_models.py # Pydantic models for requests
│   └── response_models.py # Pydantic models for responses
└── utils/
    ├── __init__.py
    └── config.py        # Configuration loading
```

## Implementation Steps

### Step 1: Set up FastAPI Application (main.py)
```python
from fastapi import FastAPI, Depends, HTTPException, BackgroundTasks
from fastapi.middleware.cors import CORSMiddleware
import uuid
from datetime import datetime

from models.request_models import QueryRequest
from models.response_models import QueryResponse
from services.rag_service import RAGService
from utils.config import get_settings

app = FastAPI(title="RAG Agent API", version="1.0.0")

# Add CORS middleware if needed
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize services
settings = get_settings()
rag_service = RAGService(settings)

@app.post("/query", response_model=QueryResponse)
async def query_endpoint(request: QueryRequest):
    """Process a user query and return an answer based on retrieved content."""
    try:
        # Generate a unique query ID
        query_id = str(uuid.uuid4())

        # Process the query through the RAG service
        result = await rag_service.process_query(request.question, query_id)

        return QueryResponse(
            answer=result.answer,
            sources=result.sources,
            query_id=query_id,
            response_timestamp=datetime.utcnow().isoformat() + "Z",
            confidence_score=result.confidence_score
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    """Health check endpoint."""
    # Add checks for external services if needed
    return {"status": "healthy"}
```

### Step 2: Implement Models (models/request_models.py and models/response_models.py)
Create Pydantic models that match the data model specifications.

### Step 3: Implement Services
- Create retrieval service that connects to Qdrant
- Create generation service that interfaces with Google Gemini
- Create RAG service that orchestrates the flow

### Step 4: Environment Configuration
Use python-dotenv to load environment variables securely.

## Running the Application

### Development
```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

### Production
```bash
cd backend
gunicorn main:app -w 4 -k uvicorn.workers.UvicornWorker --bind 0.0.0.0:8000
```

## Testing the API

### Example Query
```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -H "X-API-Key: your-api-key-here" \
  -d '{"question": "What are the key principles of retrieval-augmented generation?"}'
```

### Expected Response
```json
{
  "answer": "Retrieval-Augmented Generation (RAG) combines information retrieval...",
  "sources": [
    {
      "source_url": "https://example.com/textbook/chapter1.html",
      "document_title": "Chapter 1: Introduction to AI",
      "excerpt": "Retrieval-Augmented Generation (RAG) is a technique that...",
      "similarity_score": 0.92
    }
  ],
  "query_id": "123e4567-e89b-12d3-a456-426614174000",
  "response_timestamp": "2025-12-17T10:30:00Z",
  "confidence_score": 0.85
}
```

## Error Handling

The API returns appropriate HTTP status codes:
- 200: Successful query processing
- 400: Invalid input (malformed request)
- 429: Rate limit exceeded
- 500: Internal server error

Common error responses follow the ErrorResponse format defined in the API specification.

## Security Considerations

- Store API keys in environment variables, never in code
- Implement rate limiting for public APIs
- Validate all inputs before processing
- Use HTTPS in production
- Consider implementing proper authentication beyond API keys if needed