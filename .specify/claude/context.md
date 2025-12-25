# Agent Context for Claude Code

## Technologies Used in Current Project

### Backend Framework
- **FastAPI**: Modern, fast web framework for building APIs with Python 3.7+ based on standard Python type hints

### Vector Database
- **Qdrant**: Vector database for the next generation of AI applications, providing high-performance similarity search

### LLM Integration
- **Google Gemini**: Google's multimodal AI model accessible via API for text generation and understanding tasks

### Embedding Model
- **Cohere**: Embedding models used for converting text to vector representations for similarity search

### Orchestration
- **OpenAI Agents SDK**: Framework for creating intelligent agents that can use tools and execute tasks (Note: May need to be adapted for Gemini integration)

### API Documentation
- **OpenAPI 3.0**: Standard for defining RESTful APIs, used for documenting the RAG agent endpoints

### Development Tools
- **Pydantic**: Data validation and settings management using Python type hints
- **python-dotenv**: Reads key-value pairs from a .env file and adds them to environment variables
- **uvicorn**: Lightning-fast ASGI server implementation, using uvloop and httptools

## Project Architecture Patterns

### RAG (Retrieval-Augmented Generation)
- Pattern combining information retrieval with language model generation
- Ensures responses are grounded in retrieved content
- Includes source attribution in responses

### Microservice Architecture
- Separation of concerns with dedicated services for retrieval, generation, and orchestration
- Clean API boundaries between components
- Asynchronous processing where appropriate

### API-First Design
- OpenAPI specification drives API development
- Contract-first approach for frontend/backend integration
- Standardized error handling and response formats

## Data Flow Patterns

### Query Processing Pipeline
1. User query received via API
2. Query validated and transformed to embedding
3. Vector similarity search in Qdrant
4. Relevant content retrieved with source metadata
5. Context passed to LLM for answer generation
6. Response formatted with answer and sources
7. Response returned to client

## Security Considerations

### API Key Management
- API keys stored in environment variables
- Not hardcoded in source code
- Secure transmission over HTTPS

### Input Validation
- All user inputs validated using Pydantic models
- Character limits enforced
- Sanitization applied where appropriate

## Configuration Management

### Environment Variables
- Centralized configuration through .env files
- Different configurations for development/production
- Secure handling of sensitive information