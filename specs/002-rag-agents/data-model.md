# Data Model for RAG Agent Backend

## Entity: Query
**Description**: Represents a user's request for information from the book content

**Fields**:
- `question`: string (required) - The user's question or request for information
- `timestamp`: datetime (required) - When the query was submitted (ISO 8601 format)
- `query_id`: string (required) - Unique identifier for the query (UUID format)
- `user_id`: string (optional) - Identifier for the user making the request (if applicable)

**Validation Rules**:
- `question` must be non-empty and between 1-1000 characters
- `timestamp` must be a valid ISO 8601 datetime string
- `query_id` must be a valid UUID v4 format
- `question` must contain only printable ASCII characters

## Entity: RetrievedContext
**Description**: Contains the relevant content chunks retrieved from the vector database

**Fields**:
- `content_chunks`: array of strings (required) - Retrieved content fragments relevant to the query
- `source_metadata`: array of objects (required) - Information about the sources of each chunk
- `similarity_scores`: array of numbers (required) - Relevance scores for each content chunk
- `retrieval_timestamp`: datetime (required) - When the retrieval was performed

**Source Metadata Fields**:
- `source_url`: string (required) - URL or identifier for the source document
- `document_title`: string (required) - Title of the source document
- `chunk_index`: number (required) - Position of the chunk in the original document
- `page_number` or `section`: string (optional) - Page number or section identifier

**Validation Rules**:
- `content_chunks` must contain at least 1 element and no more than 10
- `source_metadata` array must have same length as `content_chunks`
- `similarity_scores` array must have same length as `content_chunks`
- Each similarity score must be between 0 and 1
- Each content chunk must be between 50-2000 characters

## Entity: Response
**Description**: Structured output containing the answer and source attribution

**Fields**:
- `answer`: string (required) - The generated answer based on retrieved content
- `sources`: array of objects (required) - Attribution information for the answer
- `query_id`: string (required) - Reference to the original query ID
- `response_timestamp`: datetime (required) - When the response was generated
- `confidence_score`: number (optional) - Confidence level in the answer (0-1)

**Source Object Fields**:
- `source_url`: string (required) - URL of the source document
- `document_title`: string (required) - Title of the source document
- `excerpt`: string (required) - Relevant excerpt from the source
- `similarity_score`: number (required) - Relevance score of this source (0-1)

**Validation Rules**:
- `answer` must be non-empty and between 50-5000 characters
- `sources` array must contain at least 1 element when answer is provided
- `query_id` must match the original query ID format
- `confidence_score` must be between 0 and 1 if provided
- `answer` must be grounded in the content of the provided sources

## Entity: ErrorResponse
**Description**: Standardized format for API errors

**Fields**:
- `error_code`: string (required) - Machine-readable error code
- `message`: string (required) - Human-readable error message
- `details`: object (optional) - Additional error details
- `timestamp`: datetime (required) - When the error occurred

**Validation Rules**:
- `error_code` must be from the predefined error code list
- `message` must be non-empty
- `timestamp` must be a valid ISO 8601 datetime string

## Error Codes
- `QUERY_TOO_SHORT`: User query is too short to process
- `NO_RELEVANT_CONTENT`: No relevant content found in the vector database
- `LLM_ERROR`: Error occurred during LLM processing
- `VECTOR_DB_ERROR`: Error occurred during vector database retrieval
- `SERVICE_UNAVAILABLE`: One of the external services is unavailable
- `INVALID_INPUT`: Provided input does not meet validation requirements
- `RATE_LIMIT_EXCEEDED`: API rate limit has been exceeded

## State Transitions
1. Query received → Validation performed → Valid/Invalid
2. Valid query → Retrieval performed → Context retrieved/Failure
3. Context retrieved → Generation performed → Answer generated/Generation failed
4. Answer generated → Response formatted → Response sent to client