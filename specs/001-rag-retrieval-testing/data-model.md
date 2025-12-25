# Data Model: RAG Retrieval Pipeline Testing

## Entities

### RetrievedChunk
Represents a text chunk returned from the vector database during retrieval testing, containing the original text content, similarity score, and metadata.

**Attributes:**
- `content`: String - The retrieved text content
- `similarity_score`: Float - Semantic similarity score from vector search
- `metadata`: Dictionary - Source information including URL and chunk identifier
- `expected_content`: String - Original content for comparison during testing
- `validation_status`: String - Result of content accuracy validation (pass/fail)

**Validation Rules:**
- Content must not be empty
- Similarity score must be between 0 and 1
- Metadata must contain required fields (URL, chunk_id)

### RetrievalQuery
The input query used to search for relevant content during testing, including parameters like top-k count and search configuration.

**Attributes:**
- `text`: String - The query text to be embedded and searched
- `top_k`: Integer - Number of top results to retrieve (default: 3)
- `expected_matches`: List[ExpectedMatch] - Expected results for validation
- `query_embedding`: List[Float] - The embedding vector for the query

**Validation Rules:**
- Query text must not be empty
- Top-k value must be positive integer
- Query embedding must have correct dimensions (1024 for Cohere embed-english-v3.0)

### ExpectedMatch
Represents the expected result for validation purposes, containing the original text and metadata that should be retrieved.

**Attributes:**
- `content`: String - The expected text content
- `url`: String - Source URL of the expected content
- `chunk_id`: String - Identifier of the expected chunk
- `tolerance`: Float - Acceptable similarity threshold for validation

**Validation Rules:**
- Content must not be empty
- URL must be a valid format
- Chunk_id must be unique identifier

### RetrievalResult
The complete response containing multiple retrieved chunks, metadata, and validation information for testing purposes.

**Attributes:**
- `query`: RetrievalQuery - The original query that generated this result
- `retrieved_chunks`: List[RetrievedChunk] - The chunks returned by the search
- `execution_time`: Float - Time taken to execute the retrieval in seconds
- `validation_results`: ValidationSummary - Overall validation status
- `timestamp`: DateTime - When the retrieval was performed

**Validation Rules:**
- Must contain at least one retrieved chunk if results exist
- Execution time must be recorded
- Validation results must be included

### ValidationSummary
Summary of the validation results for a retrieval operation.

**Attributes:**
- `content_accuracy`: Float - Percentage of chunks with accurate content (0-100)
- `metadata_correctness`: Float - Percentage of chunks with correct metadata (0-100)
- `top_k_relevance`: Float - Relevance score of top-k results (0-100)
- `overall_status`: String - Overall pass/fail status of validation
- `details`: List[ValidationDetail] - Detailed validation results for each chunk

**Validation Rules:**
- Accuracy percentages must be between 0 and 100
- Overall status must be either "pass" or "fail"