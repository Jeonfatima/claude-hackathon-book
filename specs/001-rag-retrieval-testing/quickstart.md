# Quickstart: RAG Retrieval Pipeline Testing

## Overview
This guide will help you set up and run the RAG retrieval pipeline testing system to verify that stored vectors in Qdrant can be retrieved accurately.

## Prerequisites
- Python 3.11+
- Access to Qdrant vector database with existing `rag_embeddings` collection
- Cohere API key
- Existing embeddings stored from Spec-1 (RAG ingestion)

## Setup

### 1. Environment Configuration
```bash
# Copy the environment template
cp .env.example .env

# Update .env with your credentials
QDRANT_HOST="your-qdrant-host"
QDRANT_API_KEY="your-qdrant-api-key"
COHERE_API_KEY="your-cohere-api-key"
```

### 2. Install Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 3. Verify Environment
```bash
# Test Qdrant connection
python -c "from qdrant_client import QdrantClient; client = QdrantClient(host='your-host', api_key='your-key'); print(client.get_collections())"
```

## Running Tests

### 1. Basic Retrieval Test
```bash
# Run the retrieval tester with default parameters
python -m backend.src.retrieval_tester.main
```

### 2. Custom Test Query
```bash
# Run with a specific query
python -m backend.src.retrieval_tester.main --query "What is ROS 2?" --top-k 3
```

### 3. Batch Test Mode
```bash
# Run comprehensive tests with multiple queries
python -m backend.src.retrieval_tester.main --batch-test
```

## Expected Output
The system will output a JSON result with:
- Retrieved chunks with content and metadata
- Validation status (pass/fail)
- Execution time
- Similarity scores

Example output:
```json
{
  "query": "What is ROS 2?",
  "retrieved_chunks": [
    {
      "content": "ROS 2 is a flexible framework for writing robot software...",
      "similarity_score": 0.85,
      "metadata": {
        "url": "https://example.com/ros2-intro",
        "chunk_id": "chunk-123"
      },
      "validation_status": "pass"
    }
  ],
  "execution_time": 1.25,
  "validation_results": {
    "content_accuracy": 100.0,
    "metadata_correctness": 100.0,
    "overall_status": "pass"
  }
}
```

## Validation Criteria
- Content accuracy: Retrieved text matches original stored content
- Metadata correctness: URL and chunk_id match expected values
- Response time: Under 2 seconds for 95% of requests
- Top-k relevance: Most relevant chunks appear in top results