from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID

class Query(BaseModel):
    """
    Entity representing a user's request for information from the book content
    """
    question: str = Field(
        ...,
        description="The user's question or request for information"
    )
    timestamp: str = Field(
        ...,
        description="When the query was submitted (ISO 8601 format)"
    )
    query_id: str = Field(
        ...,
        description="Unique identifier for the query (UUID format)"
    )
    user_id: Optional[str] = Field(
        None,
        description="Identifier for the user making the request (if applicable)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "question": "What are the key principles of retrieval-augmented generation?",
                "timestamp": "2025-12-17T10:30:00Z",
                "query_id": "123e4567-e89b-12d3-a456-426614174000",
                "user_id": "user123"
            }
        }

class RetrievedContext(BaseModel):
    """
    Entity containing the relevant content chunks retrieved from the vector database
    """
    content_chunks: List[str] = Field(
        ...,
        min_items=1,
        max_items=10,
        description="Retrieved content fragments relevant to the query"
    )
    source_metadata: List[Dict[str, Any]] = Field(
        ...,
        description="Information about the sources of each chunk"
    )
    similarity_scores: List[float] = Field(
        ...,
        description="Relevance scores for each content chunk (0-1)"
    )
    retrieval_timestamp: str = Field(
        ...,
        description="When the retrieval was performed (ISO 8601 format)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "content_chunks": [
                    "Retrieval-Augmented Generation (RAG) is a technique that...",
                    "The key components of RAG include..."
                ],
                "source_metadata": [
                    {
                        "source_url": "https://example.com/textbook/chapter1.html",
                        "document_title": "Chapter 1: Introduction to AI",
                        "chunk_index": 0,
                        "page_number": "15"
                    }
                ],
                "similarity_scores": [0.92, 0.85],
                "retrieval_timestamp": "2025-12-17T10:30:05Z"
            }
        }

class Response(BaseModel):
    """
    Entity representing structured output containing the answer and source attribution
    """
    answer: str = Field(
        ...,
        min_length=50,
        max_length=5000,
        description="The generated answer based on retrieved content"
    )
    sources: List[Dict[str, Any]] = Field(
        ...,
        min_items=1,
        description="Attribution information for the answer"
    )
    query_id: str = Field(
        ...,
        description="Reference to the original query ID"
    )
    response_timestamp: str = Field(
        ...,
        description="When the response was generated (ISO 8601 format)"
    )
    confidence_score: Optional[float] = Field(
        None,
        ge=0,
        le=1,
        description="Confidence level in the answer (0-1)"
    )

    class Config:
        json_schema_extra = {
            "example": {
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
                "response_timestamp": "2025-12-17T10:30:10Z",
                "confidence_score": 0.85
            }
        }

class ErrorResponseEntity(BaseModel):
    """
    Entity for standardized format for API errors
    """
    error_code: str = Field(
        ...,
        description="Machine-readable error code"
    )
    message: str = Field(
        ...,
        description="Human-readable error message"
    )
    details: Optional[Dict[str, Any]] = Field(
        None,
        description="Additional error details"
    )
    timestamp: str = Field(
        ...,
        description="When the error occurred (ISO 8601 format)"
    )

    class Config:
        json_schema_extra = {
            "example": {
                "error_code": "NO_RELEVANT_CONTENT",
                "message": "No relevant content found in the vector database",
                "details": {
                    "query_length": 3
                },
                "timestamp": "2025-12-17T10:30:15Z"
            }
        }