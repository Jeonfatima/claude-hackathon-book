from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime
from enum import Enum

class ErrorCodes(str, Enum):
    """
    Enum for standardized error codes
    """
    QUERY_TOO_SHORT = "QUERY_TOO_SHORT"
    NO_RELEVANT_CONTENT = "NO_RELEVANT_CONTENT"
    LLM_ERROR = "LLM_ERROR"
    VECTOR_DB_ERROR = "VECTOR_DB_ERROR"
    SERVICE_UNAVAILABLE = "SERVICE_UNAVAILABLE"
    INVALID_INPUT = "INVALID_INPUT"
    RATE_LIMIT_EXCEEDED = "RATE_LIMIT_EXCEEDED"

class Source(BaseModel):
    """
    Model for source attribution in responses
    """
    source_url: str = Field(
        ...,
        description="URL of the source document"
    )
    document_title: str = Field(
        ...,
        description="Title of the source document"
    )
    excerpt: str = Field(
        ...,
        description="Relevant excerpt from the source"
    )
    similarity_score: float = Field(
        ...,
        ge=0,
        le=1,
        description="Relevance score of this source (0-1)"
    )

class QueryResponse(BaseModel):
    """
    Model for the query response payload
    """
    answer: str = Field(
        ...,
        min_length=50,
        max_length=5000,
        description="The generated answer based on retrieved content"
    )
    sources: List[Source] = Field(
        ...,
        min_items=1,
        description="List of sources used to generate the answer"
    )
    query_id: str = Field(
        ...,
        description="Unique identifier for the query"
    )
    response_timestamp: str = Field(
        ...,
        description="Timestamp when the response was generated in ISO format"
    )
    confidence_score: Optional[float] = Field(
        None,
        ge=0,
        le=1,
        description="Confidence level in the answer (0-1)"
    )

class ErrorResponse(BaseModel):
    """
    Model for error responses
    """
    error_code: ErrorCodes = Field(
        ...,
        description="Machine-readable error code"
    )
    message: str = Field(
        ...,
        description="Human-readable error message"
    )
    timestamp: str = Field(
        ...,
        description="Timestamp when the error occurred in ISO format"
    )
    details: Optional[dict] = Field(
        None,
        description="Additional error details"
    )

class HealthResponse(BaseModel):
    """
    Model for health check responses
    """
    status: str = Field(
        ...,
        description="Health status of the service (healthy, degraded, unhealthy)"
    )
    details: Optional[dict] = Field(
        None,
        description="Additional health details"
    )