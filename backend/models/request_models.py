from pydantic import BaseModel, Field
from typing import Optional

class QueryRequest(BaseModel):
    """
    Model for the query request payload
    """
    question: str = Field(
        ...,
        min_length=1,
        max_length=1000,
        description="The user's question to be answered"
    )
    selected_text: str = Field(
        default="",
        description="Selected text context for the query"
    )

class QueryWithIdRequest(QueryRequest):
    """
    Model for the query request with additional metadata
    """
    query_id: Optional[str] = Field(
        None,
        description="Optional query identifier for tracking"
    )