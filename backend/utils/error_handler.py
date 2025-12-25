from fastapi import Request, HTTPException
from fastapi.responses import JSONResponse
from typing import Callable, Dict, Any
import logging
from datetime import datetime
import traceback

from models.response_models import ErrorResponse, ErrorCodes

logger = logging.getLogger(__name__)

class ErrorHandler:
    """
    Error handling middleware for standardized error responses
    """
    @staticmethod
    def format_error(error_code: ErrorCodes, message: str, details: Dict[str, Any] = None) -> ErrorResponse:
        """
        Format an error response according to the specification
        """
        return ErrorResponse(
            error_code=error_code,
            message=message,
            timestamp=datetime.utcnow().isoformat() + "Z",
            details=details
        )

    @staticmethod
    async def handle_exception(request: Request, exc: Exception) -> JSONResponse:
        """
        Handle exceptions and return standardized error responses
        """
        logger.error(f"Exception occurred: {str(exc)}\nTraceback: {traceback.format_exc()}")

        # Determine error code based on exception type
        if isinstance(exc, HTTPException):
            error_code = ErrorHandler._get_http_error_code(exc.status_code)
            message = exc.detail
        elif "rate limit" in str(exc).lower():
            error_code = ErrorCodes.RATE_LIMIT_EXCEEDED
            message = "Rate limit exceeded. Please try again later."
        elif "vector" in str(exc).lower() or "qdrant" in str(exc).lower():
            error_code = ErrorCodes.VECTOR_DB_ERROR
            message = "Vector database error occurred."
        elif "gemini" in str(exc).lower() or "llm" in str(exc).lower():
            error_code = ErrorCodes.LLM_ERROR
            message = "Language model service error occurred."
        else:
            error_code = ErrorCodes.SERVICE_UNAVAILABLE
            message = "An unexpected error occurred."

        error_response = ErrorHandler.format_error(
            error_code=error_code,
            message=message,
            details={"original_error": str(exc), "path": str(request.url)}
        )

        status_code = ErrorHandler._get_status_code_from_error_code(error_code)
        return JSONResponse(
            status_code=status_code,
            content=error_response.model_dump()
        )

    @staticmethod
    def _get_http_error_code(status_code: int) -> ErrorCodes:
        """
        Map HTTP status codes to our error codes
        """
        mapping = {
            400: ErrorCodes.INVALID_INPUT,
            404: ErrorCodes.INVALID_INPUT,
            429: ErrorCodes.RATE_LIMIT_EXCEEDED,
            500: ErrorCodes.SERVICE_UNAVAILABLE,
            502: ErrorCodes.SERVICE_UNAVAILABLE,
            503: ErrorCodes.SERVICE_UNAVAILABLE,
            504: ErrorCodes.SERVICE_UNAVAILABLE,
        }
        return mapping.get(status_code, ErrorCodes.SERVICE_UNAVAILABLE)

    @staticmethod
    def _get_status_code_from_error_code(error_code: ErrorCodes) -> int:
        """
        Map our error codes to HTTP status codes
        """
        mapping = {
            ErrorCodes.INVALID_INPUT: 400,
            ErrorCodes.QUERY_TOO_SHORT: 400,
            ErrorCodes.NO_RELEVANT_CONTENT: 404,
            ErrorCodes.RATE_LIMIT_EXCEEDED: 429,
            ErrorCodes.SERVICE_UNAVAILABLE: 503,
            ErrorCodes.LLM_ERROR: 502,
            ErrorCodes.VECTOR_DB_ERROR: 502,
        }
        return mapping.get(error_code, 500)

# Global error handler instance
error_handler = ErrorHandler()

def add_exception_handler(app):
    """
    Add exception handler to FastAPI app
    """
    @app.exception_handler(Exception)
    async def global_exception_handler(request: Request, exc: Exception):
        return await error_handler.handle_exception(request, exc)

    @app.exception_handler(HTTPException)
    async def http_exception_handler(request: Request, exc: HTTPException):
        return await error_handler.handle_exception(request, exc)