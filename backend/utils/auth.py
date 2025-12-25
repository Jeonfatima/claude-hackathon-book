from fastapi import Request, HTTPException
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional
import os

class APIKeyValidator:
    """
    Utility class for API key validation
    """
    def __init__(self):
        self.required_api_key = os.getenv("REQUIRED_API_KEY")  # For custom API key validation if needed

    def validate_api_key(self, api_key: str) -> bool:
        """
        Validate the provided API key
        """
        if not api_key:
            return False

        # If we have a required API key in environment, check against it
        if self.required_api_key:
            return api_key == self.required_api_key

        # Otherwise, just check if it's a valid non-empty string
        # In a real implementation, you might check against a database of valid keys
        return len(api_key.strip()) > 0

class APIKeyHeaderAuth(HTTPBearer):
    """
    Custom authentication class that checks for API key in header
    """
    def __init__(self, auto_error: bool = True):
        super().__init__(auto_error=auto_error)
        self.validator = APIKeyValidator()

    async def __call__(self, request: Request) -> Optional[HTTPAuthorizationCredentials]:
        # Check for API key in X-API-Key header (as specified in OpenAPI spec)
        api_key = request.headers.get("X-API-Key")

        if not api_key:
            # If not in X-API-Key header, also check Authorization header
            credentials: HTTPAuthorizationCredentials = await super().__call__(request)
            if credentials:
                api_key = credentials.credentials

        if not api_key or not self.validator.validate_api_key(api_key):
            if self.auto_error:
                raise HTTPException(
                    status_code=401,
                    detail="Invalid API key",
                    headers={"WWW-Authenticate": "Bearer"},
                )
            else:
                return None

        return HTTPAuthorizationCredentials(
            scheme="API-Key",  # Custom scheme for API key
            credentials=api_key
        )

# Global instance
api_key_auth = APIKeyHeaderAuth(auto_error=True)

def get_api_key_from_request(request: Request) -> Optional[str]:
    """
    Extract API key from request headers
    """
    # First check X-API-Key header
    api_key = request.headers.get("X-API-Key")
    if api_key:
        return api_key

    # Then check Authorization header
    auth_header = request.headers.get("Authorization")
    if auth_header and auth_header.startswith("Bearer "):
        return auth_header[7:]  # Remove "Bearer " prefix

    return None