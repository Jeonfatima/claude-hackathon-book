from pydantic_settings import BaseSettings
from typing import Optional
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

class Settings(BaseSettings):
    """
    Application settings loaded from environment variables
    """
    environment: str = "development"
    log_level: str = "INFO"

    # API Keys
    gemini_api_key: Optional[str] = None
    qdrant_api_key: Optional[str] = None
    qdrant_url: Optional[str] = None
    cohere_api_key: Optional[str] = None

    # Qdrant Configuration
    qdrant_host: str = "localhost"
    qdrant_port: int = 6333
    collection_name: str = "rag_embeddings"

    # Application Configuration
    host: str = "0.0.0.0"
    port: int = 8000
    debug: bool = True

    class Config:
        env_file = ".env"
        case_sensitive = True

def get_settings() -> Settings:
    """
    Get application settings
    """
    return Settings()

# For backward compatibility with existing code
def load_environment_variables():
    """
    Load environment variables from .env file
    """
    load_dotenv()