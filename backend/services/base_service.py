from abc import ABC, abstractmethod
from typing import Any, Dict, Optional
import logging

class BaseService(ABC):
    """
    Base service class with common functionality for all services
    """
    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)
        self._setup_logging()

    def _setup_logging(self):
        """
        Setup logging for the service
        """
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

    def log_info(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log an info message
        """
        self.logger.info(message, extra=extra)

    def log_error(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log an error message
        """
        self.logger.error(message, extra=extra)

    def log_warning(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log a warning message
        """
        self.logger.warning(message, extra=extra)

    def log_debug(self, message: str, extra: Optional[Dict[str, Any]] = None):
        """
        Log a debug message
        """
        self.logger.debug(message, extra=extra)

    @abstractmethod
    def validate_config(self) -> bool:
        """
        Validate that the service has all required configuration
        """
        pass