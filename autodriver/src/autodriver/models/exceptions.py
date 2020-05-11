class ConnectionError(RuntimeError):
    """Raised on failure to connect to Arduino."""
    pass


class ImageProcessingError(RuntimeError):
    """Raised when lane detection fails."""
    pass
