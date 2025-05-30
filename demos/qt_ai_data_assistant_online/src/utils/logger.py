# custom_logger.py
import logging

# ANSI escape codes for coloring
class LogColors:
    RESET = "\033[0m"
    RED = "\033[31m"
    GREEN = "\033[32m"
    YELLOW = "\033[33m"
    BLUE = "\033[34m"
    MAGENTA = "\033[35m"
    CYAN = "\033[36m"
    WHITE = "\033[37m"
    GRAY = "\033[90m"  # Gray color

# Custom formatter with colors
class ColoredFormatter(logging.Formatter):
    def format(self, record):
        levelname = record.levelname
        if levelname == "DEBUG":
            record.levelname = f"{LogColors.CYAN}[{levelname}]{LogColors.RESET}"
        elif levelname == "INFO":
            record.levelname = f"{LogColors.GREEN}[{levelname}]{LogColors.RESET}"            
        elif levelname == "WARNING":
            record.levelname = f"{LogColors.YELLOW}[{levelname}]{LogColors.RESET}"
        elif levelname == "ERROR":
            record.levelname = f"{LogColors.RED}[{levelname}]{LogColors.RESET}"
        elif levelname == "CRITICAL":
            record.levelname = f"{LogColors.MAGENTA}[{levelname}]{LogColors.RESET}"
        return super().format(record)

class Logger:
    _logger = logging.getLogger('CustomLogger')
    _logger.setLevel(logging.DEBUG)
    _logger.propagate = False  # Prevent messages from being passed to the root logger

    # Ensure no duplicate handlers
    if not _logger.handlers:
        formatter = ColoredFormatter(
            "%(levelname)s [%(asctime)s]: %(message)s",
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)
        _logger.addHandler(handler)

    @classmethod
    def info(cls, message: str):
        cls._logger.info(message)

    @classmethod
    def warning(cls, message: str):
        cls._logger.warning(message)

    @classmethod
    def error(cls, message: str):
        cls._logger.error(message)

    @classmethod
    def debug(cls, message: str):
        cls._logger.debug(message)
