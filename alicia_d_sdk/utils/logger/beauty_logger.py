import os


# 定义日志级别常量
class LogLevel:
    """日志级别定义"""
    DEBUG = 0
    INFO = 1
    MODULE = 2
    WARNING = 3
    ERROR = 4
    SUCCESS = 5


class BeautyLogger:
    """
    Lightweight logger for Alicia-D-SDK package.
    """

    def __init__(self, log_dir: str, log_name: str = 'rofunc.log', verbose: bool = True, min_level: int = LogLevel.INFO):
        """
        Initialize Alicia-D-SDK lightweight logger.

        :param log_dir: Log file save path
        :param log_name: Log file name
        :param verbose: Whether to print logs to console
        :param min_level: Minimum log level
        """
        self.log_dir = log_dir
        self.log_name = log_name
        self.log_path = os.path.join(self.log_dir, self.log_name)
        self.verbose = verbose
        self.min_level = min_level

        os.makedirs(self.log_dir, exist_ok=True)

    def _write_log(self, content, type):
        with open(self.log_path, "a", encoding="utf-8") as f:
            f.write(" Alicia-D-SDK:{}] {}\n".format(type.upper(), content))

    def _should_print(self, level: int) -> bool:
        """
        Check if log should be printed.

        :param level: Log level to check
        :return: Whether should print
        """
        return self.verbose and level >= self.min_level

    def set_min_level(self, level: int):
        """
        Set minimum log level.

        :param level: Minimum log level
        """
        if level < LogLevel.DEBUG or level > LogLevel.SUCCESS:
            raise ValueError("Invalid log level. Must be between LogLevel.DEBUG and LogLevel.SUCCESS")
        self.min_level = level

    def warning(self, content, local_verbose=True):
        """
        Print warning message.

        :param content: Warning message content
        :param local_verbose: Whether to print to console
        """
        if self._should_print(LogLevel.WARNING) and local_verbose:
            beauty_print(content, type="warning")
        self._write_log(content, type="warning")

    def module(self, content, local_verbose=True):
        """
        Print module message.

        :param content: Module message content
        :param local_verbose: Whether to print to console
        """
        if self._should_print(LogLevel.MODULE) and local_verbose:
            beauty_print(content, type="module")
        self._write_log(content, type="module")

    def info(self, content, local_verbose=True):
        """
        Print info message.

        :param content: Info message content
        :param local_verbose: Whether to print to console
        """
        if self._should_print(LogLevel.INFO) and local_verbose:
            beauty_print(content, type="info")
        self._write_log(content, type="info")

    def debug(self, content, local_verbose=True):
        """
        Print debug message.

        :param content: Debug message content
        :param local_verbose: Whether to print to console
        """
        if self._should_print(LogLevel.DEBUG) and local_verbose:
            beauty_print(content, type="debug")
        self._write_log(content, type="debug")

    def error(self, content, local_verbose=True, raise_exception=True):
        """
        Print error message.

        :param content: Error message content
        :param local_verbose: Whether to print to console
        :param raise_exception: Whether to raise exception
        """
        if self._should_print(LogLevel.ERROR) and local_verbose:
            beauty_print(content, type="error")
        self._write_log(content, type="error")
        if raise_exception:
            raise Exception(content)

    def success(self, content, local_verbose=True):
        """
        Print success message.

        :param content: Success message content
        :param local_verbose: Whether to print to console
        """
        if self._should_print(LogLevel.SUCCESS) and local_verbose:
            beauty_print(content, type="success")
        self._write_log(content, type="success")


def beauty_print(content, type: str = None):
    """
    Print content with different colors.

    :param content: Content to print
    :param type: Supported types: "warning", "module", "info", "error", "debug", "success"
    """
    if type is None:
        type = "info"
    if type == "warning":
        print("\033[1;37m [Alicia-D-SDK:WARNING] {}\033[0m".format(content))  # For warning (gray)
    elif type == "module":
        print("\033[1;33m [Alicia-D-SDK:MODULE] {}\033[0m".format(content))  # For a new module (light yellow)
    elif type == "info":
        print("\033[1;35m [Alicia-D-SDK:INFO] {}\033[0m".format(content))  # For info (light purple)
    elif type == "debug":
        print("\033[1;34m [Alicia-D-SDK:DEBUG] {}\033[0m".format(content))  # For debug (light blue)
    elif type == "error":
        print("\033[1;31m [Alicia-D-SDK:ERROR] {}\033[0m".format(content))  # For error (red)
    elif type == "success":
        print("\033[1;32m [Alicia-D-SDK:SUCCESS] {}\033[0m".format(content))  # For success (green)
    else:
        raise ValueError("Invalid level")
