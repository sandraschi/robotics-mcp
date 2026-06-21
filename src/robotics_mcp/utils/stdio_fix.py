import os
import sys


def apply_stdio_fix():
    """Apply stdio binary mode fix for Windows to prevent JSON-RPC corruption."""
    if os.name == "nt":  # Windows only
        try:
            # Force binary mode for stdin/stdout to prevent CRLF conversion
            import msvcrt

            msvcrt.setmode(sys.stdin.fileno(), os.O_BINARY)
            msvcrt.setmode(sys.stdout.fileno(), os.O_BINARY)
        except (OSError, AttributeError):
            # Fallback: just ensure no CRLF conversion
            pass


class DevNullStdout:
    """Suppress all stdout writes during stdio mode to prevent JSON-RPC protocol corruption."""

    def __init__(self, original_stdout):
        self.original_stdout = original_stdout
        self.buffer = []

    def write(self, text):
        # Buffer output instead of writing to stdout
        self.buffer.append(text)

    def flush(self):
        # Do nothing - prevent any stdout writes
        pass

    def get_buffered_output(self):
        """Get all buffered output for debugging if needed."""
        return "".join(self.buffer)

    def restore(self):
        """Restore original stdout."""
        sys.stdout = self.original_stdout


def is_stdio_mode():
    """Detect if the server is running in stdio mode."""
    return not sys.stdout.isatty()
