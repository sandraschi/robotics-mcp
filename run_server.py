import os
import sys

# Ensure src directory is in sys.path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "src")))

from robotics_mcp.server import main

if __name__ == "__main__":
    main()
