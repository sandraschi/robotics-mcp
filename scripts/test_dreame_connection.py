import asyncio
import logging
import os
import sys

# Add src to path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "src")))

from robotics_mcp.clients.dreame_client import DreameClient
from robotics_mcp.config import DREAME_IP, DREAME_TOKEN

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("test_dreame")


async def main():
    logger.info("Initializing Dreame Client...")
    logger.info(f"Target IP: {DREAME_IP}")
    logger.info(
        f"Token Configured: {'Yes' if DREAME_TOKEN and DREAME_TOKEN != 'YOUR_TOKEN_HERE' else 'No (Placeholder)'}"
    )

    client = DreameClient(DREAME_IP, DREAME_TOKEN)

    logger.info("Connecting...")
    success = await client.connect()

    if success:
        logger.info("✅ Connection Successful!")
        status = await client.get_status()
        logger.info(f"Robot Status: {status}")
    else:
        logger.error("❌ Connection Failed.")
        if DREAME_TOKEN == "YOUR_TOKEN_HERE":
            logger.info("💡 Hint: You need to set the DREAME_TOKEN in src/robotics_mcp/config.py")


if __name__ == "__main__":
    asyncio.run(main())
