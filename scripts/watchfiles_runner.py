#!/usr/bin/env python3
"""
Robotics MCP WebApp Watchfiles Crashproof Runner

This script provides crashproofing for the Robotics MCP web application using watchfiles.
It automatically detects crashes and restarts the web server with exponential backoff.

Usage:
    python scripts/watchfiles_runner.py

Environment Variables:
    ROBOTICS_MCP_HOST: Host to bind to (default: 0.0.0.0)
    ROBOTICS_MCP_PORT: Port to bind to (default: 12230)
    ROBOTICS_MCP_DEBUG: Enable debug mode (default: False)
    WATCHFILES_MAX_RESTARTS: Max restart attempts (default: 10)
    WATCHFILES_RESTART_DELAY: Base restart delay in seconds (default: 1.0)
    WATCHFILES_BACKOFF_MULTIPLIER: Exponential backoff multiplier (default: 1.5)
    WATCHFILES_HEALTH_CHECK_INTERVAL: Health check interval in seconds (default: 30)
    WATCHFILES_NOTIFY_ON_CRASH: Enable crash notifications (default: True)

Author: FlowEngineer
Created: 2026-01-17
"""

import asyncio
import json
import logging
import os
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List

import aiohttp

# Configure logging
# Ensure logs directory exists
Path("logs").mkdir(exist_ok=True)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler("logs/robotics-mcp-watchfiles.log", mode="a"),
    ],
)
logger = logging.getLogger("robotics-mcp-watchfiles")


class CrashproofRunner:
    """Crashproof runner for Robotics MCP web application using watchfiles monitoring."""

    def __init__(
        self,
        command: List[str],
        cwd: str = None,
        max_restarts: int = 10,
        restart_delay: float = 1.0,
        backoff_multiplier: float = 1.5,
        health_check_url: str = None,
        health_check_interval: int = 30,
        notify_on_crash: bool = True,
    ):
        self.command = command
        self.cwd = cwd or os.getcwd()
        self.max_restarts = max_restarts
        self.restart_delay = restart_delay
        self.backoff_multiplier = backoff_multiplier
        self.health_check_url = health_check_url
        self.health_check_interval = health_check_interval
        self.notify_on_crash = notify_on_crash

        self.process = None
        self.restart_count = 0
        self.start_time = datetime.now()
        self.last_health_check = 0
        self.uptime_total = 0
        self.crash_log = []

    async def start_process(self) -> bool:
        """Start the Robotics MCP web application process."""
        try:
            # Set environment variables for the webapp
            env = os.environ.copy()
            env.update(
                {
                    "PYTHONPATH": str(Path(self.cwd) / "src"),
                    "ROBOTICS_MCP_DEBUG": "false",
                    "ROBOTICS_MCP_LOG_LEVEL": "INFO",
                }
            )

            self.process = await asyncio.create_subprocess_exec(
                *self.command,
                cwd=self.cwd,
                env=env,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
                preexec_fn=None if os.name == "nt" else os.setsid,
            )

            logger.info(
                f"✓ Started Robotics MCP WebApp process: {' '.join(self.command)} (PID: {self.process.pid})"
            )
            return True

        except Exception as e:
            logger.error(f"✗ Failed to start Robotics MCP WebApp process: {e}")
            return False

    async def check_health(self) -> bool:
        """Check Robotics MCP WebApp health via HTTP endpoint."""
        if not self.health_check_url:
            return True  # Assume healthy if no health check configured

        try:
            timeout = aiohttp.ClientTimeout(total=5)
            async with aiohttp.ClientSession(timeout=timeout) as session:
                async with session.get(self.health_check_url) as resp:
                    if resp.status == 200:
                        # Check if this is a Robotics MCP health endpoint
                        try:
                            data = await resp.json()
                            return data.get("status") == "healthy" or "Robotics" in str(data)
                        except:
                            # If we can't parse JSON, just check status
                            return resp.status == 200
                    return False
        except Exception as e:
            logger.debug(f"Health check failed: {e}")
            return False

    async def monitor_process(self):
        """Monitor process health and restart on crash."""
        while True:
            current_time = time.time()

            try:
                # Periodic health check
                if current_time - self.last_health_check > self.health_check_interval:
                    if not await self.check_health():
                        logger.warning("WARNING: Health check failed - initiating restart")
                        await self.restart_process()
                    self.last_health_check = current_time

                # Check if process is still running
                if self.process:
                    if self.process.returncode is not None:
                        # Process has exited
                        crash_time = datetime.now()
                        self.uptime_total += (crash_time - self.start_time).total_seconds()

                        # Log crash details
                        crash_info = {
                            "timestamp": crash_time.isoformat(),
                            "exit_code": self.process.returncode,
                            "uptime_before_crash": self.uptime_total,
                            "restart_attempt": self.restart_count + 1,
                            "pid": self.process.pid,
                        }
                        self.crash_log.append(crash_info)

                        # Read stderr for error details
                        if self.process.stderr:
                            try:
                                stderr_data = await self.process.stderr.read()
                                if stderr_data:
                                    crash_info["stderr"] = stderr_data.decode(
                                        "utf-8", errors="ignore"
                                    ).strip()
                            except:
                                pass

                        logger.warning(
                            f"CRASH: Robotics MCP WebApp crashed with exit code {self.process.returncode}"
                        )
                        logger.info(f"UPTIME: Uptime before crash: {self.uptime_total:.1f}s")
                        logger.info(
                            f"RESTART: Restart attempt {self.restart_count + 1}/{self.max_restarts}"
                        )

                        if self.notify_on_crash:
                            await self.notify_crash(crash_info)

                        await self.restart_process()
                    else:
                        # Process is healthy, update uptime
                        self.uptime_total = (datetime.now() - self.start_time).total_seconds()
                else:
                    # No process running, start one
                    await self.start_process()

            except Exception as e:
                logger.error(f"Error in process monitoring: {e}")

            await asyncio.sleep(1)

    async def restart_process(self):
        """Restart crashed process with exponential backoff."""
        if self.restart_count >= self.max_restarts:
            logger.error(f"✗ Maximum restarts ({self.max_restarts}) exceeded. Giving up.")
            logger.error("💡 Consider checking application logs and fixing underlying issues")
            await self.save_crash_report()
            return

        self.restart_count += 1
        delay = self.restart_delay * (self.backoff_multiplier ** (self.restart_count - 1))

        logger.info(
            f"🔄 Restarting Robotics MCP WebApp in {delay:.1f}s... (attempt {self.restart_count}/{self.max_restarts})"
        )

        # Clean up old process
        if self.process:
            try:
                if os.name == "nt":
                    self.process.terminate()
                else:
                    os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                await asyncio.wait_for(self.process.wait(), timeout=5.0)
                logger.debug(f"Cleaned up old process (PID: {self.process.pid})")
            except Exception as e:
                logger.warning(f"Error cleaning up old process: {e}")
                try:
                    self.process.kill()
                except:
                    pass

        await asyncio.sleep(delay)

        # Reset start time for new process
        self.start_time = datetime.now()

        # Start new process
        if not await self.start_process():
            logger.error(
                "Failed to restart Robotics MCP WebApp - will retry on next monitoring cycle"
            )

    async def notify_crash(self, crash_info: Dict[str, Any]):
        """Send crash notification (extend this for email/SMS/etc)."""
        try:
            # For now, just log the crash details
            logger.error("🚨 ROBOTICS MCP WEBAPP CRASH DETECTED")
            logger.error(f"   Exit Code: {crash_info['exit_code']}")
            logger.error(f"   Uptime: {crash_info['uptime_before_crash']:.1f}s")
            logger.error(f"   PID: {crash_info['pid']}")
            logger.error(f"   Timestamp: {crash_info['timestamp']}")

            if crash_info.get("stderr"):
                # Filter out common uvicorn startup messages and show only errors
                stderr_lines = crash_info["stderr"].split("\n")
                error_lines = [
                    line
                    for line in stderr_lines
                    if any(
                        keyword in line.upper()
                        for keyword in ["ERROR", "EXCEPTION", "FAILED", "CRASH", "FATAL"]
                    )
                ]
                if error_lines:
                    logger.error(f"   Error Output: {'; '.join(error_lines[:3])}...")

            # TODO: Add email/SMS notifications here
            # await self.send_notification_email(crash_info)
            # await self.send_sms_alert(crash_info)

        except Exception as e:
            logger.error(f"Error sending crash notification: {e}")

    async def save_crash_report(self):
        """Save detailed crash report to file."""
        try:
            report_path = Path("logs") / f"robotics-mcp-crash-report-{int(time.time())}.json"

            report = {
                "generated_at": datetime.now().isoformat(),
                "total_crashes": len(self.crash_log),
                "total_restarts": self.restart_count,
                "total_uptime": self.uptime_total,
                "max_restarts_allowed": self.max_restarts,
                "crash_events": self.crash_log[-10:],  # Last 10 crashes
                "system_info": {
                    "platform": sys.platform,
                    "python_version": sys.version,
                    "cwd": self.cwd,
                    "command": self.command,
                },
                "app_info": {
                    "name": "Robotics MCP WebApp",
                    "default_host": "0.0.0.0",
                    "default_port": 12230,
                    "health_endpoint": self.health_check_url or "Not configured",
                },
            }

            report_path.parent.mkdir(exist_ok=True)
            with open(report_path, "w") as f:
                json.dump(report, f, indent=2, default=str)

            logger.info(f"💾 Crash report saved to: {report_path}")

        except Exception as e:
            logger.error(f"Error saving crash report: {e}")

    def get_stats(self) -> Dict[str, Any]:
        """Get current runner statistics."""
        return {
            "process_running": self.process is not None and self.process.returncode is None,
            "process_pid": self.process.pid if self.process else None,
            "restart_count": self.restart_count,
            "max_restarts": self.max_restarts,
            "total_uptime": self.uptime_total,
            "crash_count": len(self.crash_log),
            "start_time": self.start_time.isoformat(),
            "command": self.command,
            "health_check_url": self.health_check_url,
        }

    async def run(self):
        """Main runner loop."""
        logger.info("START: Starting Robotics MCP WebApp Crashproof Runner")
        logger.info("==================================================")
        logger.info(f"DIR: Working directory: {self.cwd}")
        logger.info(f"CMD: Command: {' '.join(self.command)}")
        logger.info(f"MAX_RESTARTS: Max restarts: {self.max_restarts}")
        logger.info(f"HEALTH: Health check: {self.health_check_url or 'Disabled'}")
        logger.info("==================================================")

        # Set up signal handlers for graceful shutdown
        def signal_handler(signum, frame):
            logger.info(f"🛑 Received signal {signum} - shutting down...")
            if self.process:
                try:
                    if os.name == "nt":
                        self.process.terminate()
                    else:
                        os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
                except Exception as e:
                    logger.error(f"Error terminating process: {e}")

            # Save final crash report
            asyncio.create_task(self.save_crash_report())
            sys.exit(0)

        # Register signal handlers
        signal.signal(signal.SIGINT, signal_handler)
        signal.signal(signal.SIGTERM, signal_handler)

        try:
            # Start initial process
            if not await self.start_process():
                logger.error("Failed to start initial Robotics MCP WebApp process")
                return

            # Monitor and restart loop
            await self.monitor_process()

        except KeyboardInterrupt:
            logger.info("🛑 Shutting down gracefully...")
        except Exception as e:
            logger.error(f"Fatal error in runner: {e}")
        finally:
            # Save final crash report
            await self.save_crash_report()
            logger.info("👋 Robotics MCP WebApp Crashproof Runner stopped")


def load_config() -> Dict[str, Any]:
    """Load configuration from environment variables."""
    return {
        "host": os.getenv("ROBOTICS_MCP_HOST", "0.0.0.0"),
        "port": int(os.getenv("ROBOTICS_MCP_PORT", "12230")),
        "debug": os.getenv("ROBOTICS_MCP_DEBUG", "false").lower() == "true",
        "max_restarts": int(os.getenv("WATCHFILES_MAX_RESTARTS", "10")),
        "restart_delay": float(os.getenv("WATCHFILES_RESTART_DELAY", "1.0")),
        "backoff_multiplier": float(os.getenv("WATCHFILES_BACKOFF_MULTIPLIER", "1.5")),
        "health_check_interval": int(os.getenv("WATCHFILES_HEALTH_CHECK_INTERVAL", "30")),
        "notify_on_crash": os.getenv("WATCHFILES_NOTIFY_ON_CRASH", "true").lower() == "true",
    }


async def main():
    """Main entry point."""
    config = load_config()

    # Build python module command for Robotics MCP WebApp
    command = [
        sys.executable,
        "-m",
        "robotics_mcp.server",
        "--mode",
        "http",
        "--host",
        config["host"],
        "--port",
        str(config["port"]),
    ]

    # Health check URL
    health_url = f"http://{config['host']}:{config['port']}/api/v1/health"

    # Create and run crashproof runner
    runner = CrashproofRunner(
        command=command,
        cwd=Path(__file__).parent.parent,  # Go up to project root
        max_restarts=config["max_restarts"],
        restart_delay=config["restart_delay"],
        backoff_multiplier=config["backoff_multiplier"],
        health_check_url=health_url,
        health_check_interval=config["health_check_interval"],
        notify_on_crash=config["notify_on_crash"],
    )

    await runner.run()


if __name__ == "__main__":
    # Ensure we're in the right directory
    script_dir = Path(__file__).parent
    os.chdir(script_dir.parent)  # Go to project root

    # Run the crashproof runner
    asyncio.run(main())