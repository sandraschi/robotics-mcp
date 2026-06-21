# Robotics MCP Watchfiles Crashproofing

This document explains how to use watchfiles protection for crashproof operation of the Robotics MCP server.

## Overview

The Robotics MCP server now includes automatic crash recovery using the `watchfiles` library. This provides:

- **Automatic crash detection** and restart
- **Exponential backoff** to prevent restart loops
- **Health monitoring** with HTTP endpoint checks
- **Comprehensive logging** and crash reports
- **Zero-touch operation** for production deployments

## Quick Start

### 1. Install Dependencies

```powershell
# Run the installation script
.\scripts\install-watchfiles.ps1
```

Or manually:
```powershell
pip install watchfiles>=0.24.0 aiohttp>=3.9.0
```

### 2. Run with Protection

```powershell
# Basic usage (recommended)
.\scripts\run-with-watchfiles.ps1

# With custom settings
.\scripts\run-with-watchfiles.ps1 -Port 12231 -Debug -MaxRestarts 5

# Disable notifications
.\scripts\run-with-watchfiles.ps1 -NoNotifications
```

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `ROBOTICS_MCP_HOST` | `0.0.0.0` | Server bind host |
| `ROBOTICS_MCP_PORT` | `12230` | Server port |
| `ROBOTICS_MCP_DEBUG` | `false` | Enable debug mode |
| `WATCHFILES_MAX_RESTARTS` | `10` | Max restart attempts |
| `WATCHFILES_RESTART_DELAY` | `1.0` | Initial delay (seconds) |
| `WATCHFILES_BACKOFF_MULTIPLIER` | `1.5` | Backoff multiplier |
| `WATCHFILES_HEALTH_CHECK_INTERVAL` | `30` | Health check interval |
| `WATCHFILES_NOTIFY_ON_CRASH` | `true` | Enable crash notifications |

### PowerShell Parameters

```powershell
.\scripts\run-with-watchfiles.ps1 `
    -Host "0.0.0.0" `
    -Port 12230 `
    -Debug `
    -MaxRestarts 10 `
    -RestartDelay 1.0 `
    -BackoffMultiplier 1.5 `
    -HealthCheckInterval 30 `
    -NoNotifications
```

## How It Works

1. **Process Monitoring**: The runner monitors the MCP server process for crashes
2. **Health Checks**: Periodically pings `/api/v1/health` endpoint
3. **Automatic Restart**: Restarts crashed processes with exponential backoff
4. **Crash Reports**: Saves detailed crash logs to `logs/` directory
5. **Graceful Shutdown**: Handles Ctrl+C and system signals properly

## Crash Reports

When crashes occur, detailed reports are saved to:
```
logs/robotics-mcp-crash-report-[timestamp].json
```

Report includes:
- Crash timestamp and exit code
- Process uptime before crash
- System information
- Recent stderr output
- Restart attempt count

## Systemd Service (Linux)

For production deployment on Linux:

```bash
# Copy service file
sudo cp robotics-mcp-watchfiles.service /etc/systemd/system/

# Edit paths in service file
sudo nano /etc/systemd/system/robotics-mcp-watchfiles.service

# Reload systemd
sudo systemctl daemon-reload

# Enable and start
sudo systemctl enable robotics-mcp-watchfiles
sudo systemctl start robotics-mcp-watchfiles

# Check status
sudo systemctl status robotics-mcp-watchfiles
```

## Troubleshooting

### Process Won't Start
- Check Python path and dependencies
- Verify port 12230 is not in use
- Check logs in `logs/robotics-mcp-watchfiles.log`

### Health Checks Failing
- Ensure MCP server is exposing `/api/v1/health` endpoint
- Check network connectivity
- Verify CORS settings allow health check requests

### Excessive Restarts
- Check application logs for underlying issues
- Increase `WATCHFILES_HEALTH_CHECK_INTERVAL`
- Review crash reports for patterns

## Logs

### Application Logs
- `logs/robotics-mcp-watchfiles.log` - Runner activity
- `logs/robotics-mcp-crash-report-*.json` - Crash details

### System Logs (Linux)
```bash
# View service logs
sudo journalctl -u robotics-mcp-watchfiles -f

# View recent logs
sudo journalctl -u robotics-mcp-watchfiles --since "1 hour ago"
```

## Benefits

### Before Watchfiles
- ❌ Manual restart required after crashes
- ❌ Service downtime during off-hours
- ❌ No visibility into crash causes
- ❌ Potential data loss on crashes

### After Watchfiles
- ✅ **Zero-touch recovery** - automatic restart
- ✅ **24/7 uptime** - survives crashes gracefully
- ✅ **Crash analytics** - detailed reports and logs
- ✅ **Production ready** - enterprise-grade stability

## Best Practices

1. **Monitor Logs**: Regularly check crash reports for patterns
2. **Set Appropriate Limits**: Configure restart limits based on your environment
3. **Health Endpoints**: Ensure your app has reliable health checks
4. **Resource Limits**: Set appropriate memory/CPU limits for the runner
5. **Backup Strategies**: Combine with data backup for complete protection

## Security Considerations

- The runner runs with the same permissions as your application
- Crash reports may contain sensitive information - store securely
- Consider log rotation for long-running deployments
- Use firewall rules to restrict access to health endpoints

---

**Last Updated**: 2026-01-17
**Compatibility**: Windows 10+, Linux (systemd), macOS