# Per-repo fleet start config for robotics-mcp
# Edit ports/backend target here - start.ps1 is fleet-standard.
@{
    Name         = 'robotics-mcp'
    BackendPort  = 10707
    FrontendPort = 10706
    HealthPath   = '/api/v1/health'
    WebRoot      = 'D:\Dev\repos\robotics-mcp\web_sota'
    Backend = @{
        Kind          = 'uvicorn'
        UvicornTarget = 'robotics_mcp.server:app'
        SyncExtras    = @('dev')
        Env           = @{ WEB_PORT = '10707' }
    }
    Frontend = @{
        Kind           = 'vite-npm'
        PackageManager = 'npm'
        PortEnvVar     = 'VITE_PORT'
        ApiTargetEnv   = 'VITE_API_TARGET'
    }
}
