# Per-repo fleet start config for robotics-mcp
# Edit ports/backend target here - start.ps1 is fleet-standard.
@{
    Name         = 'robotics-mcp'
    BackendPort  = 11199
    FrontendPort = 10706
    HealthPath   = '/api/v1/health'
    WebRoot      = 'web_sota'
    Backend = @{
        Kind          = 'uvicorn'
        UvicornTarget = 'web_sota.backend.server:app'
        SyncExtras    = @('dev')
        Env           = @{ WEB_PORT = '11199' }
    }
    Frontend = @{
        Kind           = 'vite-npm'
        PackageManager = 'npm'
        PortEnvVar     = 'VITE_PORT'
        ApiTargetEnv   = 'VITE_API_TARGET'
    }
}
