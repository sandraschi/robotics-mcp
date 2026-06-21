!macro KillProcesses
  DetailPrint "Stopping robotics-mcp processes..."
  ExecWait 'taskkill /F /IM "robotics-mcp-backend.exe" /T' $0
  ExecWait 'taskkill /F /IM "robotics-mcp-native.exe" /T' $0
  !if "${INSTALLMODE}" == "currentUser"
    nsis_tauri_utils::KillProcessCurrentUser "robotics-mcp-backend.exe"
    Pop $0
    nsis_tauri_utils::KillProcessCurrentUser "robotics-mcp-native.exe"
    Pop $0
  !else
    nsis_tauri_utils::KillProcess "robotics-mcp-backend.exe"
    Pop $0
    nsis_tauri_utils::KillProcess "robotics-mcp-native.exe"
    Pop $0
  !endif
  Sleep 2000
!macroend

!macro NSIS_HOOK_PREINSTALL
  !insertmacro KillProcesses
!macroend

!macro NSIS_HOOK_PREUNINSTALL
  !insertmacro KillProcesses
!macroend
