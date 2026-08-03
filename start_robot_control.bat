@echo off
setlocal
cd /d "%~dp0"

where python >nul 2>nul
if not errorlevel 1 (
    python tools\start_robot_control.py %*
    if errorlevel 1 pause
    exit /b %errorlevel%
)

where py >nul 2>nul
if not errorlevel 1 (
    py -3 tools\start_robot_control.py %*
    if errorlevel 1 pause
    exit /b %errorlevel%
)

echo Python 3 was not found. Install Python or open an ESP-IDF terminal first.
pause
exit /b 1
