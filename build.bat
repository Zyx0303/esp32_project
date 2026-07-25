@echo off
setlocal

if "%IDF_PATH%"=="" (
    echo ESP-IDF environment is not active.
    echo Run export.bat from your ESP-IDF installation first.
    exit /b 1
)

cd /d "%~dp0"

echo Building...
python "%IDF_PATH%\tools\idf.py" build
if errorlevel 1 exit /b 1

if "%~1"=="" (
    echo Build complete. Pass a port such as COM6 to also flash.
    exit /b 0
)

echo Flashing to %~1...
python "%IDF_PATH%\tools\idf.py" -p "%~1" flash
exit /b %errorlevel%
