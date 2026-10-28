@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set SECS=%2
if "%SECS%"=="" set SECS=120
cargo run -p host --bin gps -- %PORT% %SECS% %3
exit /b %errorlevel%
