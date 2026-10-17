@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set PIN=%2
if "%PIN%"=="" set PIN=15
set SECS=%3
if "%SECS%"=="" set SECS=15
cargo run -p host --bin ir -- %PORT% %PIN% %SECS%
exit /b %errorlevel%
