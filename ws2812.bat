@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set PIN=%2
if "%PIN%"=="" set PIN=16
cargo run -p host --bin ws2812 -- %PORT% %PIN%
exit /b %errorlevel%
