@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set PIN=%2
if "%PIN%"=="" set PIN=26
cargo run -p host --bin battery -- li %PORT% %PIN%
exit /b %errorlevel%
