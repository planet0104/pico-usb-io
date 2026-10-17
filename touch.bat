@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set PIN=%2
if "%PIN%"=="" set PIN=14
set POL=%3
cargo run -p host --bin touch -- %PORT% %PIN% %POL%
exit /b %errorlevel%
