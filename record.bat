@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
cargo run -p host --bin record -- %PORT%
exit /b %errorlevel%
