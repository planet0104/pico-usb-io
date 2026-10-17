@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set BASE=%2
if "%BASE%"=="" set BASE=2
set RATE=%3
if "%RATE%"=="" set RATE=400
cargo run -p host --bin stepper -- %PORT% %BASE% %RATE%
exit /b %errorlevel%
