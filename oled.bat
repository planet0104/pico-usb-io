@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set SDA=%2
if "%SDA%"=="" set SDA=8
set SCL=%3
if "%SCL%"=="" set SCL=9
set H=%4
if "%H%"=="" set H=64
set SECS=%5
if "%SECS%"=="" set SECS=12
set ROT=%6
if "%ROT%"=="" set ROT=0
cargo run -p host --bin oled -- %PORT% %SDA% %SCL% %H% %SECS% %ROT%
exit /b %errorlevel%
