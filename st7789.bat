@echo off
setlocal
cd /d %~dp0
chcp 65001 >nul
set PORT=%1
if "%PORT%"=="" set PORT=COM5
set SCK=%2
if "%SCK%"=="" set SCK=10
set MOSI=%3
if "%MOSI%"=="" set MOSI=11
set DC=%4
if "%DC%"=="" set DC=12
set RST=%5
if "%RST%"=="" set RST=13
set CS=%6
if "%CS%"=="" set CS=none
set ROT=%7
if "%ROT%"=="" set ROT=0
set SECS=%8
if "%SECS%"=="" set SECS=15
cargo run -p host --bin st7789 -- %PORT% %SCK% %MOSI% %DC% %RST% %CS% %ROT% %SECS%
exit /b %errorlevel%
