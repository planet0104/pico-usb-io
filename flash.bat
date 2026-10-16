@echo off
chcp 65001 >nul
setlocal
cd /d %~dp0
set PORT=%1
if /I "%PORT%"=="build" (
  set PORT=
  set BUILD_ONLY=1
) else (
  set BUILD_ONLY=
  if "%PORT%"=="" set PORT=COM5
)

echo 编译固件
cargo build -p firmware --release --target thumbv6m-none-eabi
if errorlevel 1 exit /b 1

where elf2uf2-rs >nul 2>&1
if errorlevel 1 (
  echo 安装 elf2uf2-rs
  cargo install elf2uf2-rs
  if errorlevel 1 exit /b 1
)

echo 生成 firmware.uf2
elf2uf2-rs target\thumbv6m-none-eabi\release\firmware firmware.uf2
if errorlevel 1 exit /b 1

if defined BUILD_ONLY (
  echo 已生成 %~dp0firmware.uf2
  exit /b 0
)

powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0flash-copy.ps1" -Port "%PORT%" -Uf2 "%~dp0firmware.uf2"
exit /b %errorlevel%
