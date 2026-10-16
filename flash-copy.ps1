param(
    [Parameter(Mandatory = $true)][string]$Port,
    [Parameter(Mandatory = $true)][string]$Uf2
)

function Find-BootDrive {
    Get-CimInstance Win32_LogicalDisk | Where-Object { $_.VolumeName -eq "RPI-RP2" } | Select-Object -First 1
}

$drive = Find-BootDrive
if (-not $drive) {
    Write-Host "没有找到 RPI-RP2，尝试用 $Port 以 1200 波特进入 BOOTSEL"
    try {
        $serial = New-Object System.IO.Ports.SerialPort $Port, 1200
        $serial.DtrEnable = $false
        $serial.Open()
        Start-Sleep -Milliseconds 400
        $serial.Close()
    } catch {
        Write-Host "打开 $Port 失败。按住 BOOTSEL 再插 USB，然后重跑本脚本。"
    }
    $deadline = (Get-Date).AddSeconds(20)
    while ((Get-Date) -lt $deadline) {
        Start-Sleep -Milliseconds 300
        $drive = Find-BootDrive
        if ($drive) { break }
    }
}

if (-not $drive) {
    Write-Error "仍然没有 RPI-RP2 盘。按住 BOOTSEL 上电后再运行 flash.bat。"
    exit 1
}

$dest = Join-Path ($drive.DeviceID + "\") (Split-Path $Uf2 -Leaf)
Write-Host "复制到 $dest"
Copy-Item -Force $Uf2 $dest
Write-Host "已写入，开发板会自动重启"
