# 用 DAP-Link（CMSIS-DAP）烧录，走 OpenOCD
# 用法：powershell -NoProfile -ExecutionPolicy Bypass -File flash_dap.ps1
#       powershell ... -File flash_dap.ps1 -elf "D:/path/x.elf" -speed 100

param(
    [string]$elf   = "D:/6233rm/build/Debug/6233rm.elf",
    [int]   $speed = 1000
)

$ErrorActionPreference = 'Continue'
$ocd = "D:\tools\openocd\xpack-openocd-0.12.0-6\bin\openocd.exe"

if (-not (Test-Path $elf)) { Write-Host "找不到 ELF: $elf" -ForegroundColor Red; exit 1 }
if (-not (Test-Path $ocd)) { Write-Host "找不到 OpenOCD: $ocd" -ForegroundColor Red; exit 1 }

Write-Host "DAP-Link 烧录: $elf  (SWD ${speed}kHz)" -ForegroundColor Cyan

# NRST 未接，改用 SYSRESETREQ 软件复位，
# 否则会报 "timed out while waiting for target halted"
$a = @(
    "-f", "interface/cmsis-dap.cfg",
    "-f", "target/stm32f1x.cfg",
    "-c", "adapter speed $speed",
    "-c", "reset_config none",
    "-c", "cortex_m reset_config sysresetreq",
    "-c", "program $elf verify reset exit"
)

$out = & $ocd @a 2>&1
$code = $LASTEXITCODE
$out | Where-Object { $_ -match 'Info|Error|Warn|Programming|Verified|Resetting|halted' } |
     Select-Object -First 25 | ForEach-Object { Write-Host $_ }

if ($code -eq 0 -and ($out -join "`n") -match 'Verified OK') {
    Write-Host "`n烧录成功" -ForegroundColor Green
} else {
    Write-Host "`n烧录失败" -ForegroundColor Red
    Write-Host "检查接线：GND / SWDIO(PA13) / SWCLK(PA14)"
    Write-Host "若探针报 CMD_INFO failed：拔插 DAP-Link 的 USB 线"
}
