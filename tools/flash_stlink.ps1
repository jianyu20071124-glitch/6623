# 用 ST-Link 烧录（走 STM32CubeProgrammer CLI，Under Reset 模式）
# 用法：pwsh -File flash_stlink.ps1
#       pwsh -File flash_stlink.ps1 -elf "D:\path\to\other.elf"

param(
    [string]$elf = "D:\6233rm\build\Debug\6233rm.elf"
)

$cli = "C:\ST\STM32CubeCLT_1.18.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"

if (-not (Test-Path $elf)) { Write-Host "找不到 ELF: $elf" -ForegroundColor Red; exit 1 }
if (-not (Test-Path $cli)) { Write-Host "找不到 CubeProgrammer CLI: $cli" -ForegroundColor Red; exit 1 }

Write-Host "烧录 $elf ..." -ForegroundColor Cyan
& $cli -c port=SWD mode=UR -w $elf -v -rst

if ($LASTEXITCODE -eq 0) { Write-Host "`n烧录完成" -ForegroundColor Green }
else                     { Write-Host "`n烧录失败（检查 ST-Link 接线和板子供电）" -ForegroundColor Red }
