param(
    [string]$Python = ".\.venv\Scripts\python.exe",
    [string]$Iscc = ""
)

$ErrorActionPreference = "Stop"
$toolRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $toolRoot

if (-not (Test-Path -LiteralPath $Python)) {
    throw "未找到Python环境：$Python"
}

& $Python -m PyInstaller --noconfirm --clean `
    --workpath release_build --distpath release_dist SolarIMU_Tool.spec
if ($LASTEXITCODE -ne 0) { throw "PyInstaller构建失败" }

if (-not $Iscc) {
    $candidates = @(
        "$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe",
        "$env:ProgramFiles(x86)\Inno Setup 6\ISCC.exe",
        "$env:ProgramFiles\Inno Setup 6\ISCC.exe"
    )
    $Iscc = $candidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
}
if (-not $Iscc -or -not (Test-Path -LiteralPath $Iscc)) {
    throw "未找到ISCC.exe，请安装Inno Setup 6或通过-Iscc指定路径"
}

& $Iscc ".\installer\SolarIMU_Tool.iss"
if ($LASTEXITCODE -ne 0) { throw "Inno Setup构建失败" }

Write-Host "安装包已生成：release_dist\SolarIMU_Tool_Setup_v1.1.0_Win64.exe"
