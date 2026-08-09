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

# PyInstaller会沿PATH搜索被Qt间接引用的DLL。开发机若安装过Anaconda、
# Matlab或其他Qt软件，可能把它们的ICU/Qt DLL误收进发布包，导致目标电脑
# 导入QtWidgets失败。构建阶段只保留当前Python和Windows系统目录。
$originalPath = $env:PATH
$originalPythonPath = $env:PYTHONPATH
$originalPythonHome = $env:PYTHONHOME
$pythonExe = (Resolve-Path -LiteralPath $Python).Path
$pythonScripts = Split-Path -Parent $pythonExe
$basePython = (& $pythonExe -c "import sys; print(sys.base_prefix)").Trim()
if ($LASTEXITCODE -ne 0 -or -not $basePython) {
    throw "无法确定Python基础目录"
}

try {
    $env:PATH = @(
        $pythonScripts,
        $basePython,
        (Join-Path $basePython "DLLs"),
        (Join-Path $env:SystemRoot "System32"),
        $env:SystemRoot
    ) -join ";"
    Remove-Item Env:PYTHONPATH -ErrorAction SilentlyContinue
    Remove-Item Env:PYTHONHOME -ErrorAction SilentlyContinue

    & $pythonExe -m PyInstaller --noconfirm --clean `
        --workpath release_build --distpath release_dist SolarIMU_Tool.spec
    if ($LASTEXITCODE -ne 0) { throw "PyInstaller构建失败" }
}
finally {
    $env:PATH = $originalPath
    if ($null -eq $originalPythonPath) {
        Remove-Item Env:PYTHONPATH -ErrorAction SilentlyContinue
    } else {
        $env:PYTHONPATH = $originalPythonPath
    }
    if ($null -eq $originalPythonHome) {
        Remove-Item Env:PYTHONHOME -ErrorAction SilentlyContinue
    } else {
        $env:PYTHONHOME = $originalPythonHome
    }
}

$foreignIcu = Get-ChildItem -LiteralPath ".\release_dist\SolarIMU_Tool\_internal" `
    -Filter "icu*.dll" -File -ErrorAction SilentlyContinue
if ($foreignIcu) {
    throw "发布包检测到外部ICU DLL污染：$($foreignIcu.Name -join ', ')"
}

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

Write-Host "安装包已生成：release_dist\SolarIMU_Tool_Setup_v1.1.1_Win64.exe"
