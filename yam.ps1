# Launcher for the YAM test toolkit (PowerShell).
# Usage:  .\yam.ps1 checkup   |   .\yam.ps1 arm   |   .\yam.ps1 live --mode jog
# Finds a Python that has the arm_test package installed and runs the CLI.

$ErrorActionPreference = "Stop"

$py = $null

# 1) prefer a python / py already on PATH (usually the one you pip-installed with)
$cmd = Get-Command python -ErrorAction SilentlyContinue
if ($cmd) { $py = $cmd.Source }
if (-not $py) {
    $cmd = Get-Command py -ErrorAction SilentlyContinue
    if ($cmd) { $py = $cmd.Source }
}

# 2) fall back to a local Anaconda / Miniconda install
if (-not $py) {
    $candidates = @(
        "C:\ProgramData\Anaconda3\python.exe",
        "$env:USERPROFILE\anaconda3\python.exe",
        "$env:USERPROFILE\miniconda3\python.exe"
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { $py = $c; break }
    }
}

if (-not $py) {
    Write-Error "No Python found. Install Python 3.10+, or edit yam.ps1 and set `$py to your python.exe."
    exit 1
}

& $py -m arm_test.cli @args
exit $LASTEXITCODE
