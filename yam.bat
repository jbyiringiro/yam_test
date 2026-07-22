@echo off
REM Launcher for the YAM test toolkit (cmd.exe / PowerShell / double-click).
REM Usage:  yam checkup   |   yam arm   |   yam live --mode jog
setlocal
set "PY=python"
where python >nul 2>nul && goto :run
where py >nul 2>nul && set "PY=py" && goto :run
if exist "C:\ProgramData\Anaconda3\python.exe" set "PY=C:\ProgramData\Anaconda3\python.exe"
:run
"%PY%" -m arm_test.cli %*
