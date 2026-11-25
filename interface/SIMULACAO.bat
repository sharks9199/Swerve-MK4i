@echo off
cd /d "%~dp0"
if not exist .venv\Scripts\python.exe (
 echo Execute INSTALAR.bat primeiro.
 pause
 exit /b 1
)
.venv\Scripts\python.exe app.py --robot 127.0.0.1
pause
