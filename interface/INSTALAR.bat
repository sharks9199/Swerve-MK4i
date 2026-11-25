@echo off
cd /d "%~dp0"
py -3.12 -m venv .venv
if errorlevel 1 goto erro
.venv\Scripts\python.exe -m pip install -r requirements.txt
if errorlevel 1 goto erro
echo Instalacao concluida.
pause
exit /b 0
:erro
echo Instale Python 3.12 de 64 bits e tente novamente.
pause
exit /b 1
