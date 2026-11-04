@echo off
cd /d "C:\Users\Lenovo\Downloads\main_project"
echo ========================================================
echo   DANG BIEN DICH VA NAP CODE VAO MCU SONIX SN32F407
echo ========================================================
powershell -NoProfile -ExecutionPolicy Bypass -File ".\build_and_flash.ps1"
echo.
pause
