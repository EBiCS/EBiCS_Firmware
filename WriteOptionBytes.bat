@echo off
rem Unlocks the chip (Read Out Protection -> Level 0). WARNING: this erases the whole flash.
rem The Java app passes the ST-Link path as %1. It is ignored because paths with spaces get split.
rem Uses STM32CubeProgrammer CLI (for STM32F1 level 0 = 0xA5), otherwise the old ST-LINK Utility.
setlocal EnableExtensions
set "CUBE=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "STLU=C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"

:try
echo.
echo ==== Unlock (RDP Level 0) ====
if exist "%CUBE%" goto cube
"%STLU%" -c SWD -OB RDP=0
goto check
:cube
"%CUBE%" -c port=SWD freq=400 -ob RDP=0xA5
:check
if errorlevel 1 goto failed
echo.
echo ==== HOTOVO: cip odemcen. Odpoj a pripoj napajeni controlleru, pak flashuj. ====
goto end

:failed
echo.
echo ==== UNLOCK SELHAL ====
echo Zkontroluj SWDIO/SWCLK/GND a napajeni controlleru.
choice /c AN /m "Zkusit znovu? A = ano, N = konec"
if errorlevel 2 goto end
goto try

:end
pause
exit
