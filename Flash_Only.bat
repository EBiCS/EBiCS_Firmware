@echo off
rem Flashes the already compiled firmware Debug\LishuiFOC_01.hex, without recompiling.
rem If flashing fails, it offers a retry (e.g. after fixing the cable or powering the controller).
rem Uses STM32CubeProgrammer CLI if installed, otherwise the old ST-LINK Utility.
setlocal EnableExtensions
cd /d "%~dp0"

set "HEX=%~dp0Debug\LishuiFOC_01.hex"
set "CUBE=C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
set "STLU=C:\Program Files (x86)\STMicroelectronics\STM32 ST-LINK Utility\ST-LINK Utility\ST-LINK_CLI.exe"

if not exist "%HEX%" goto nohex

:try
echo.
echo ==== Flash %HEX% ====
if exist "%CUBE%" goto cube
"%STLU%" -c SWD -P "%HEX%" -V -Rst
goto check
:cube
"%CUBE%" -c port=SWD -w "%HEX%" -v -rst
:check
if errorlevel 1 goto failed
echo.
echo ==== HOTOVO: firmware nahran a overen ====
goto end

:failed
echo.
echo ==== FLASH SELHAL ====
echo Zkontroluj SWDIO/SWCLK/GND, napajeni controlleru a pripadne Read Protection (WriteOptionBytes.bat).
choice /c AN /m "Zkusit flash znovu? A = ano, N = konec"
if errorlevel 2 goto end
goto try

:nohex
echo CHYBA: %HEX% neexistuje. Nejdriv zkompiluj (Java app nebo Start_Compiling.bat).

:end
if /i not "%~1"=="nopause" pause
endlocal
