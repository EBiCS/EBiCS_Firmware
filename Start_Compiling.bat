@echo off
rem Called by the Java configurator as:
rem   Start_Compiling <eclipsepath> <stmpath> <flashoption> void
rem The Java app does not quote the paths, so a path with spaces gets split.
rem That is why the toolchain is found automatically and the arguments are only a hint.
setlocal EnableExtensions
cd /d "%~dp0"

set "eclipsepath=%~1"
if "%eclipsepath%"=="" set "eclipsepath=C:\Ac6\SystemWorkbench"
if not exist "%eclipsepath%\plugins" set "eclipsepath=C:\Ac6\SystemWorkbench"

rem Find the arm-none toolchain plugin of any version
set "TOOLS="
for /d %%D in ("%eclipsepath%\plugins\fr.ac6.mcu.externaltools.arm-none.win32_*") do set "TOOLS=%%D\tools"
if not defined TOOLS goto notools
if not exist "%TOOLS%\compiler\bin\arm-none-eabi-gcc.exe" goto notools
set "PATH=%TOOLS%\make;%TOOLS%\compiler\bin;%PATH%"

echo ==== Kompilace (toolchain: %TOOLS%) ====
IF NOT exist Debug (md Debug)
cd Debug
make -f ..\make\Makefile clean

cd ..\make
copy /y *.* ..\Debug >nul
cd ..\Debug
md Drivers\STM32F1xx_HAL_Driver\Src 2>nul
md startup 2>nul
md src 2>nul

copy /y subdir_drivers.mk Drivers\STM32F1xx_HAL_Driver\Src\subdir.mk >nul
copy /y subdir_src.mk Src\subdir.mk >nul
copy /y subdir_startup.mk startup\subdir.mk >nul

copy /y STLINK_FLASH.ld ..\STM32F103C6Tx_FLASH.ld >nul
copy /y STLINK_system_stm32f1xx.c ..\Src\system_stm32f1xx.c >nul

make all
if errorlevel 1 goto buildfail
if not exist LishuiFOC_01.hex goto buildfail
cd ..

rem Flashing (with a retry option) is in Flash_Only.bat.
rem It can also be started on its own to flash the already compiled firmware again.
call "%~dp0Flash_Only.bat" nopause
pause
exit

:notools
echo.
echo CHYBA: ARM toolchain nenalezen v "%eclipsepath%\plugins\fr.ac6.mcu.externaltools.arm-none.win32_*\tools\compiler\bin"
pause
exit

:buildfail
echo.
echo CHYBA: kompilace selhala, nic se neflashuje.
pause
exit
