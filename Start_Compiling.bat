set eclipsepath=%~1
set stmpath=%~2

PATH = %PATH%;%eclipsepath%\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.17.0.201812190825\tools\make;%eclipsepath%\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.17.0.201812190825\tools\compiler\bin;%stmpath%\STM32 ST-LINK Utility\ST-LINK Utility
IF NOT exist Debug (md Debug)
cd Debug
make -f ..\make\Makefile clean

set flashoption=%~3

cd..\make
copy *.* ..\Debug
cd ..\Debug
md Drivers\STM32F1xx_HAL_Driver\Src
md startup
md src

copy subdir_drivers.mk Drivers\STM32F1xx_HAL_Driver\Src\subdir.mk
copy subdir_src.mk Src\subdir.mk
copy subdir_startup.mk startup\subdir.mk

make all
IF "%flashoption%"=="STLink" (

ST-LINK_CLI.exe -c SWD -P LishuiFOC_01.hex -V
) ELSE (
hex2lsh.jar
lishuiFlash %~4 LishuiFOC_01.lsh
)

pause

exit

