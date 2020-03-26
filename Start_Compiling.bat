PATH = %PATH%;%1\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.17.0.201812190825\tools\make;%1\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.17.0.201812190825\tools\compiler\bin;%2\STM32 ST-LINK Utility\ST-LINK Utility
IF NOT exist Debug (md Debug)
cd Debug
make -f ..\make\Makefile clean


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

ST-LINK_CLI.exe -c SWD -P LishuiFOC_01.hex -V

pause

exit

