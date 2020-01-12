PATH = %PATH%;C:\GNU_ARM\eclipse\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.16.0.201807130628\tools\make;C:\GNU_ARM\eclipse\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.16.0.201807130628\tools\compiler\bin
cd debug

make clean


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

pause

