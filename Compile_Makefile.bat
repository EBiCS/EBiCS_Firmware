set eclipsepath=C:\Ac6\SystemWorkbench\
set stmpath=%~2
set flashoption=%~3

PATH = %PATH%;%eclipsepath%\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.17.0.201812190825\tools\make;%eclipsepath%\plugins\fr.ac6.mcu.externaltools.arm-none.win32_1.17.0.201812190825\tools\compiler\bin;%stmpath%\STM32 ST-LINK Utility\ST-LINK Utility


make all


