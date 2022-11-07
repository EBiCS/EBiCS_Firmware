PATH = %PATH%;%1\STM32 ST-LINK Utility\ST-LINK Utility

ST-LINK_CLI.exe -c SWD SWCLK=8 -OB RDP=0

pause

exit