#!/bin/bash 
echo %1%
while true 
	:loop 
		echo next 
		"C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe" -c port=SWD -e all -w %1%
		echo sleep for 1 and next 
		timeout /t 5 /nobreak >nul
	goto loop