# Project with FreeRTOS

For this project you should set the environment variable FREERTOS_ROOT to point to your local FreeRTOS installation. The makefile points to a default location too but that is probably not where your FreeRTOS lives. Specifically the environment variable should point to the FreeRTOS/Source directory from the standard install.

```make
# if the environment variable is not set, this is used
FREERTOS_ROOT?=../../../FreeRTOSv10.2.0/FreeRTOS/Source
```

This project has a few basic tasks and a CLI. The tasks a) check the power supply status bits and b) handle the LED. From the CLI the i2c bus is available for interactive read/write (currently only I2C1).  Example console screen from UART:
```shell
----------------------------
Staring Project2 v0.1.0-3-g98e2ad0-dirty (FreeRTOS scheduler about to start)
----------------------------
FreeRTOS command server.
Type "help" to view a list of registered commands.
% pwr 0
Calling command >pwr 0<
% 
PowerSupplyTask: power supplied turned off.

% task-stats
Calling command >task-stats<
Task		Time		Fraction
************************************************
CON		0		<1%
IDL		938		99%
LED		0		<1%
POW		2		<1%
```


## Next steps
* I2C monitoring task. Define what to store in a local memory
* I2C slave for access from Zynq
* PMBus implementation
* digitize and store ADC voltages for access via CON and ADC
* Explore FreeRTOS-MPU for more robust OS (uses Cortex-M4 MPU facility)
