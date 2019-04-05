# Project with FreeRTOS

For this project you should set the environment variable FREERTOS_ROOT to point to your local FreeRTOS installation. The makefile points to a default location too but that is probably not where your FreeRTOS lives. Specifically the environment variable should point to the FreeRTOS/Source directory from the standard install.

```make
# if the environment variable is not set, this is used
FREERTOS_ROOT?=../../../FreeRTOSv10.2.0/FreeRTOS/Source
```

This project has a few basic tasks and a CLI. The tasks a) check the power supply status bits and b) handle the LED. From the CLI I intend to expose the i2c bus for interactive read/write.
