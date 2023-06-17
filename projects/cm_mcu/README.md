# Main project for the Apollo CM microcontroller.


This project has the source code for the firmware that runs on the microcontroller to provide low-level control of the power supplies, low-level monitoring of temperatures, voltages and currents, monitoring information that is provided the to the Apollo Service Module and to the IPMC via an I2C worker, an error logger to allow basic debugging after-the-fact, and a  UART-based command line interface (CLI), available either from the front panel or from the service module.

Example of the CLI (this is from version 0.28.):

```
% version
Version v0.28 built at 13:53:27, Sep 28 2020.
% task-stats
Task            Time     %
********************************
CLIFP           4071    <1%
IDLE      1620425999    96%
POW           409614    <1%
PSMON       15085622    <1%
FFLY               5    <1%
XIMON        3807859    <1%
LED                0    <1%
ADC                0    <1%
SUART       43800249     2%
TALM               0    <1%
CLIZY             36    <1%
I2CS0              0    <1%
EPRM              12    <1%
INIT               0    <1%
% 
```

## Code structure
Upon boot, the TM4C first calls the reset vector code (`ResetISR` in startup_gcc.c). The code is copied from on-board flash to SRAM, initializes the vector table and the various memory regions. Then the code jumps to `main`, which is in cm_mcu.c. 

In `main` the initalization sequence first sets up the pins of the TM4C (special function, direction, etc), then sets up various semaphores, mutexes, queues, data strucutres and interrupts (without enabling the global interrupt), adds tasks to the FreeRTOS scheduler, and finally starts the FreeRTOS scheduler. (The scheduler also enables the global interrupts.)

Tasks are located in their own C files and are identified by names such as `XXXTask.c`. Most tasks have the following structure: a short initialization block, followed by an infinite loop with a delay. 


``` c
void Task(void *parameters)
{
  // do some initialization
  // ... 
  
  // initialize to the current tick time right before main task loop starts
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    // do stuff
    // ...
    // wait here for the x msec, where x is 2nd argument below.
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(25));    
  }

}


```

The `vTaskDelayUntil` sets the frequency of the tasks being called, for tasks that are not real-time critical. 

The microcontroller runs at 80 MHz and is idle most of the time. 

The vector table is defined in `startup_gcc.c`. Interrupt handlers are either in `startup_gcc.c` or in `InterruptHandlers.c`.  FreeRTOS's configuration is in the standard   `FreeRTOSConfig.h`.


## List of Tasks
| *Task Source code* | *Short description* | 
| --- | --- | 
| ADCMonitorTask.c | Reads out TM4C's built-in ADC to monitor board voltages. Uses TM4C built-in ADCs. |
| CommandLineTask.c | CLI interface. Instantiated twice (front panel and SM). Uses a hardware UART. |
| EEPromTask.c | Gatekeeper task to control access to the built-in eeprom. | 
| FireFlyTask.c | Collects monitoring information from SamTec Firefly devices, to be acted on by the alarm task. Acts as an I2C controller. Uses hardware I2C device. | 
| GenericAlarmTask.c | Generic alarm task, with call-backs for how to react. |
| I2CSlaveTask.c | Provides data to the IPMC. Acts as an I2C target. Uses hardware I2C device.| 
| InitTask.c | Initalization task; runs once then exits. |
| LedTask.c  | Gatekeeper task for GPIO control of external LEDs. |
| MonitorTask.c | Task to collect monitoring information, to be acted on by alarm task. Acts as an I2C controller. Instantiated twice, this task reads out data from the LGA80D power supplies or the FPGAs. Uses hardware I2C device. | 
| PowerSupplyTask.c | Turns non-management power on and off on the board in proper order. Uses GPIOs. |
| ZynqMonTask.c | Sends monitoring data to the Zynq SoC on the Apollo SM. Acts as a soft UART, with only the Tx function implemented. Uses a hardware timer and GPIOs.|  

## Building FreeRTOS

FreeRTOS is now included as a git submodule. 

```make
```
