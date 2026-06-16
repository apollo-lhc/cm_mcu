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

## LED Status Signals

The board has a single RGB LED that communicates system state. The LED task runs every 250 ms; blink rates are expressed in multiples of that tick.

| State | Color | Pattern | Condition |
|-------|-------|---------|-----------|
| INIT | Blue | Solid | MCU startup, peripherals initializing |
| PS_OFF (idle) | Blue | Slow blink (~0.5 Hz) | MCU running, waiting for blade power enable |
| PS_LOADING | Cyan (G+B) | Slow blink (~0.5 Hz) | Power supplies ramping up (L1–L6 sequence) |
| NORMAL | Green | Solid | Fully operational |
| WARN | Red | Medium blink (~0.67 Hz) | Temperature above warning threshold |
| ALARM | Red | Solid | Temperature alarm triggered, power shut off |
| PS_FAULT | Red | Fast blink (~1 Hz) | Power supply hardware failure |
| FW_FAULT | Magenta (R+B) | Fast blink (~1 Hz) | Firmware / watchdog fault |
| BOOTLOADER | White (R+G+B) | Solid | Bootloader mode |

Normal power-up progression: **Blue solid → Blue slow blink → Cyan slow blink → Green solid**

To set the LED from code, send a `LedMsg_t` to `xLedQueue`:
```c
xQueueSendToBack(xLedQueue, &LED_STATUS_NORMAL, pdMS_TO_TICKS(10));
```

The `led` CLI command can force a state for debugging:
```
led init | idle | load | normal | warn | alarm | psfault | fwfault | white
```

## I2C bus access and semaphores

Each I2C master bus (1–6) is protected by its own FreeRTOS mutex (`i2c1_sem` … `i2c6_sem`, see
`Semaphore.c`). Any task that issues a transaction sequence on a bus must hold that bus's mutex for
the duration. The monitor tasks, the `clocksynth`/`FireflyUtils` helpers, and the `commands/`
handlers (`FireflyCommands`, `ClockCommands`, `PowerCommands`) all acquire the correct per-bus mutex.

**Deliberate exception — the raw CLI commands.** The generic raw-access commands in
`commands/I2CCommands.c` (`i2cr`, `i2crr`, `i2cw`, `i2cwr`, `i2c_scan`) **intentionally do not take
the semaphore.** This lets a user compose a longer interactive sequence under a *manually* held lock
via the `sem_ctl <bus> take` / `sem_ctl <bus> release` command (`commands/SoftwareCommands.c`).
The intended workflow is:

```
sem_ctl 2 take      # manually grab the bus-2 (clocks) mutex
i2cw  2 0x70 1 0x4  # ... raw transactions, mutually excluded from the monitors ...
i2cr  2 0x6b 1
sem_ctl 2 release   # hand the bus back to the monitor tasks
```

This still works under the interrupt/notification-based I2C driver: the I2C completion handshake
(`ulTaskNotifyTake` / `vTaskNotifyGiveFromISR`) and the UART stream buffer both use task-notification
index 0, but they never overlap — the stream buffer only notifies the CLI task while it is parked
inside `xStreamBufferReceive`, which is mutually exclusive with the task waiting on an I2C
transaction. **Caveat / footgun:** this safety relies on that single shared notification index never
being contended. If the CLI task is ever made to wait on its task notification for another purpose,
the raw-command + I2C path will break silently. The robust fix is to give I2C completion its own
notification index (`configTASK_NOTIFICATION_ARRAY_ENTRIES = 2`, using the `…Indexed` notify APIs),
leaving index 0 to the stream buffer. See `i2c_lockup_notes.md` for the full analysis.

### Higher-level semaphore-free helpers and unprogrammed exploration

A few *higher-level* helpers also intentionally skip the bus mutex so they stay usable for expert
exploration and when a bus is wedged (i.e. when the mutex is held by a stuck transaction and blocking
on it would defeat the diagnostic). The current examples are `readFFpresentSignals(acquire_sem=false)`
and its CLI front-end `ff_present`, which walk the F1/F2 optics I/O-expander muxes on buses 3 and 4.

**This is no longer a memory-safety hazard.** The original crash was a *stack* use-after-return — the
SMBus ISR writing into a freed/reused stack frame after the wrapper returned. The wrappers now use
persistent per-bus `.bss` scratch buffers (`i2c_txbuf` / `i2c_rxbuf` in `I2CCommunication.c`) instead
of stack locals, so a late ISR or a concurrent caller can only ever produce a *wrong value* — never
corrupt an unrelated frame.

**What is still shared and unprotected if you skip the mutex while a monitor task runs the same bus:**

- the per-bus scratch buffers — a concurrent `MonitorTaskI2C` transaction can interleave bytes into
  the same `i2c_txbuf[bus]`/`i2c_rxbuf[bus]`, giving a garbage reading;
- `TaskNotifySMBus[bus]`, the *single* per-bus completion slot — last writer wins, so the loser never
  gets its `vTaskNotifyGiveFromISR` and waits out the full 250 ms timeout, returning `SMBUS_TIMEOUT`.

Neither corrupts memory; both just make the result unreliable.

**Safe workflow today:** wrap the exploration in a manually held lock, exactly like the raw commands
above — `sem_ctl 3 take` / `sem_ctl 4 take`, run the semaphore-free helper(s), then `sem_ctl … release`.
That excludes the monitor tasks for the duration and makes the shared buffer/notify-slot races moot.

**For future consideration** (increasing robustness), if the manual-lock workflow proves too easy to
forget:

1. Default the one-shot diagnostics (`ff_present`) to a *bounded-timeout* acquire
   (`acquireI2CSemaphoreTime(s, pdMS_TO_TICKS(100))`) and print `bus busy, try again` on failure;
   keep an explicit `force` argument for the genuine wedged-bus case rather than skipping the lock by
   default.
2. Give the `force`/bypass path its **own dedicated scratch buffer**, separate from the monitor-task
   buffers, so even bypass mode can never alias `i2c_txbuf`/`i2c_rxbuf`. (The notify-slot race
   remains, but only costs a 250 ms timeout — not corruption.)
3. Add a maintenance **pause** (`mon_pause` / `mon_resume`) that `vTaskSuspend`s the three
   `MonitorTaskI2C` instances. This is the only option that gives exploration *exclusive*,
   uncontended bus access **and** persistent mux state across a sequence of commands — the monitor
   tasks otherwise reset the mux to 0 between their own transactions. It needs the task handles, which
   are currently created with `NULL` in `cm_mcu.c`; the cleanest wiring is for each task to store
   `xTaskGetCurrentTaskHandle()` into its own `args` struct at startup, which the CLI already
   references.

## Logging and the log mutex

The logger is a modified copy of [rxi/log.c](https://github.com/rxi/log.c) in `common/log.c`. The
Apollo-specific sink `ApolloLog()` does two things per line: it appends the formatted text to an
in-RAM circular scrollback buffer (`log_buffer` / `struct buff_t b`, dumpable from the CLI via
`log_dump`), and it writes the line to the UART(s) through `Print()`.

`Print()` busy-waits one character at a time on the UART FIFO (`MAP_UARTCharPut`), so emitting a full
line takes on the order of the line's transmit time (several ms at typical baud). `Print()` is
independently serialized by its own `xUARTMutex`.

**The log mutex (`log_sem`) protects only the shared scrollback buffer**, not the UART. It is wired in
via rxi's `log_set_lock()` hook, with `vGiveOrTakeSemaphore()` (in `Semaphore.c`) as the callback:

- The critical section is just the `log_add_string()` append inside `ApolloLog()` — microseconds.
  `Print()` is deliberately left *outside* the log mutex so a slow UART transmit in one task never
  blocks another task's logging.
- Acquire is **non-blocking** (`xSemaphoreTake(s, 0)`). The `log_LockFn` contract returns `bool`; on a
  failed acquire `ApolloLog()` **drops the scrollback append and still prints the line**. Logging must
  never stall a task on the lock, and the live UART output is never lost — only the in-RAM copy.
- `log_set_lock()` captures the mutex handle by value, so `initSemaphores()` **must run before**
  `log_set_lock()` in `main()`. With the order reversed the handle is still `0` and the lock is
  silently a no-op.
- Polarity: the `log_LockFn` first argument is `true` to **acquire**, `false` to release
  (`vGiveOrTakeSemaphore(bool take, …)` — `take==true` ⇒ `xSemaphoreTake`).

**Constraint:** `log_*` is task-context only. The lock uses `xSemaphoreTake`/`Give`, which must not be
called from an ISR. ISR-side logging would need the `FromISR` variants or a separate lockless path.

## Temperature sources and stale-value invalidation (`TempStatus`)

`TempStatus()` (`AlarmUtilities.c`) computes the per-device max temperature each cycle:

- **TM4C**: on-chip ADC (always valid).
- **DCDC**: max over `dcdc_args.pm_values` (I2C/PMBus; monitored on management power, `requirePower=false`).
- **FPGA**: max over `fpga_args.pm_values` (I2C) **and**, on REV2/REV3, the MCU-ADC FPGA diode temps
  (`ADC_INFO_F1_TEMP_ENTRY`/`F2_TEMP_ENTRY`, ADC CH18/19). The ADC diodes are power-independent and read
  sanely even when the FPGA is unpowered (bench-verified), so REV2/3 evaluates the FPGA alarm regardless
  of power state. REV1 (no FPGA diodes) skips the FPGA/FF checks when power is off.
- **Firefly**: max `getFFtemp()` (whole °C, `uint16_t`), gated by `isFFStale()`; checked only when powered.

**Stale-value invalidation (fix for the alarm re-triggering after a power cycle):** when a
`requirePower` `MonitorTask` (FPGA) sees power leave `POWER_ON`, it clears its `pm_values[]` to the
`-999.f` sentinel and backdates `updateTick` 60 s into the past. Both are needed: `-999` is ignored by
numeric consumers (the `TempStatus` max-loops seed at `-99`, so `-999` never wins), while the stale
`updateTick` makes `checkStale()`-based consumers (notably `I2CSlaveTask`, which casts `pm_values` to
`uint8_t`) reject the entry instead of reading the sentinel. `-999.f` is the codebase-wide "no/stale
data" sentinel for `pm_values` (also the boot-time init in `cm_mcu.c`).

## Building FreeRTOS

FreeRTOS is now included as a git submodule. 
