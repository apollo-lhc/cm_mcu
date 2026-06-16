# Production Test Firmware

This firmware is intended for Rev2 and later revisions (rev1 not supported).
If you compile and run it on a rev 1 board, caveat emptor.

`prod_test` is a minimal FreeRTOS application — no alarm, EEPROM, or monitor tasks. It has three tasks:
`vCommandLineTask` (UART0 CLI), `ADCMonitorTask` (continuous on-chip ADC sampling, read via the `adc`
command), and `I2CSlaveTask` (responds to an external I2C master on I2C0, slave address `0x40`).

## CLI commands

| Command | Description |
| ------- | ----------- |
| `adc` | Display ADC voltage/current measurements with pass/fail |
| `poweron <level>` | Enable power supplies up to the given priority level |
| `poweroff` | Disable all power supplies |
| `dcdci2ctest` | Test I2C communication to DCDC converters |
| `dcdcpowertest` | Test DCDC power-on and voltage levels |
| `clocki2ctest` | Test I2C to clock synthesizers |
| `ffi2ctest [mask]` | Test I2C to Firefly optics (optional channel mask) |
| `eepromi2ctest` | Test I2C to external EEPROM |
| `fpgai2ctest` | Test I2C to FPGAs |
| `initclockreg` | Initialize clock synthesizer IO expanders |
| `initffreg` | Initialize Firefly IO expanders |
| `prodtest1 [mask]` | Run full first-step production test sequence (see below) |
| `version` | Display firmware version and build time |
| `heap` | Show free and minimum-ever-free heap |
| `bootloader` | Jump to ROM bootloader |
| `restart` | Reset the MCU |

## `prodtest1` sequence (stops on first failure)

1. DCDC I2C connectivity check
2. DCDC power-on and voltage verification
3. Initialize clock IO expanders
4. Clock I2C test
5. FPGA I2C test
6. Initialize Firefly IO expanders
7. Firefly I2C test
8. External EEPROM I2C test
