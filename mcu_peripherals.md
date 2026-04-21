# Apollo CM MCU Hardware Pin and Peripheral Assignments

**MCU**: TI TM4C1290NCPDT (ARM Cortex-M4F, 80 MHz)  
**Sources**: `common/`pinout_rev1`.c`, `common/`pinout_rev2`.c`, `common/`gpio_pins_rev2`.def`, `common/pinsel.h`, `projects/`cm_mcu`/ADCMonitorTask.c`, `common/LocalUart.c`, `common/`i2c_reg`.c`

---

## Dedicated Peripherals

### UART

| Peripheral | TX Pin | RX Pin | Baud | REV1 Function | REV2/3 Function |
|------------|--------|--------|------|---------------|-----------------|
| UART0 | PA1 (U0TX) | PA0 (U0RX) | 115200 | (unused / debug) | `ZQ_UART` — Zynq-facing CLI |
| UART1 | PB1 (U1TX) | PB0 (U1RX) | 115200 | `ZQ_UART` — Zynq-facing CLI | — (PB0/1 reassigned to I2C5) |
| UART3 | PA5 (U3TX) | PA4 (U3RX) | — | — (PA4/5 are GPIO outputs) | pins muxed in `pinout_rev2.c` but peripheral **not initialized** |
| UART4 | PA3 (U4TX) | PA2 (U4RX) | 115200 | `FP_UART` — front-panel CLI | `ZynqMonTask` TX-only output to Zynq (hardware UART, no soft-UART) |

UART0, UART1 (REV1), and UART4 have RX+RT interrupts enabled with handlers in `InterruptHandlers.c`. In REV2/3, `FP_UART` is not defined — there is no separate front-panel CLI UART.

---

### I2C

| Peripheral | SCL Pin | SDA Pin | Mode | Function |
|------------|---------|---------|------|----------|
| I2C0 | PB2 | PB3 | **Slave** (addr 0x40) | I2C slave interface to external master (`I2CSlaveTask`) |
| I2C1 | PG0 | PG1 | SMBus Master | DCDC / power supply monitoring |
| I2C2 | PG2 | PG3 | SMBus Master | Clock synthesizers (Si5395) |
| I2C3 | PG4 | PG5 | SMBus Master | F2 Firefly optics |
| I2C4 | PG6 | PG7 | SMBus Master | F1 Firefly optics |
| I2C5 | PB0 | PB1 | SMBus Master | FPGA monitoring (REV2/3 only) |
| I2C6 | PA6 | PA7 | SMBus Master | FPGA monitoring (REV1 only) |

All I2C masters use interrupts at `configKERNEL_INTERRUPT_PRIORITY`; I2C0 slave uses `I2C0SlaveIntHandler`.

---

### ADC

| Peripheral | Sequence | Reference | Interrupt | Usage |
|------------|----------|-----------|-----------|-------|
| ADC0 | Seq 1 | External 3 V | `INT_ADC0SS1` | Voltage monitoring (first 4–8 channels) |
| ADC1 | Seq 0 | External 3 V | `INT_ADC1SS0` | Voltage / current / temperature monitoring |

Internal TM4C temperature sensor sampled via `ADC_CTL_TS`.

---

### Timer

| Peripheral | Sub-timer | Priority | REV1 Usage | REV2/3 Usage |
|------------|-----------|----------|------------|--------------|
| TIMER0A | Timer A (periodic) | 0x80 (high) | Bit-time counter for ZynqMon **software UART** (`ZynqMonTask.c`) | not used (ZynqMon uses hardware UART4 instead) |

**Note**: Timer 0A priority (REV1) must not be changed — it is deliberately higher than FreeRTOS-managed interrupts.

---

### Internal Peripherals

| Peripheral | Usage |
|------------|-------|
| EEPROM0 | Non-volatile storage: board identity, alarm thresholds, power-supply ignore mask, error log ring buffer |
| Hibernate module | RTC (`InitRTC()`) — REV2/3 only |

---

## ADC Channel Signal Assignments

The ADC channel numbers are fixed by the TM4C silicon; signals differ between board revisions.

### GPIO → ADC Channel Map (all revisions, from `pinout_rev1.c` / `pinout_rev2.c`)

| GPIO Pin | ADC Channel | Hex |
|----------|------------|-----|
| PE3 | `AIN0_1` | CH0 |
| PE2 | `AIN1_1` | CH1 |
| PE1 | `AIN2_1` | CH2 |
| PE0 | `AIN3_1` | CH3 |
| PD7 | AIN4 | CH4 |
| PD6 | AIN5 | CH5 |
| PD5 | AIN6 | CH6 |
| PD4 | AIN7 | CH7 |
| PE5 | AIN8 | CH8 |
| PE4 | AIN9 | CH9 |
| PB4 | AIN10 | CH10 |
| PB5 | AIN11 | CH11 |
| PD3 | `AIN12_1` | CH12 |
| PD2 | `AIN13_1` | CH13 |
| PD1 | `AIN14_1` | CH14 |
| PD0 | `AIN15_1` | CH15 |
| PK0 | `AIN16_1` | CH16 |
| PK1 | `AIN17_1` | CH17 |
| PK2 | `AIN18_1` | CH18 |
| PK3 | `AIN19_1` | CH19 |

### ADC Signal Names by Revision (`ADCMonitorTask.c`)

| GPIO Pin | ADC Ch | REV1 Signal | REV2/3 Signal |
|----------|--------|-------------|---------------|
| PE3 | CH0 | `F2_MGTY1_AVTT` | `VCC_12V` |
| PE2 | CH1 | `F2_MGTY1_AVCC` | `VCC_M3V3` |
| PE1 | CH2 | `F2_MGTY1_VCCAUX` | `VCC_3V3` |
| PE0 | CH3 | `F2_VCCINT` | `VCC_4V0` |
| PD7 | CH4 | `F1_MGTY_AVTT` | `VCC_1V8` |
| PD6 | CH5 | `F1_MGTY_AVCC` | `F1_VCCINT` |
| PD5 | CH6 | `F1_MGTY_VCCAUX` | `F1_AVCC` |
| PD4 | CH7 | `VCC_1V8` | `F1_AVTT` |
| PE5 | CH8 | `F1_VCCINT` | `F1_VCCAUX` |
| PE4 | CH9 | `F1_MGTH_VCCAUX` | `F2_VCCINT` |
| PB4 | CH10 | `F1_MGTH_AVCC` | `F2_AVCC` |
| PB5 | CH11 | `F1_MGTH_AVTT` | `F2_AVTT` |
| PD3 | CH12 | `VCC_12V` | `F2_VCCAUX` |
| PD2 | CH13 | `VCC_2V5` | `CUR_V_12V` |
| PD1 | CH14 | `VCC_M3V3` | `CUR_V_M3V3` |
| PD0 | CH15 | `VCC_M1V8` | `CUR_V_4V0` |
| PK0 | CH16 | `VCC_3V3` | `CUR_V_F1VCCAUX` |
| PK1 | CH17 | `F2_MGTY2_VCCAUX` | `CUR_V_F2VCCAUX` |
| PK2 | CH18 | `F2_MGTY2_AVCC` | `F1_TEMP` (°C) |
| PK3 | CH19 | `F2_MGTY2_AVTT` | `F2_TEMP` (°C) |
| (internal) | TS | `TM4C_TEMP` (°C) | `TM4C_TEMP` (°C) |

---

## GPIO Pin Assignments

`Dir` abbreviations: **In** = input, **Out** = push-pull output, **OD** = open-drain output.

Pin numbers are 128-pin TQFP package pin numbers. Numbers marked with † come directly from `gpio_pins_rev1.def` / `gpio_pins_rev2.def` (generated by TI PinMux); others are derived from the package pinout.

### Port A

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PA0 | 33 | UART | UART0 RX (U0RX) | UART | UART0 RX (U0RX) |
| PA1 | 34 | UART | UART0 TX (U0TX) | UART | UART0 TX (U0TX) |
| PA2 | 35 | UART | UART4 RX (U4RX) | UART | UART4 RX (U4RX) |
| PA3 | 36 | UART | UART4 TX (U4TX) | UART | UART4 TX (U4TX) |
| PA4 | 37† | Out | `CTRL_V_VCCINT_PWR_EN` | UART | UART3 RX (U3RX) |
| PA5 | 38† | Out | `CTRL_VCC_1V8_PWR_EN` | UART | UART3 TX (U3TX) |
| PA6 | 39 | I2C | I2C6 SCL (FPGA I2C, REV1) | — | not used |
| PA7 | 40 | I2C | I2C6 SDA (FPGA I2C, REV1) | — | not used |

### Port B

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PB0 | 18 | UART | UART1 RX (U1RX) | I2C | I2C5 SCL (FPGA I2C) |
| PB1 | 19 | UART | UART1 TX (U1TX) | I2C | I2C5 SDA (FPGA I2C) |
| PB2 | 20 | I2C | I2C0 SCL (slave) | I2C | I2C0 SCL (slave) |
| PB3 | 21 | I2C | I2C0 SDA (slave) | I2C | I2C0 SDA (slave) |
| PB4 | 16 | ADC | AIN10 → `F1_MGTH_AVCC` | ADC | AIN10 → `F2_AVCC` |
| PB5 | 17 | ADC | AIN11 → `F1_MGTH_AVTT` | ADC | AIN11 → `F2_AVTT` |

Note: PB4/PB5 (ADC-capable) have lower pin numbers than PB0–PB3 because they are placed adjacent to the ADC input bank on the package.

### Port C

PC0–PC3 are reserved for JTAG (SWDIO/SWDCLK/TDI/TDO) and must not be reconfigured.

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PC4 | 25† | In | `V_MGTY2_AVTT_OK` | — | not used |
| PC5 | 24† | In | `V_MGTY2_AVCC_OK` | — | not used |
| PC6 | 23† | In | `V_MGTY1_AVCC_OK` | Out | `F2_FPGA_PROGRAM` |
| PC7 | 22† | In | `V_MGTY1_AVTT_OK` | In | `PG_4V0` (4 V power-good) |

### Port D (ADC — same pin mux both revisions)

| Pin | MCU Pin | ADC Ch | REV1 Signal | REV2/3 Signal |
|-----|---------|--------|-------------|---------------|
| PD0 | 1 | CH15 | `VCC_M1V8` | `CUR_V_4V0` |
| PD1 | 2 | CH14 | `VCC_M3V3` | `CUR_V_M3V3` |
| PD2 | 3 | CH13 | `VCC_2V5` | `CUR_V_12V` |
| PD3 | 4 | CH12 | `VCC_12V` | `F2_VCCAUX` |
| PD4 | 91 | CH7 | `VCC_1V8` | `F1_AVTT` |
| PD5 | 90 | CH6 | `F1_MGTY_VCCAUX` | `F1_AVCC` |
| PD6 | 89 | CH5 | `F1_MGTY_AVCC` | `F1_VCCINT` |
| PD7 | 88 | CH4 | `F1_MGTY_AVTT` | `VCC_1V8` |

PD7 requires LOCK/COMMIT register unlock before pin-mux configuration (shared NMI function).  
PD0–PD3 (AIN alternate-bank) and PD4–PD7 (AIN primary-bank) are in different quadrants of the package.

### Port E (ADC — same pin mux both revisions)

| Pin | MCU Pin | ADC Ch | REV1 Signal | REV2/3 Signal |
|-----|---------|--------|-------------|---------------|
| PE0 | 7 | CH3 | `F2_VCCINT` | `VCC_4V0` |
| PE1 | 8 | CH2 | `F2_MGTY1_VCCAUX` | `VCC_3V3` |
| PE2 | 9 | CH1 | `F2_MGTY1_AVCC` | `VCC_M3V3` |
| PE3 | 10 | CH0 | `F2_MGTY1_AVTT` | `VCC_12V` |
| PE4 | 12 | CH9 | `F1_MGTH_VCCAUX` | `F2_VCCINT` |
| PE5 | 13 | CH8 | `F1_VCCINT` | `F1_VCCAUX` |

### Port F

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PF0 | 42† | OD | `_FPGA_I2C_RESET` | In | `PG_F1_INT_A` (F1 FPGA power-good: INT rail A) |
| PF1 | 43† | Out | `CTRL_VCC_3V3_PWR_EN` | In | `PG_F1_INT_B` (F1 FPGA power-good: INT rail B) |
| PF2 | 44† | Out | `CTRL_K_VCCINT_PWR_EN` | In | `PG_F1_AVCC` (F1 FPGA power-good: AVCC) |
| PF3 | 45† | OD | `_PWR_I2C_RESET` | In | `PG_F1_AVTT` (F1 FPGA power-good: AVTT) |
| PF4 | 46† | In | `TM4C_TP1` (test point) | In | `PG_F1_VCCAUX` (F1 FPGA power-good: VCCAUX) |

### Port G (I2C — identical both revisions)

| Pin | MCU Pin | Function |
|-----|---------|----------|
| PG0 | 48 | I2C1 SCL → DCDC power supplies |
| PG1 | 49 | I2C1 SDA → DCDC power supplies |
| PG2 | 50 | I2C2 SCL → clock synthesizers |
| PG3 | 51 | I2C2 SDA → clock synthesizers |
| PG4 | 52 | I2C3 SCL → F2 Firefly optics |
| PG5 | 53 | I2C3 SDA → F2 Firefly optics |
| PG6 | 54 | I2C4 SCL → F1 Firefly optics |
| PG7 | 55 | I2C4 SDA → F1 Firefly optics |

PG0–PG7 pins (48–55) are derived: they occupy the consecutive block between PF4 (46) and PQ5 (57), with supply pins at 47 and 56.

### Port H

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PH0 | 29† | In | `VCC_1V8_PG` (1.8 V power-good) | OD | `_FPGA_I2C_RESET` |
| PH1 | 30† | In | `VCC_3V3_PG` (3.3 V power-good) | In (WPU) | `BLADE_POWER_EN` |
| PH2 | 31† | In | `F2_VCCINT_PG_B` | In | `PG_3V3` (3.3 V power-good) |
| PH3 | 32† | In | `F2_VCCINT_PG_A` | In | `PG_1V8` (1.8 V power-good) |

### Port J

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PJ0 | 116† | Out | `MCU_LED_BLUE` | In | `F1_C2C_OK` (F1 FPGA chip-to-chip link OK) |
| PJ1 | 117† | Out | `MCU_LED_GREEN` | In | `F2_C2C_OK` (F2 FPGA chip-to-chip link OK) |

### Port K

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PK0 | 68 | ADC | CH16 → `VCC_3V3` | ADC | CH16 → `CUR_V_F1VCCAUX` |
| PK1 | 67 | ADC | CH17 → `F2_MGTY2_VCCAUX` | ADC | CH17 → `CUR_V_F2VCCAUX` |
| PK2 | 66 | ADC | CH18 → `F2_MGTY2_AVCC` | ADC | CH18 → `F1_TEMP` |
| PK3 | 65 | ADC | CH19 → `F2_MGTY2_AVTT` | ADC | CH19 → `F2_TEMP` |
| PK4 | 62† | Out | `BLADE_POWER_OK` | In | `_F1_FPGA_DONE` |
| PK5 | 61† | In | `K_VCCINT_PG_A` | In | `PG_F2_VCCAUX` (F2 FPGA power-good: VCCAUX) |
| PK6 | 60† | In | `K_VCCINT_PG_B` | In | `PG_F2_AVTT` (F2 FPGA power-good: AVTT) |
| PK7 | 59† | OD | `_K_OPTICS_I2C_RESET` | In | `PG_F2_AVCC` (F2 FPGA power-good: AVCC) |

### Port L

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PL0 | 81† | Out | `K_FPGA_PROGRAM` | Out | `EN_F1_INT` (enable F1 INT supply) |
| PL1 | 82† | In | `TM4C_DIP_SW_1` (F1 installed?) | Out | `EN_F1_AVCC` (enable F1 AVCC) |
| PL2 | 83† | Out | `CTRL_K_MGTY_VCCAUX_PWR_EN` | Out | `EN_F1_AVTT` (enable F1 AVTT) |
| PL3 | 84† | Out | `CTRL_K_MGTH_VCCAUX_PWR_EN` | Out | `EN_F1_VCCAUX` (enable F1 VCCAUX) |
| PL4 | 85† | Out | `CTRL_K_MGTY_AVCC_PWR_EN` | Out | `EN_F2_INT` (enable F2 INT supply) |
| PL5 | 86† | Out | `CTRL_K_MGTH_AVCC_PWR_EN` | Out | `EN_F2_AVCC` (enable F2 AVCC) |
| PL6 | 94† | Out | `TM4C_TO_KU15P_0` | Out | `EN_F2_VCCAUX` (enable F2 VCCAUX) |
| PL7 | 93† | In | `TM4C_FROM_KU15P_0` | Out | `EN_F2_AVTT` (enable F2 AVTT) |

### Port M

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PM0 | 78† | In | `_K_FPGA_DONE` | Out | `EN_1V8` (enable 1.8 V supply) |
| PM1 | 77† | In | `K_MGTY_AVTT_OK` | Out | `EN_3V3` (enable 3.3 V supply) |
| PM2 | 76† | In | `K_MGTY_AVCC_OK` | Out | `BLADE_POWER_OK` |
| PM3 | 75† | In | `K_MGTH_AVCC_OK` | Out | `MCU_TO_F2` (MCU→F2 FPGA GPIO) |
| PM4 | 74† | In | `K_MGTH_AVTT_OK` | Out | `MCU_TO_F1` (MCU→F1 FPGA GPIO) |
| PM5 | 73† | In | `BLADE_ZYNQ_GPIO2` | In | `F2_TO_MCU` (F2 FPGA→MCU GPIO) |
| PM6 | 72† | In | `BLADE_ZYNQ_GPIO1` / soft-UART TX (REV1) | In | `F1_TO_MCU` (F1 FPGA→MCU GPIO) |
| PM7 | 71† | In (WPU) | `BLADE_POWER_EN` | In | `_F2_FPGA_DONE` |

### Port N

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PN0 | 107† | Out | `CTRL_V_MGTY2_AVTT_PWR_EN` | Out | `ID_EEPROM_WP` (EEPROM write-protect) |
| PN1 | 108† | Out | `CTRL_V_MGTY1_AVTT_PWR_EN` | Out | `JTAG_FROM_SM` |
| PN2 | 109† | Out | `CTRL_V_MGTY2_AVCC_PWR_EN` | Out | `_F1_JTAG_BYPASS` |
| PN3 | 110† | Out | `CTRL_V_MGTY1_AVCC_PWR_EN` | Out | `_F2_JTAG_BYPASS` |
| PN4 | 111† | Out | `CTRL_V_MGTY2_VCCAUX_PWR_EN` | In | `SPARE_GPIO0` |
| PN5 | 112† | Out | `CTRL_V_MGTY1_VCCAUX_PWR_EN` | In | `SPARE_GPIO1` |

### Port P

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PP0 | 118† | Out | `TM4C_LED_RED` | In | `_F1_INSTALLED` (DIP switch: F1 FPGA present) |
| PP1 | 119† | In | `_V_FPGA_DONE` | In | `_F2_INSTALLED` (DIP switch: F2 FPGA present) |
| PP2 | 103† | Out | `CTRL_K_MGTH_AVTT_PWR_EN` | Out | `F1_FPGA_PROGRAM` |
| PP3 | 104† | In | `TM4C_TP2` (test point) | Out | `MCU_LED_RED` |
| PP4 | 105† | In | `TM4C_TP3` (test point) | Out | `MCU_LED_GREEN` |
| PP5 | 106† | Out | `ID_EEPROM_WP` | Out | `MCU_LED_BLUE` |

### Port Q

| Pin | MCU Pin | REV1 Dir | REV1 Signal / Function | REV2/3 Dir | REV2/3 Signal / Function |
|-----|---------|----------|------------------------|------------|--------------------------|
| PQ0 | 5† | Out | `TM4C_TO_VU7P_0` | OD | `_PWR_I2C_RESET` |
| PQ1 | 6† | In | `TM4C_FROM_VU7P_0` | OD | `_CLOCKS_I2C_RESET` |
| PQ2 | 11† | In | `TM4C_DIP_SW_2` (F2 installed?) | OD | `_F2_OPTICS_I2C_RESET` |
| PQ3 | 27† | Out | `V_FPGA_PROGRAM` | OD | `_F1_OPTICS_I2C_RESET` |
| PQ4 | 102† | Out | `CTRL_K_MGTY_AVTT_PWR_EN` | Out | `FPGA_CFG_FROM_FLASH` |
| PQ5 | 57† | OD | `_CLOCKS_I2C_RESET` | In | `PG_F2_INT_A` (F2 FPGA power-good: INT rail A) |
| PQ6 | 58† | OD | `_V_OPTICS_I2C_RESET` | In | `PG_F2_INT_B` (F2 FPGA power-good: INT rail B) |

---

## Unimplemented / Unused Peripherals

### Unused UART

| Peripheral | REV1 | REV2/3 | Pin options | Current use of those pins |
|------------|------|--------|-------------|---------------------------|
| UART0 | unused | **used** (`ZQ_UART`) | PA0 (RX), PA1 (TX) | Used by UART0 itself in REV2/3; idle in REV1 |
| UART1 | **used** (`ZQ_UART`) | unused | PB0 (RX), PB1 (TX) | Reassigned to I2C5 in REV2/3 |
| UART2 | unused | unused | PD6 (RX), PD7 (TX) **or** PG4 (RX), PG5 (TX) | PD6/PD7 → ADC (`AIN5`/`AIN4`); PG4/PG5 → I2C3 (F2 Firefly) |
| UART3 | unused | unused | PA4 (RX), PA5 (TX) | REV1: GPIO (`CTRL_V_VCCINT_PWR_EN` / `CTRL_VCC_1V8_PWR_EN`); REV2/3: muxed in `pinout_rev2.c` but peripheral never initialized |
| UART5 | unused | unused | PE4 (RX), PE5 (TX) | ADC (`AIN9` / `AIN8`) |
| UART6 | unused | unused | PD4 (RX), PD5 (TX) | ADC (`AIN7` / `AIN6`) |
| UART7 | unused | unused | PE0 (RX), PE1 (TX) | ADC (`AIN3_1` / `AIN2_1`) |

### Unused I2C

| Peripheral | REV1 | REV2/3 | Pin options | Current use of those pins |
|------------|------|--------|-------------|---------------------------|
| I2C5 | unused | **used** (FPGA monitoring) | PB0 (SCL), PB1 (SDA) | UART1 (`ZQ_UART`) in REV1 |
| I2C6 | **used** (FPGA monitoring) | unused | PA6 (SCL), PA7 (SDA) | REV1: I2C6 itself; REV2/3: **pins unassigned** — I2C6 is available if PA6/PA7 are configured |
| I2C7 | unused | unused | PA4 (SCL), PA5 (SDA) **or** PD0 (SCL), PD1 (SDA) | PA4/PA5 → UART3 (see above); PD0/PD1 → ADC (`AIN15_1` / `AIN14_1`) |
| I2C8 | unused | unused | PA2 (SCL), PA3 (SDA) **or** PD2 (SCL), PD3 (SDA) | PA2/PA3 → UART4 (used); PD2/PD3 → ADC (`AIN13_1` / `AIN12_1`) |
| I2C9 | unused | unused | PA0 (SCL), PA1 (SDA) **or** PE6 (SCL), PE7 (SDA) | PA0/PA1 → UART0 (used); PE6/PE7 → not bonded out on TM4C1290NCPDT |

With the exception of I2C6 in REV2/3 (PA6/PA7 are unassigned and available), all other unused UART and I2C peripherals have every available pin option occupied by another function in this design.

### Other Unused Peripherals

| Peripheral | Notes |
|------------|-------|
| SSI0–SSI3 (SPI) | `IntDefaultHandler` in `startup_gcc.c`; never initialized |
| PWM0 | Not used |
| CAN0 / CAN1 | Not used |
| USB | Not used |
| EPI0 | Not used |

---

*Generated from firmware source. For authoritative pin assignments refer to `common/pinout_rev1.c` and `common/pinout_rev2.c` / `common/gpio_pins_rev2.def`.*
