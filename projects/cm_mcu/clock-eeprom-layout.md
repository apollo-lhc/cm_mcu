# External Clock EEPROM Layout

The external EEPROM (I2C address `0x50`, bus 2) stores configuration programs for all five
clock synthesizers. It is divided into five equal blocks of 32 pages (256 bytes/page = 8 KB
per block).

## Block Map

| Clock | Device # | Chip (REV3)  | Block address range | Metadata page |
|-------|----------|--------------|---------------------|---------------|
| R0A   | 0        | Si5395-REVA  | `0x0000`–`0x1FFF`   | `0x1F00`      |
| R0B   | 1        | Si5395-REVA  | `0x2000`–`0x3FFF`   | `0x3F00`      |
| R1A   | 2        | Si5395-REVA  | `0x4000`–`0x5FFF`   | `0x5F00`      |
| R1B   | 3        | Si5395-REVA  | `0x6000`–`0x7FFF`   | `0x7F00`      |
| R1C   | 4        | Si5395-REVA  | `0x8000`–`0x9FFF`   | `0x9F00`      |

Block start address = `device * 0x2000`.
Metadata page = last page of the block = `(32 * (device + 1) - 1) * 0x100`.

## Register Entries

Starting at byte 0 of each block, the clock config is stored as a flat sequence of 3-byte
entries (the format produced by Silicon Labs' ClockBuilder Pro export):

```
[page (1 byte)] [register address (1 byte)] [value (1 byte)]
```

The number of entries (preamble + register + postamble sections) determines where any given
register lands within the block. For Si5395 with 587 total entries this is `587 × 3 = 1761`
bytes (`0x6E1`).

## Metadata (last page of each block)

The final page of each block stores counts used by the loader:

| Offset within page | Size    | Field            | Unprogrammed sentinel |
|--------------------|---------|------------------|-----------------------|
| `0x7C`             | 1 byte  | PreambleList_row | `0xFF`                |
| `0x7D`             | 2 bytes | RegisterList_row | `0xFFFF`              |
| `0x7F`             | 1 byte  | PostambleList_row| `0xFF`                |

Absolute EEPROM addresses for each clock's RegisterList_row (`0x_F7D`):

| Clock | Address  |
|-------|----------|
| R0A   | `0x1F7D` |
| R0B   | `0x3F7D` |
| R1A   | `0x5F7D` |
| R1B   | `0x7F7D` |
| R1C   | `0x9F7D` |

## DESIGN_ID Location (`eeprom_progname_reg`)

The DESIGN_ID is an 8-byte string (chip registers `0x026B`–`0x0272`) stored as 8 individual
3-byte entries within the register sequence. The loader extracts the value byte (3rd byte) of
each entry.

For Si5395 with 587 entries, DESIGN_ID entry #538 sits at offset `0x064E` from the block
start, giving these absolute addresses:

| Clock | Address  |
|-------|----------|
| R0A   | `0x064E` |
| R0B   | `0x264E` |
| R1A   | `0x464E` |
| R1B   | `0x664E` |
| R1C   | `0x864E` |

These values are the `eeprom_progname_reg` field in `clk_moni2c_addrs[]`
(`LocalTasks.c`). If the clock config file is regenerated with a different number of
entries, that field and the comment on the same line must be updated to match.
