# MCU bug: four-channel FireFly alarm address and read width

Date: 2026-09-12

Status: confirmed against the supplied Samtec manual; MCU fix not implemented.
This report documents source inspection and previously captured hardware data.
No MCU rebuild, deployment, FPGA reconfiguration, or new fault stimulus was
performed to produce this report.

## Scope and authoritative reference

Affected device type: `DEVICE_25G4`, the four-channel 25G FireFly. Do not apply
its register map to the separate 12-channel FireFly devices.

Reference: *FireFly Manual ECUO 25G_28G x4 Revision_1.0.pdf*, supplied locally at
`/Users/wittich/Downloads/docs/Firefly Manual ECUO 25G_28G x4 Revision_1.0.pdf`.
The manual is marked Samtec Confidential; it is not copied into this repository.

Pages 52–54, Tables 27, 29, and 31 specify:

| Function | Decimal address | Hex address | Width |
| --- | ---: | ---: | ---: |
| LOS alarms | 3 | 0x03 | 1 byte |
| CDR loss-of-lock alarms | 5 | 0x05 | 1 byte |
| Temperature alarms | 6 | 0x06 | 1 byte |
| Supply-voltage alarms | 7 | 0x07 | 1 byte |
| Reserved | 8 | 0x08 | — |

For both LOS and CDR LOL, bits 0–3 correspond to RX channels 1–4 and bits
4–7 to TX channels 1–4. These are bits, not separate bytes. Set bits indicate
alarms. Flags latch until read or reset, and may immediately reassert while the
underlying condition persists. These lower-memory addresses are always directly
addressable, independent of upper-page selection.

For related fault-control context, Table 27 and Table 35 (pages 52 and 55)
confirm TX disable at decimal byte 86 (0x56), with bits 0–3 disabling TX
channels 1–4 when set. This report does not authorize an output-disable test.

## Source findings

MCU source root: `/Users/wittich/src/apollo_cm_mcu`
Line references below describe the inspected working tree and may move.

- `projects/cm_mcu/MonitorTaskI2C.h:59` defines `DEVICE_25G4` as `0x04`;
  the monitor's device-type selection uses table index 2 for this type.
- In `projects/cm_mcu/MonI2C_addresses.c`, the rev3 F1 `LOS_ALARM` entry
  around lines 2239–2245 specifies addresses `{7, 7, 7, 7}` and a shared
  read size of 2. Thus the four-channel case reads bytes 7–8 instead of byte 3.
- The F1 `CDR_LOL_ALARM` entry around lines 2343–2350 selects address 5
  for the four-channel case and address 20 for the 25G twelve-channel case,
  but uses a shared read size of 2. The four-channel read therefore also
  consumes byte 6, the temperature-alarm register.
- `projects/cm_mcu/MonitorTaskI2C.c:179` passes the selected per-device
  command address and the shared command size to the I2C read operation.
- `projects/cm_mcu/commands/FireflyCommands.c` displays stored monitor data:
  LOS through `get_FF_LOS_ALARM_data` near line 755 and CDR LOL through its
  custom row function near line 791. These CLI displays are not themselves
  fresh module reads. The background monitor performs the reads.

The twelve-channel CDR address was explicitly distinguished during inspection;
this finding is not based on accidentally applying a twelve-channel table to
the four-channel device.

## Impact and observed evidence

The incorrect LOS read can hide real LOS alarms or present supply alarms as LOS.
The oversized CDR LOL read can consume/clear a temperature alarm as a side
effect, and introduces an unrelated byte into the alarm value passed through
the monitor. Exact displayed packing should be covered by regression tests.

Previously captured F1 four-channel device reads showed:

| Read | Observed bytes |
| --- | --- |
| Actual LOS, byte 3 | FF |
| Actual CDR LOL, byte 5 | FF |
| Temperature alarms, byte 6 | 00 |
| MCU's LOS source, bytes 7–8 | 00 00 |

This is consistent with the operator seeing `ff_los` report zero while direct
byte-3 reads report FF. The observed CDR LOL FF is consistent with the correct
starting address, despite the excessive read width.

The repeated direct LOS/CDR LOL FF/FF readings remain an independent unresolved
hardware/diagnostic observation. Fixing the MCU mapping does not establish that
those alarms are false, clear them permanently, or qualify the optical links.
Multiple readers can also consume latched flags; comparisons must account for
read-to-clear behavior rather than require identical unsynchronized samples.

## Proposed correction

1. Select address 3 and a one-byte read for four-channel LOS.
2. Keep address 5 but select a one-byte read for four-channel CDR LOL.
3. Make read width device-specific, or introduce equivalent explicit handling.
   Do not globally change the shared size to 1: twelve-channel devices have
   distinct maps and may require multi-byte reads.
4. Review and correct the corresponding F2 entries and any generated-table
   source, so regeneration cannot reintroduce the bug. Preserve other device
   types and board revisions unless separately verified against their manuals.
5. Zero-extend the four-channel alarm byte into existing stored/displayed
   values without changing the public monitor/CLI ABI. Keep temperature
   monitoring independent of CDR LOL polling.

The separate `cm_interface/registers/firefly4.json` map also needs a follow-up
review against this manual; it must not be used as authority for this MCU fix.

## Validation before closing

- With mocked I2C transactions, verify that both FPGA sites request exactly
  `(address=3, size=1)` for four-channel LOS and `(address=5, size=1)` for
  four-channel CDR LOL.
- Use distinct nonzero sentinel values at bytes 3, 5, 6, and 7 to verify
  decoding and zero-extension, and prove CDR polling does not read byte 6.
- Check RX/TX bit interpretation, including 0x01, 0x10, and 0xFF.
- Regress twelve-channel paths and other supported device types without
  assuming their register addresses or widths match the four-channel device.
- After an explicitly authorized MCU deployment, compare monitor/CLI data with
  controlled direct reads from the same physical module. Coordinate readers
  and polling to account for clear-on-read flags and reassertion latency.
- Preserve the existing FF/FF evidence. Do not silently reset, reconfigure,
  retry failed hardware runs, or introduce optical fault stimuli as part of
  validating this mapping correction.
