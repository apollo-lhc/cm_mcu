# Data Files for `sm_cm_config`

This directory contains YAML inputs and XML artifacts used by the CM MCU/Zynq address-generation flow.

## File Overview

- `MON_I2C_rev2.yml`
  - I2C monitor register map input for CM hardware revision 2.
  - Used to generate MCU-side monitor address tables for REV2.

- `MON_I2C_rev3.yml`
  - I2C monitor register map input for CM hardware revision 3.
  - Defines monitored devices, register layout, masks, units, and data types.

- `PL_MEM_CM_rev1.yml`
  - Main PL memory-map input for CM hardware revision 1.
  - Used by generation scripts to produce MCU C tables and Zynq XML address mappings.

- `PL_MEM_CM_rev2.yml`
  - Main PL memory-map input for CM hardware revision 2.

- `PL_MEM_CM_rev2.yml~`
  - Editor backup file for `PL_MEM_CM_rev2.yml`.
  - Not used by generation scripts; safe to ignore for normal builds.

- `PL_MEM_CM_rev3.yml`
  - Main PL memory-map input for CM hardware revision 3.

- `address_apollo.xml`
  - Generated XML address-table artifact used on the Zynq side.
  - Included in the software stack that decodes CM MCU memory fields.

- `connections.xml`
  - XML file describing module connections used by the address-table hierarchy.

## Notes

- Revision-specific YAML files must stay in sync with firmware expectations for `REV1`/`REV2`/`REV3` builds.
- If a YAML file changes, regenerate and commit the corresponding generated outputs consumed by firmware and Zynq software.
- The canonical generation entry points are in `sm_cm_config/src` (see parent docs in `sm_cm_config/README.md`).
