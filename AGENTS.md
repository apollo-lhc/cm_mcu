# Repository Guidelines

## Project Structure & Module Organization

This is Apollo command-module MCU firmware for the TI TM4C1290NCPDT with TivaWare and FreeRTOS. Shared code lives in `common/`; TivaWare in `inc/` and `driverlib/`; FreeRTOS in `FreeRTOS-Kernel`. Application firmware is under `projects/`: main image in `projects/cm_mcu`, bootloader in `projects/boot_loader`, and production tests in `projects/prod_test`. In `projects/cm_mcu`, tasks use `*Task.c`, commands live in `commands/`, and pin maps use `pinout_rev*.c`.

## Build, Test, and Development Commands

- `git submodule update --init --recursive`: initialize FreeRTOS before the first build.
- `make -j DEBUG=1 REV3=1`: build a Rev3 debug image with symbols.
- `make -j REV1=1`: build a Rev1 release image; release builds use warnings as errors.
- `make clean`: clean before switching between debug and non-debug builds or board revisions.
- `make COMPILER=clang REV3=1`: run a Clang syntax/static-analysis build.
- `make check-for-pr`: run formatting checks and the full build matrix.
- `make release`: build release artifacts and package XML/tarball output.

## Coding Style & Naming Conventions

C code uses `.clang-format` based on LLVM: 2-space indentation, no tabs, Stroustrup braces, unsorted includes, and no column limit. Run `make format` or `make format-apply` for changed `.c/.h` files. Prefer fixed-width integers such as `uint32_t` for registers. Keep hardware access centralized in modules such as `I2CCommunication.*`, `PowerSupplyTask.c`, and pinout files. Use explicit `REV1`, `REV2`, or `REV3` guards for board-specific behavior.

## Firmware Constraints

This is bare-metal FreeRTOS firmware. Avoid heap allocation unless an existing pattern justifies it, and use queues, mutexes, or semaphores for cross-task ownership. In `cm_mcu`, task-context EEPROM writes must use `write_eeprom()` and related gatekeeper APIs. CLI handlers use `BaseType_t command(int argc, char **argv, char *m)`; write with `snprintf`, register in `CommandLineTask.c`, and update help text. UART output should use `\r\n`.

## Testing Guidelines

Validation is build and analysis based. Before a PR, run the target build and prefer `make check-for-pr` for shared or revision-sensitive changes. CI runs GCC, Clang, CodeQL, and clang-format.

## Generated Sources & Configuration

Address-table files such as `ZynqMon_addresses.*` and `MonI2C_addresses.*` are generated from YAML under `sm_cm_config/data` by Python scripts in `sm_cm_config/src`. When changing YAML, let `make` regenerate outputs and commit both YAML and generated files.

## Commit & Pull Request Guidelines

Recent commits use short imperative summaries with optional PR numbers, for example `Fix stale temperature alarms and add hysteresis deadband (#282)`. Keep commits focused. PRs should describe behavior changes, affected revisions, validation commands, and generated-file updates. Link issues when available.
