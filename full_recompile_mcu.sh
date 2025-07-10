#!/bin/bash

# Go to the cm_mcu directory (assumes the script is in the cm_mcu directory)
cd "$(dirname "$0")" || { echo "Failed to enter cm_mcu directory"; exit 1; }

# Step 1: Clean build
echo "=== Running make clean ==="
make clean || { echo "make clean failed"; exit 1; }

# Step 2: Build
echo "=== Running make -j DEBUG=1 VERBOSE=1 REV3=1 ==="
make -j DEBUG=1 VERBOSE=1 REV3=1 || { echo "Build failed"; exit 1; }

# Step 3: Run jlink_prep
echo "=== Running ./jlink_prep ==="
./jlink_prep || { echo "jlink_prep failed"; exit 1; }

# Step 4: Go to gcc directory
cd projects/cm_mcu/gcc || { echo "Failed to enter gcc directory"; exit 1; }

# Step 5: Flash main program
echo "=== Flashing main firmware (cm_mcu.bin) ==="
JLinkExe -CommandFile jlinkload.cmd || { echo "Flashing cm_mcu.bin failed (this is as far as it ever gets)"; exit 1; }

# Step 6: Flash bootloader
echo "=== Flashing bootloader (bl_main.bin) ==="
JLinkExe -CommandFile jlinkloadbl.cmd || { echo "Flashing bl_main.bin failed"; exit 1; }

echo "=== Done ==="
