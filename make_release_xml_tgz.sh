#! /bin/bash

# exit on error
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command failed with exit code $?."' ERR

# choose which release this build is for
REV="2" # default
if [ "${REV1}" == "1" ]; then
    REV="1"
elif [ "${REV3}" == "1" ]; then
    REV="3"
fi

[ -d tmp ] && rm -rf tmp

mkdir -p tmp/CornellCM_MCU/address_table/modules_CM_MCU
cd tmp
cp ../PL_MEM*.xml CornellCM_MCU/address_table/modules_CM_MCU
cp ../sm_cm_config/data/*.xml CornellCM_MCU/address_table
cd CornellCM_MCU/address_table/modules_CM_MCU
ln -s PL_MEM_CM_rev${REV}.xml PL_MEM_CM.xml
cd ../../..
tar -czf ../CornellCM_MCU.tar.gz CornellCM_MCU
cd ..
rm -rf tmp
