#! /bin/sh

# exit on error
set -e

# keep track of the last executed command
trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
# echo an error message before exiting
trap 'echo "\"${last_command}\" command failed with exit code $?."' ERR


[ -d tmp ] && rm -rf tmp

mkdir -p tmp/CornellCM_MCU/address_table/modules_CM_MCU
cd tmp
cp ../PL_MEM*.xml CornellCM_MCU/address_table/modules_CM_MCU
cp ../sm_cm_config/data/*.xml CornellCM_MCU/address_table
cd CornellCM_MCU/address_table/modules_CM_MCU
ln -s PL_MEM_CM_Rev2.xml PL_MEM_CM.xml
cd ../../..
tar -czf ../CornellCM_MCU.tar.gz CornellCM_MCU
cd ..
rm -rf tmp
