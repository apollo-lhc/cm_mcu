#default jlink command file creation

#after calling use, JLinkExe -CommandFile "command file"
#call bootloader last (bl)

cat <<EOF > /nfs/cms/tracktrigger/logan/cm_mcu/projects/cm_mcu/gcc/jlinkload.cmd

// this is a command file for JLINK

log loadlog.txt
exitonerror 1

usb 
device TM4C1290NCPDT
speed auto
si jtag
//JTagConf 18,2 // KU + VU
JTagConf 0,0 // MCU only
//JTagConf 6,1 // KU only
connect
loadbin ./cm_mcu.bin,0x4000
exit
EOF

echo "created 'jlinkload.cmd'"

cat << EOF > /nfs/cms/tracktrigger/logan/cm_mcu/projects/cm_mcu/gcc/jlinkloadbl.cmd

// this is a command file for JLINK

log loadbllog.txt
exitonerror 1

usb 
device TM4C1290NCPDT
speed auto
si jtag
//JTagConf 18,2 // KU + VU
JTagConf 0,0 // MCU only
//JTagConf 6,1 // KU only
connect
loadbin ./bl_main.bin,0x0
exit
EOF

echo "created 'jlinkloadbl.cmd'"
