# sm_cm_config
Configuration file and code to generate the memory mapping between the MCU and the Zynq. Here we generate the C code used on the MCU which sends the data and the XML files used on the Zynq to interpret the data.

On the MCU side, we define which addresses the MCU will send with the data. (python code `mcu_generate.py`)

On the Zynq side, the XML files allow us to decode the memory on the zynq according to the data as laid out in the MCU. (python code `xml_generate.py`)

The input files are in the `data` directory and are given by `PL_MEM_CM_rev<i>.yml` where `i` refers to the CM revision number.
There are two outputs:
1. The `C` code consists of a `c` and a header file. It is compiled into the MCU binary in the `cm_mcu` repo
1. The xml files must be included in the Zynq in the `/fw/CM/CornellCM_MCU/address_table/modules_CM_MCU` directory. Note that the `xml_generate` script creates a `PM_MEM_CM_rev<i>.xml` file but the file that is included is just called `PL_MEM_CM.xml`.
