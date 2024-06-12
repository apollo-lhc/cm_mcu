#! /usr/bin/env python
# -*- coding: utf-8 -*-
"""generate the C source file for the MCU Mon I2C addresses and initialization calls"""

import os
import sys
import argparse
import datetime
import subprocess
import io
from string import Template

import yaml

def parse_args() -> argparse.Namespace:
    """Parse the command line arguments"""
    parser = argparse.ArgumentParser(description='Process YAML for MCU Mon.')
    parser.add_argument('-v', '--verbose', action='store_true',
                    help='increase output verbosity')
    parser.add_argument('-o', '--output', type=str, help='output file name',
                    default="MonI2C_addresses.c")
    # this argument is required
    parser.add_argument('input_file', metavar='file', type=str,
                    help='input yaml file names')

    return parser.parse_args()

def write_boilderplate(fout: io.TextIOWrapper):
    """Write the boilerplate to the output file"""
    print(f"// This file is generated by {os.path.basename(sys.argv[0])}", file=fout)
    print(r"// Do not edit this file directly", file=fout)
    print(r"// Edit the yaml files in the data directory and run the script again", file=fout)
    print(r"// to re-generate this file", file=fout)
    print(r"//", file=fout)
    print(r"// This file contains the arrays of the i2c monitoring data", file=fout)
    print(r"// and the C calls to set and retrieve it", file=fout)
    # Print timestamp to output file
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f"// Generated: {timestamp}", file=fout)
    print(r"//", file=fout)

def int_to_list(ndev_types, prefix, var_list: list) -> str:
    """Convert an integer to a list of integers of length ndev_types"""
    if isinstance(var_list, int):
        var_list = [var_list]*int(ndev_types)
    var_list_str = '{'
    for r in var_list:
        if r >= 0:
            var_list_str += f"{r}, "
        else:
            var_list_str += f"{prefix}_NOT_COVERED, "
    var_list_str += "}"
    return var_list_str

# main program
def main():
    """generate the C source file for the MCU Mon I2C addresses and initialization calls"""
    #######################################
    #
    # parse the command line arguments
    #
    #######################################
    args = parse_args()

    if args.output and args.verbose:
        print('Output file name:', args.output)

    addr_template = Template("{$reg_size, $page, $reg_list, $size, \"$name\","
                            " $mask, \"$units\", $type, get_${prefix}_${name}_mask,"
                            " set_${prefix}_${name}_data, get_${prefix}_${name}_data},")
    # output file names: header file and c source file
    # make sure that the output file ends with .c. Print error message and exit if it doesn't
    if not args.output.endswith(".c"):
        print(f"Output file name {args.output} does not end with .c")
        sys.exit(1)
    source_fname = args.output
    header_fname = args.output[:-2] + '.h'

    #######################################
    #
    # end of processing command line arguments
    #
    #######################################

    # open output c source file for writing
    with open(source_fname, 'w', encoding="ascii") as fout_source, \
            open(header_fname, 'w', encoding="ascii") as fout_header:
        write_boilderplate(fout_source)
        write_boilderplate(fout_header)
        print(f"#include \"{header_fname}\"", file=fout_source)

        # header file
        print(r"#ifndef MON_I2C_ADDRESSES_H", file=fout_header)
        print(r"#define MON_I2C_ADDRESSES_H", file=fout_header)
        print("#include <stdint.h>", file=fout_header)
        print("#include \"MonitorTaskI2C.h\"", file=fout_header)
        print("#include \"FireflyUtils.h\"", file=fout_source)

        with open(args.input_file, encoding="ascii") as f:

            # generate the list of registers to access
            # loop over devices first
            data = yaml.load(f, Loader=yaml.FullLoader)

            for d in data['devices']:
                ndev = d['ndevices']
                ndev_types = d['ndevice_types']
                prefix = d['prefix']
                config = d['config']
                ncommands = len(config)
                print(f"#define NCOMMANDS_{prefix} {ncommands}", file=fout_header)
                print(f"// {prefix} has {ndev} devices and {ncommands} commands", file=fout_source)
                print(f"#define {prefix}_NOT_COVERED (-1)", file=fout_source)
                print(f"struct i2c_reg_command_t sm_command_test_{prefix}[NCOMMANDS_{prefix}] = {{",
                    file=fout_source)
                print(f"extern struct i2c_reg_command_t sm_command_test_{prefix}[NCOMMANDS_{prefix}];",
                    file=fout_header)
                for c in config:
                    reg_list = c['reg_address']
                    #print(f"reg list is >{reg_list}<")
                    # if reg_list is an integer, convert it to a list of ndev_types integers
                    # this handles the case where all ndev types share the same address
                    reg_list_str = int_to_list(ndev_types, prefix, reg_list)
                    # ditto for the page
                    page_list = c['page']
                    page_list_str = int_to_list(ndev_types, prefix, page_list)
                    s = addr_template.substitute(c, reg_list=reg_list_str, prefix=prefix, 
                                                 page=page_list_str)
                    print(s, file=fout_source)
                print(r"};", file=fout_source)

                # generate the arrays to store the data
                print(r"// Arrays to store the data", file=fout_source)
                prefix = d['prefix']
                for c in config:
                    data_name = f"{prefix}_{c['name']}_data"
                    print(f"static uint16_t {data_name}[{ndev}] = {{0}};", file=fout_source)
                # generate access functions
                print(r"// Access functions", file=fout_source)
                for c in config:
                    data_name = f"{prefix}_{c['name']}_data"
                    set_fcn_name = f"set_{prefix}_{c['name']}_data"
                    get_fcn_name = f"get_{prefix}_{c['name']}_data"
                    # getter
                    print(f"uint16_t {get_fcn_name}(int which);", file=fout_header)
                    print(f"uint16_t {get_fcn_name}(int which) {{", file=fout_source)
                    print(f"    return {data_name}[which];", file=fout_source)
                    print(r"}", file=fout_source)
                    # setter
                    print(f"void {set_fcn_name}(uint16_t data, int which);", file=fout_header)
                    print(f"void {set_fcn_name}(uint16_t data, int which) {{", file=fout_source)
                    print(f"        {data_name}[which] = data;", file=fout_source)
                    print(r"}", file=fout_source)
                # using the devices field, generate a function that returns a mask of the devices
                print(r"// Function to return a mask of the devices for each command", 
                      file=fout_source)
                for c in config:
                    get_mask_fcn_name = f"get_{prefix}_{c['name']}_mask"
                    print(f"uint16_t {get_mask_fcn_name}(void);", file=fout_header)
                    print(f"uint16_t {get_mask_fcn_name}(void) {{", file=fout_source)
                    print(r"    return ", end="", file=fout_source)
                    for d in c['devicetypes']:
                        print(f"DEVICE_{d} | ", end="", file=fout_source)
                    print(r"0;", file=fout_source)
                    print(r"}", file=fout_source)
            # special: for the firefly devices, generate a list of functions that
            # return either the data in the F1 array or the data in the F2 array,
            # depending on if the argument is >= than or less than NFIREFLIES_F1
            print(r"// Functions to return the data in the F1 or F2 arrays", file=fout_source)
            config = data['define'] #['FF_SHARED_REGS']
            #print(config)
            for c in config:
                data_name_f1 = f"FF_F1_{c['name']}_data"
                data_name_f2 = f"FF_F2_{c['name']}_data"
                get_fcn_name_f1 = f"get_{data_name_f1}"
                get_fcn_name_f2 = f"get_{data_name_f2}"
                get_fcn_name = f"get_FF_{c['name']}_data"
                # write a preprocessor macro that uses a trigram to select the correct function
                print(f"#define {get_fcn_name}(which) \\",
                        file=fout_header)
                print(f"    ((which) < NFIREFLIES_F1 ? {get_fcn_name_f1}((which)) : {get_fcn_name_f2}((which) - NFIREFLIES_F1))",
                        file=fout_header)

            # closing header guard
            print(r"#endif// MON_I2C_ADDRESSES_H", file=fout_header)

    # reformat the c file using clang-format
    # -style=file:$HOME/src/apollo_cm_mcu/.clang-format
    # if the clang-format fails, we just ignore it
    try:
        r = subprocess.run(["clang-format", "-i", source_fname, header_fname], check=False)
        if r.returncode != 0 and args.verbose:
            print('clang-format failed')
        if args.verbose:
            print('clang-format complete')
    except FileNotFoundError as e:
        if args.verbose:
            print(f"clang-format not found: {e}")

if __name__ == '__main__':
    main()
