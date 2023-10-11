""" Generate the C code for the microcontroller using the yaml files in the data directory"""
import os
import sys
import argparse
import yaml

parser = argparse.ArgumentParser(description='Process YAML for MCU.')
parser.add_argument('-v', '--verbose', action='store_true',
                    help='increase output verbosity')
parser.add_argument('-o', '--output', type=str, help='output file name',
                    default="ZynqMon_addresses.c")
# this argument is required
parser.add_argument('input_files', metavar='file', type=str,
                    nargs='+', help='input yaml file names')

args = parser.parse_args()

if args.output:
    print('Output file name:', args.output)

if args.verbose:
    print('Input file names:', args.input_files)

# open output text file for writing
with open(args.output, 'w', encoding="ascii") as fout:
    print(r"// This file is generated by mcu_generate.py", file=fout)
    print(r"// Do not edit this file directly", file=fout)
    print(r"// Edit the yaml files in the data directory and run mcu_generate.py", file=fout)
    print(r"// to generate this file", file=fout)
    print(r"//", file=fout)
    print(r"// This file contains the addresses of the zynqmon data", file=fout)
    print(r"// and the C calls to initialize the zynqmon data", file=fout)
    print(r"//", file=fout)
    print("#include \"Tasks.h\"", file=fout)

    # first set the #ifdef for REV1
    print(r"#ifdef REV1", file=fout)
    for idx, fname in enumerate(args.input_files):
        with open(fname, encoding="ascii") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        if idx != 0:
            ver = idx + 1
            print(f"#elif defined(REV{ver}) // REV{ver}", file=fout)
        print("void zm_fill_structs(void) \n{", file=fout)

        # print the names of the variables
        if args.verbose:
            print(f"dump variables for iteration {idx}")
        config = data['config']
        for c in config:
            if args.verbose:
                print(c['name'])

        pairs = []
        size = 0
        for c in config:
            # generate C call
            expected_length = len(c['names'])
            if 'postfixes' in c:
                expected_length *= len(c['postfixes'])
            if '32' in c['type']:
                expected_length *= 2
            elif 'char' in c['type']:
                expected_length = expected_length/2.
            if 'size' in c:
                expected_length *= c['size']*4 # size in 4 byte words -- extra factor of 4
            if c['count'] != expected_length:
                print(f"// mismatch:  {expected_length}, size {c['count']}", file=fout)
                print(f"Mismatch in size for {c['name']}, file {fname}")
                print(f"Mismatch: expected: {expected_length}, size {c['count']}")
                # close and delete the output file before exiting
                fout.close()
                os.remove(args.output)
                sys.exit(1)
            print(f"// {c['name']}, size {expected_length}", file=fout)
            if c['mcu_extra_call'] is None:
                print(f"zm_set_{c['mcu_call']}(&zynqmon_data[{size}], {c['start']});", file=fout)
            else :
                print(f"zm_set_{c['mcu_call']}(&zynqmon_data[{size}], {c['start']}, "
                      f"{c['mcu_extra_call']});", file=fout)
            size += int(expected_length)
            pairs.append((c['start'], c['start'] + int(expected_length)-1))

        # check to ensure that none of the tuples in pairs overlap
        # this is a sanity check to ensure that the yaml file is correct
        errors=False
        for a, b in zip(pairs, pairs[1:]):
            if a[0] <= b[0] and a[1] >= b[0] or a[0] <= b[1] and a[1] >= b[1]:
                print(f"ERROR: overlap: {a} and {b}")
                errors=True
        if errors:
            print("ERRORS FOUND")
            sys.exit(1)
        # close input yaml file
        f.close()
        print("}", file=fout)
        print(f"#define ZMON_VALID_ENTRIES {size}", file=fout)
    # close output text file
    print(r"#else // REV1", file=fout)
    # error if no revision is defined
    print(r"#error No revision defined", file=fout)
    print(r"#endif // REV1", file=fout)
    fout.close()
