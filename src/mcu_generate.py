import yaml
#import dicttoxml
# there is also dict2xml available in pypy (python3)

# generate two file names with prefix zynqmon_ and suffixes 1 and 2
files=['../data/zynqmon_1.yml', '../data/zynqmon_2.yml']

# open output text file named "zynqmon_addresses.c" for writing
with open('../src/ZynqMon_addresses.c', 'w') as fout:
    print(r"// This file is generated by mcu_generate.py", file=fout)
    print(r"// Do not edit this file directly", file=fout)
    print(r"// Edit the yaml files in the data directory and run mcu_generate.py", file=fout)
    print(r"// to generate this file", file=fout)
    print(r"//", file=fout)
    print(r"// This file contains the addresses of the zynqmon data", file=fout)
    print(r"// and the C calls to initialize the zynqmon data", file=fout)
    print(r"//", file=fout)
    print(r"#include \"Tasks.h\"", file=fout)

    # first set the #ifdef for REV1
    print(f"#ifdef REV1", file=fout)
    for idx, fname in enumerate(files):
        with open(fname) as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        if (idx != 0):
            ver = idx + 1
            print(f"#elif defined(REV{ver}) // REV{ver}", file=fout)
        print("void zm_fill_structs(void) \n{", file=fout)

        # print the names of the variables
        print(f"dump variables for iteration {idx}")
        config = data['config']
        for c in config:
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
            if ( c['count'] != expected_length ) :
                 print(f"// mismatch:  {expected_length}, size {c['count']}", file=fout)
            print(f"// {c['name']}, size {expected_length}", file=fout)
            if c['mcu_extra_call'] == None : 
                 print(f"zm_set_{c['mcu_call']}(&zynqmon_data[{size}], {c['start']});", file=fout)
            else :
                 print(f"zm_set_{c['mcu_call']}(&zynqmon_data[{size}], {c['start']}, {c['mcu_extra_call']});", file=fout)
            size += int(expected_length)
            pairs.append((c['start'], c['start'] + int(expected_length)-1))

        # check to ensure that none of the tuples in pairs overlap
        # this is a sanity check to ensure that the yaml file is correct
        errors=False
        for i in range(len(pairs)):
            for j in range(i+1, len(pairs)):
                if ( pairs[i][0] <= pairs[j][0] and pairs[i][1] >= pairs[j][0] ) or \
                   ( pairs[i][0] <= pairs[j][1] and pairs[i][1] >= pairs[j][1] ) :
                    print(f"ERROR: overlap: {pairs[i]} and {pairs[j]}")
                    errors=True
        if (errors):
            print("ERRORS FOUND")
            exit(1)
        # close input yaml file
        f.close()
        print("}", file=fout)
        print(r"#define ZMON_VALID_ENTRIES {size}", file=fout)
    # close output text file
    print(r"#else // REV1", file=fout)
    # error if no revision is defined
    print(r"#error No revision defined", file=fout)
    print(r"#endif // REV1", file=fout)
    fout.close()

# reformat the c file using clang-format
import subprocess
# -style=file:$HOME/src/apollo_cm_mcu/.clang-format
subprocess.run(["clang-format", "-i", "../src/zynqmon_addresses.c"])
