import yaml from pprint import pprint
#import dicttoxml
#there is also dict2xml available in pypy(python3)

    with open('../data/zynqmon_2.yml') as f:data = yaml.load(f, Loader = yaml.FullLoader)

                                                                    config = data['config'] for c in config:print(c['name'])

                                                                        size = 0 for c in config:
#generate C call
                                                             expected_length = len(c['names']) if 'postfixes' in c:expected_length *= len(c['postfixes']) if '32' in c['type'] :expected_length *= 2 elif 'char' in c['type'] :expected_length = expected_length / 2. if 'size' in c:expected_length *= c['size'] * 4 #size in 4 byte words -- extra factor of 4
#if (c['count'] != expected_length) :
#print(f "// mismatch:  {expected_length}, size {c['count']}")
                                                             print(f "// {c['name']}, size {expected_length}")
                                                                 print(f "zm_set_{c['mcu_call']}(&zynqmon_data[{size}], {c['start']});")
                                                                     size += int(expected_length)
                                                                         print(f "#define ZMON_VALID_ENTRIES {size}")
