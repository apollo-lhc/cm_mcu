#!/usr/bin/env python
"""Generate XML file from YAML input"""
import xml.etree.ElementTree as ET
from pprint import pprint
import argparse
import os
import yaml

zm_num_entries = 1024

#% %
def make_node(parent: ET.Element, myid: str, thedict: dict, addr2: int, bit: int, 
              parent_id: str) -> ET.Element:
    """create the node to be inserted into the xml tree"""
#pylint: disable=too-many-branches
#I disable this check because as far as I can tell it's wrong
    thenode = ET.SubElement(parent, 'node')
    myid = myid.replace(' ', '_')
    thenode.set('id', myid)
#address is half of the sensor address since these are 32 bit addresses
    theaddr = int(addr2/2)
    remain = bit
    thenode.set('address', str(hex(theaddr)))
#this appears to be on all the nodes
    thenode.set("permission", "r")
#set parameter based on the type
    if thedict['type'] == 'int8':
        width = 8
        theformat = "d"
    elif thedict['type'] == 'int16':
        width = 16
        theformat = "d"
    elif thedict['type'] == 'fp16':
        width = 16
        theformat = "fp16"
    elif thedict['type'] == 'char':
        thenode.set('mode', "incremental")
        theformat = "c"
    elif thedict['type'] == 'uint32_t':
        width = 32
        theformat = "u"
    elif thedict['type'] == 'uint16_t':
        width = 16
        theformat = "u"
    else:
        print("ERROR: unknown type", thedict['type'])
        return None
    if 'size' in thedict:  # if there is a size, it cannot have a mask
        thenode.set('size', str(hex(thedict['size'])))
    else:
        mask = (1 << width) - 1
        if remain == 0:
            thenode.set('mask', "0x{0:08X}".format(mask))
        else:
            thenode.set('mask', "0x{0:08X}".format(mask << 16))
    if 'extra' in thedict:
        extra = thedict['extra']
        if not "Column" in extra:
#this semicolon asks for no semicolon in extra when postfixes are included
            if 'default_col' in thedict and thedict['default_col'] is not None:
                colid = thedict['default_col']
            else:
                colid = myid
#for some special patterns, generate a different column id
            if myid.lower().startswith("cur_"):
                colid = "IOUT"
            elif "temp" in myid.lower():
                colid = "TEMP_C"
            extra = extra + ";Column=" + colid
        if not "Row" in extra:
            if parent_id != "":
                extra = "Row=" + parent_id + ";" + extra
            else:
                extra = "Row=" + myid + ";" + extra
    else:
        extra = ""
    thenode.set('parameters', "Format=" + theformat + ";" + extra)
    return thenode

def make_bitfields(parent: ET.Element, valnode: ET.Element, thedict: dict, fields: dict,
                   bit: int, row: str = "", extra: str = ""):
    """add a node per bit field of a value, at the address of the value's node.
    fields maps a field name to its mask within the value; the field is decoded
    by the mask, so it is shown shifted down to bit 0.
    Without extra, each field is a row in the value's table and column.
    With extra (Table=...), each field is a column of that table, in row 'row'."""
    if thedict['type'] == 'uint16_t':
        width = 16
    elif thedict['type'] == 'uint32_t':
        width = 32
    else:
        print("ERROR: bitfields need type uint16_t or uint32_t, not", thedict['type'])
        return
    shift = 16 if bit == 1 else 0
    # same Table, Status and Column as the value; own Row; unsigned
    params = [p for p in valnode.get('parameters').split(';')
              if not p.startswith('Format=') and not p.startswith('Row=')]
    for fname, fmask in fields.items():
        if not isinstance(fname, str):  # e.g. a bare OFF/ON/NO in YAML reads as a bool
            print("ERROR: bit field name", fname, "is not a string; quote it, in",
                  valnode.get('id'))
            continue
        if not isinstance(fmask, int) or fmask <= 0 or fmask >= (1 << width):
            print("ERROR: bit field", fname, "has a mask that does not fit in",
                  valnode.get('id'))
            continue
        node = ET.SubElement(parent, 'node')
        node.set('id', valnode.get('id') + "_" + fname if extra else fname)
        node.set('address', valnode.get('address'))
        node.set('permission', 'r')
        node.set('mask', "0x{0:08X}".format(fmask << shift))
        if extra:
            node.set('parameters', "Format=u;Row=" + row + ";Column=" + fname + ";" + extra)
        else:
            node.set('parameters', ";".join(["Format=u", "Row=" + fname] + params))


def calc_size(thedict: dict) -> int:
    """Calculate size based on type"""
    if 'size' in thedict:
        sz = thedict['size']*2  # extra factor of 2 for 16 to 32 bit
    else:
        sz = 1
    if '32' in thedict['type']:
        sz = sz * 2
    elif 'char' in thedict['type']:
        sz = sz // 2
    return sz

#% %
#check overlaps
#create an object with a name, and a start end end register
#do so for every entry
#then ensure that no two objects overlap

#create a class to hold a name, start, end, and size
#add a method to check if two objects overlap
#and a pretty print method
class reg:
    """create an object with a name, and a start end end register"""
    def __init__(self, name, sta, end, sz, width):
        self.name = name
        self.start = sta
        self.end = end
        self.size = sz
        self.width = width

    def __str__(self):
        return "name: " + self.name + " start: " + str(self.start) + \
            " end: " + str(self.end) + " size: " + str(self.size)

    def overlaps(self, other):
        """calculate overlap between two objects"""
        if (self.start <= other.start and self.end >= other.start):
            return True
        if (self.start <= other.end and self.end >= other.end):
            return True
        if (self.start >= other.start and self.end <= other.end):
            return True
        return False

    def overloads(self):
        """check if the object overloads the register space"""
        if self.start + self.size >= zm_num_entries - 1:
            return True
        return False

#custom file type for yaml file, to be used with argparse
def yaml_file(filename):
    """custom file type for yaml file, to be used with argparse"""
    if not filename.endswith('.yml'):
        raise argparse.ArgumentTypeError('File must have a .yml extension')
    return filename

parser = argparse.ArgumentParser(description='Process YAML for XML.')
parser.add_argument('-v', '--verbose', action='store_true',
                    help='increase output verbosity')
parser.add_argument('-d', '--directory', type=str, help='output directory')
#this argument is required, one input file ending with yaml extension
parser.add_argument('input_file', metavar='file', type=yaml_file,
                    help='input yaml file name')

args = parser.parse_args()

if args.verbose:
    print('Verbose mode on')
    print('Input file names:', args.input_file)
    if args.directory:
        print('Output directory:', args.directory)


with open(args.input_file, encoding='ascii') as f:
    y = yaml.load(f, Loader=yaml.FullLoader)
    if args.verbose:
        pprint(y)

#% %
#This is the parent(root) tag
#onto which other tags would be
#created
cm = ET.Element('node')
cm.set('id', 'CM')
cm.set('address', '0x00000000')
#keep track of the most recent address that comes into a pair of bytes for 8 - bit masking
prev_addr = 0x0
prev_j = 0x0 # keep track of the order of postfixes in each name node
prev_bit = 0x0 # keep track of the even or odd order of bytes globally sent for masking
#% %
config = y['config']
for c in config:  # loop over entries in configuration (sensor category)
    i = 0  # counter over the number of sensors within a category
    names = c['names']
    start = c['start']
    count = c['count']
    # bitfields are keyed by name, or by postfix in a block with postfixes; the
    # latter go to their own table, given by bitfield_extra
    bitfields = c.get('bitfields') or {}
    bitfield_extra = c.get('bitfield_extra') or ""
    keys = c['postfixes'] if 'postfixes' in c else names
    for b in bitfields:
        if b not in keys:
            print("ERROR: bitfields for", b, "which is not a name or postfix in", c['name'])
    if bitfields and 'postfixes' in c and not bitfield_extra:
        print("ERROR: bitfields on postfixes need bitfield_extra, in", c['name'])
    # postfix_types: postfix -> type, for one postfix whose type differs from the block's
    postfix_types = c.get('postfix_types') or {}
    for p, t in postfix_types.items():
        if p not in c.get('postfixes', []) or calc_size(dict(c, type=t)) != calc_size(c):
            print("ERROR: postfix_types", p, t, "must be a postfix of the same size, in", c['name'])
    for n in names:  # loop over names of sensors within a category
        if 'postfixes' in c:
            postfixes = c['postfixes']
            j = 0
            for p in postfixes:
                addr = int((start + i)/2)
                bit = i%2
                if p == 'RESERVED':
                    i += 1
                    j += 1
                    continue
                cp = dict(c, type=postfix_types[p]) if p in postfix_types else c
                # the previous name node has odd bytes so this postfix node uses the
                # previous postfix address but masks off the lower byte
                if (bit == 1 and j == 0):   
                    pp = node = ET.SubElement(cm, 'node')
                    pp.set('id', n)
                    pp.set('address', str(hex(prev_addr)))
                    node = make_node(pp, p, cp, j, bit, n)
                elif (bit == 0 and j == 0): #starting a new postfix node in a new name node
                    pp = node = ET.SubElement(cm, 'node')
                    pp.set('id', n)
                    pp.set('address', str(hex(addr)))
                    node = make_node(pp, p, cp, j, bit, n)
                else: # any non-first byte in a name node
                    if prev_bit == 0: #the upper byte of the previous postfix node
                        node = make_node(pp, p, cp, j, bit, n)
                    else :               #the low byte with an increasing postfix node by one
                        node = make_node(pp, p, cp, j+1, bit, n)
                if p in bitfields and node is not None:
                    make_bitfields(pp, node, cp, bitfields[p], bit, n, bitfield_extra)
                if (prev_bit == bit and prev_addr == addr and prev_addr != 0) :
                    print("warning : please check if masks overlapped at node ", 
                          n, " addr ", hex(prev_addr))
                prev_addr = addr
                prev_j = j
                prev_bit = bit
                i += 1
                j += 1
        else:
            valnode = make_node(cm, n, c, start+i, (start+i)%2, "")
            if n in bitfields and valnode is not None:
                make_bitfields(cm, valnode, c, bitfields[n], (start+i)%2)
            if (prev_bit == (start+i)%2 and prev_addr == int((start+i)/2) and prev_addr != 0) :
                print("warning : please check if masks overlapped at node ", n, 
                      " addr ", hex(prev_addr))
            prev_addr = int((start + i)/2)
            prev_bit = (start+i)%2
            i += 1
        prev_start = start
        prev_count = count

tree = ET.ElementTree(cm)
ET.indent(tree, space='\t')
#create output file name based on input file, replacing 'yml' with 'xml'
out_name = os.path.basename(args.input_file)[:-len('.yml')] + '.xml'
out_name = args.directory + '/' + out_name
if args.verbose:
    print("writing to file", out_name)
tree.write(out_name)

#% %

#create a list of objects
entries = []
for c in config:  # loop over entries in configuration (sensor category)
    if not 'start' in c:
        pprint(c)
    start = c['start']
    count = c['count']
    i = 0  # counter over the number of sensors within a category
    names = c['names']
    if 'postfixes' in c:
        postfixes = c['postfixes']
    else:
        postfixes = ' '
    size = calc_size(c)
    thislength = len(postfixes) * len(names)*size
    width = 99
    if c['type'] == 'int8':
        width = 8
    elif c['type'] == 'int16':
        width = 16
    elif c['type'] == 'fp16':
        width = 16
    elif c['type'] == 'char':
        width = 16
    elif c['type'] == 'uint32_t':
        width = 32
    elif c['type'] == 'uint16_t':
        width = 16
    entries.append(reg(c['name'], start, start + thislength - 1, thislength, width))
if args.verbose:
    for e in entries:
        print(e)

#check for overlaps and overloads
for i in range(len(entries)):
    for j in range(i+1, len(entries)):
        if entries[i].overlaps(entries[j]):
            print(f"{entries[i].name} overlaps with {entries[j].name}")

if entries[len(entries)-1].overloads():
    print(f"{entries[len(entries)-1].name} overloads")
