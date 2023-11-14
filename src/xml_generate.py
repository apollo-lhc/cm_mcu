"""Generate XML file from YAML input"""
import xml.etree.ElementTree as ET
from pprint import pprint
import argparse
import yaml

#% %
def make_node(parent: ET.Element, myid: str, thedict: dict, addr2: int,
              parent_id: str) -> ET.Element:
    """create the node to be inserted into the xml tree"""
# pylint: disable=too-many-branches
# I disable this check because as far as I can tell it's wrong
    thenode = ET.SubElement(parent, 'node')
    myid = myid.replace(' ', '_')
    thenode.set('id', myid)
#address is half of the sensor address since these are 32 bit addresses
    theaddr = int(addr2/2)
    remain = addr2 % 2
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
            extra = extra + ";Column=" + myid
        if not "Row" in extra:
            if parent_id != "":
                extra = "Row=" + parent_id + ";" + extra
            else:
                extra = "Row=" + myid + ";" + extra
    else:
        extra = ""
    thenode.set('parameters', "Format=" + theformat + ";" + extra)
    return thenode


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
    def __init__(self, name, sta, end, sz):
        self.name = name
        self.start = sta
        self.end = end
        self.size = sz

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
        if self.start + self.size >= 255:
            return True
        return False


# custom file type for yaml file, to be used with argparse
def yaml_file(filename):
    """custom file type for yaml file, to be used with argparse"""
    if not filename.endswith('.yaml'):
        raise argparse.ArgumentTypeError('File must have a .yaml extension')
    return filename

parser = argparse.ArgumentParser(description='Process YAML for XML.')
parser.add_argument('-v', '--verbose', action='store_true',
                    help='increase output verbosity')
parser.add_argument('-d', '--directory', type=str, help='output directory')
# this argument is required, one input file ending with yaml extension
parser.add_argument('input_file', metavar='file', type=yaml_file,
                    help='input yaml file name')

args = parser.parse_args()

if args.verbose:
    print('Verbose mode on')

if args.directory:
    print('Output directory:', args.directory)

if args.verbose:
    print('Input file names:', args.input_file)



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

#% %
config = y['config']

for c in config:  # loop over entries in configuration (sensor category)
    i = 0  # counter over the number of sensors within a category
    names = c['names']
    for n in names:  # loop over names of sensors within a category
        if 'postfixes' in c:
            pp = node = ET.SubElement(cm, 'node')
            pp.set('id', n)
            start = c['start']
            addr = int((start + i)/2)
            pp.set('address', str(hex(addr)))
            postfixes = c['postfixes']
            j = 0
            for p in postfixes:
                if p == 'RESERVED':
                    i += 1
                    j += 1
                    continue
                if args.verbose:
                    print("adding postfix", p, "to node", n)
                node = make_node(pp, p, c, j, n)
                i += 1
                j += 1
        else:
            start = c['start']
            make_node(cm, n, c, start+i, "")
            i += 1
tree = ET.ElementTree(cm)
ET.indent(tree, space='\t')
# create output file name based on input file, replacing 'yaml' with 'xml'
out_name = args.input_file[:-len('yaml')] + 'xml'
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
    entries.append(reg(c['name'], start, start + thislength - 1, thislength))
pprint(entries)

#check for overlaps and overloads
for i in range(len(entries)):
    for j in range(i+1, len(entries)):
        if entries[i].overlaps(entries[j]):
            print(f"{entries[i].name} overlaps with {entries[j].name}")

if entries[len(entries)-1].overloads():
    print(f"{entries[len(entries)-1].name} overloads")
