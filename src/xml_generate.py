# %%
import yaml
from pprint import pprint
import xml.etree.ElementTree as ET


# %%
verbose=False

# %%
with open('../data/zynqmon_2.yml') as f:
    y = yaml.load(f, Loader=yaml.FullLoader)
    if (verbose) :
        pprint(y)


# %%
def makeNode(parent: ET.Element, id: str, c: dict, addr2: int, parent_id: str) -> ET.Element:
    node = ET.SubElement(parent, 'node')
    id = id.replace(' ', '_')
    start = c['start']
    count = c['count']
    node.set('id', id)
    # address is half of the sensor address since these are 32 bit addresses
    addr = int(addr2/2)
    remain = addr2 % 2
    node.set('address', str(hex(addr)))
    # this appears to be on all the nodes
    node.set("permission", "r")
    # set parameter based on the type
    if (c['type'] == 'int8'):
        width = 8
        format = "d"
    elif (c['type'] == 'int16'):
        width = 16
        format = "d16"
    elif (c['type'] == 'fp16'):
        width = 16
        format = "fp16"
    elif (c['type'] == 'char'):
        width = 8
        format = "c"
    elif (c['type'] == 'uint32_t'):
        width = 32
        format = "u"
    elif (c['type'] == 'uint16_t'):
        width = 16
        format = "u16"
    else:
        print("ERROR: unknown type", c['type'])
        return None
    if 'size' in c: # if there is a size, it cannot have a mask
        node.set('size', str(hex(c['size'])))
    else: 
        mask = (1 << width) - 1
        if (remain == 0):
            node.set('mask', "0x{0:08X}".format(mask))
        else:
            node.set('mask', "0x{0:08X}".format(mask << 16))
    if 'extra' in c:
        extra = c['extra']
        if not "Column" in extra:
            extra = extra + ";Column=" + id
        if not "Row" in extra :
            if parent_id != "":
                extra = "Row=" + parent_id + ";" + extra
            else:
                extra = "Row=" + id + ";" + extra
    else:
        extra = ""
    node.set('parameters', "Format=" + format + ";" + extra)
    return node


# %%
# This is the parent (root) tag
# onto which other tags would be
# created
cm = ET.Element('node')
cm.set('id', 'CM')
cm.set('address', '0x00000000')


# %%
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
            #addr = int((start + i)/2)
            pp.set('address', str(hex(addr)))
            #pp.set("mask", "0x{0:08X}".format(0xFFFFFFFF))
            postfixes = c['postfixes']
            j = 0
            for p in postfixes:
                if p == 'RESERVED':
                    i += 1
                    j += 1
                    continue
                if verbose: 
                    print("adding postfix", p, "to node", n)
                node = makeNode(pp, p, c, j, n)
                i += 1
                j += 1
        else:
            start = c['start']
            makeNode(cm, n, c, start+i, "")
            i += 1

# %% [markdown]
# config = y['config']
# 
# for c in config: # loop over entries in configuration (sensor category)
#     print("here", c['name'])
#     if not 'start' in c:
#         pprint(c)
#     start = c['start']
#     count = c['count']
#     i = 0 # counter over the number of sensors within a category
#     names = c['names']
#     if 'prefixes' in c:
#         postfixes = c['postfixes']
#     else:
#         postfixes = ' '
#     for p in prefixes:
#         for n in names: # loop over sensor names
#             node = ET.SubElement(cm, 'node')
#             if ( len(p.lstrip()) > 0 ):
#                 id = p +"_" +  n.replace(' ', '_')
#             else:
#                 id = n.replace(' ', '_')
#             node.set('id', id)
#             addr = int((start + i)/2) ## address is half of the sensor address since these are 32 bit addresses
#             remain = (start + i) % 2
#             node.set('address', str(hex(addr)))
#             i += 1 # offset for next sensor
#             # this appears to be on all the nodes
#             node.set("permission", "r")
#             # set parameter based on the type
#             if ( c['type'] == 'int8' ):
#                 width=8
#                 format = "d"
#             elif ( c['type'] == 'fp16' ):
#                 width=16
#                 format = "fp16"
#             elif ( c['type'] == 'char' ):
#                 width=8
#                 format = "c"
#             elif ( c['type'] == 'uint32_t' ):
#                 width=32
#                 format = "u"
#             else:
#                 print("ERROR: unknown type", c['type'])
#                 break
#             mask = (1 << width) - 1
#             if (remain == 0):
#                 node.set('mask', "0x{0:08X}".format(mask))
#             else:
#                 node.set('mask', "0x{0:08X}".format(mask<<16))
#             if 'extra' in c:
#                 extra = c['extra']
#             else:
#                 extra = ""
#             node.set('parameters', "format=" + format + ";" + extra)
#         if 'size' in c:
#             print("size:", c['size'], c['name'])
#             node.set('size', str(hex(c['size'])))
# 

# %% [markdown]
# 

# %%
tree = ET.ElementTree(cm)
ET.indent(tree, space='\t')
tree.write("test2.xml")

# %%
def calcSize(c: dict) -> int:
    if 'size' in c:
        size = c['size']*2 # extra factor of 2 for 16 to 32 bit
    else:
        size = 1
    if '32' in c['type']:
        size = size * 2
    elif 'char' in c['type']:
        size = size // 2
    return size


# %%
# check overlaps
# create an object with a name, and a start end end register
# do so for every entry
# then ensure that no two objects overlap

# create a class to hold a name, start, end, and size
# add a method to check if two objects overlap
# and a pretty print method
from itertools import chain
class reg:
    def __init__(self, name, start, end, size):
        self.name = name
        self.start = start
        self.end = end
        self.size = size
    def __str__(self):
        return "name: " + self.name + " start: " + str(self.start) + " end: " + str(self.end) + " size: " + str(self.size)
    def __repr__(self):
        return "name: " + self.name + " start: " + str(self.start) + " end: " + str(self.end) + " size: " + str(self.size)
    def overlaps(self, other):
        if (self.start <= other.start and self.end >= other.start):
            return True
        if (self.start <= other.end and self.end >= other.end):
            return True
        if (self.start >= other.start and self.end <= other.end):
            return True
        return False

# create a list of objects
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
    size = calcSize(c)    
    thislength = len(postfixes) * len(names)*size
    entries.append(reg(c['name'], start, start + thislength - 1, thislength))
pprint(entries)

# check for overlaps
for i in range(len(entries)):
    for j in range(i+1, len(entries)):
        if entries[i].overlaps(entries[j]):
            print(f"{entries[i].name} overlaps with {entries[j].name}")



