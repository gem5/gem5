# -*- coding: utf-8 -*-
"""
Device Tree Blob Parser

   Copyright 2014  Neil 'superna' Armstrong <superna9999@gmail.com>

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.

@author: Neil 'superna' Armstrong <superna9999@gmail.com>
"""

from __future__ import print_function
from __future__ import absolute_import

import string
import os
import json
from copy import deepcopy, copy
from struct import Struct, unpack, pack

FDT_MAGIC = 0xD00DFEED
FDT_BEGIN_NODE = 0x1
FDT_END_NODE = 0x2
FDT_PROP = 0x3
FDT_NOP = 0x4
FDT_END = 0x9

INDENT = " " * 4

FDT_MAX_VERSION = 17


class FdtProperty(object):
    """ Represents an empty property"""

    @staticmethod
    def __validate_dt_name(name):
        """Checks the name validity"""
        return not any([True for char in name if char not in string.printable])

    def __init__(self, name):
        """Init with name"""
        self.name = name
        if not FdtProperty.__validate_dt_name(self.name):
            raise Exception("Invalid name '%s'" % self.name)

    def get_name(self):
        """Get property name"""
        return self.name

    def __str__(self):
        """String representation"""
        return "Property(%s)" % self.name

    def dts_represent(self, depth=0):
        """Get dts string representation"""
        return INDENT * depth + self.name + ";"

    def dtb_represent(self, string_store, pos=0, version=17):
        """Get blob representation"""
        # print "%x:%s" % (pos, self)
        strpos = string_store.find(self.name + "\0")
        if strpos < 0:
            strpos = len(string_store)
            string_store += self.name + "\0"
        pos += 12
        return (pack(">III", FDT_PROP, 0, strpos), string_store, pos)

    def json_represent(self, depth=0):
        """Ouput JSON"""
        return "%s: null" % json.dumps(self.name)

    def to_raw(self):
        """Return RAW value representation"""
        return ""

    def __getitem__(self, value):
        """Returns No Items"""
        return None

    def __ne__(self, node):
        """Check property inequality
        """
        return not self.__eq__(node)

    def __eq__(self, node):
        """Check node equality
           check properties are the same (same values)
        """
        if not isinstance(node, FdtProperty):
            raise Exception("Invalid object type")
        if self.name != node.get_name():
            return False
        return True

    @staticmethod
    def __check_prop_strings(value):
        """Check property string validity
           Python version of util_is_printable_string from dtc
        """
        pos = 0
        posi = 0
        end = len(value)

        if not len(value):
            return None

        # Needed for python 3 support: If a bytes object is passed,
        # decode it with the ascii codec. If the decoding fails, assume
        # it was not a string object.
        try:
            value = value.decode("ascii")
        except ValueError:
            return None

        # Test both against string 0 and int 0 because of
        # python2/3 compatibility
        if value[-1] != "\0":
            return None

        while pos < end:
            posi = pos
            while (
                pos < end
                and value[pos] != "\0"
                and value[pos] in string.printable
                and value[pos] not in ("\r", "\n")
            ):
                pos += 1

            if value[pos] != "\0" or pos == posi:
                return None
            pos += 1

        return True

    @staticmethod
    def new_raw_property(name, raw_value):
        """Instantiate property with raw value type"""
        if FdtProperty.__check_prop_strings(raw_value):
            return FdtPropertyStrings.init_raw(name, raw_value)
        elif len(raw_value) and len(raw_value) % 4 == 0:
            return FdtPropertyWords.init_raw(name, raw_value)
        elif len(raw_value) and len(raw_value):
            return FdtPropertyBytes.init_raw(name, raw_value)
        else:
            return FdtProperty(name)


class FdtPropertyStrings(FdtProperty):
    """Property with strings as value"""

    @classmethod
    def __extract_prop_strings(cls, value):
        """Extract strings from raw_value"""
        return [st for st in value.decode("ascii").split("\0") if len(st)]

    def __init__(self, name, strings):
        """Init with strings"""
        FdtProperty.__init__(self, name)
        if not strings:
            raise Exception("Invalid strings")
        for stri in strings:
            if len(stri) == 0:
                raise Exception("Invalid strings")
            if any(
                [
                    True
                    for char in stri
                    if char not in string.printable or char in ("\r", "\n")
                ]
            ):
                raise Exception("Invalid chars in strings")
        self.strings = strings

    @classmethod
    def init_raw(cls, name, raw_value):
        """Init from raw"""
        return cls(name, cls.__extract_prop_strings(raw_value))

    def dts_represent(self, depth=0):
        """Get dts string representation"""
        return (
            INDENT * depth
            + self.name
            + ' = "'
            + '", "'.join(self.strings)
            + '";'
        )

    def dtb_represent(self, string_store, pos=0, version=17):
        """Get blob representation"""
        # print "%x:%s" % (pos, self)
        blob = pack("")
        for chars in self.strings:
            blob += chars.encode("ascii") + pack("b", 0)
        blob_len = len(blob)
        if version < 16 and (pos + 12) % 8 != 0:
            blob = pack("b", 0) * (8 - ((pos + 12) % 8)) + blob
        if blob_len % 4:
            blob += pack("b", 0) * (4 - (blob_len % 4))
        strpos = string_store.find(self.name + "\0")
        if strpos < 0:
            strpos = len(string_store)
            string_store += self.name + "\0"
        blob = pack(">III", FDT_PROP, blob_len, strpos) + blob
        pos += len(blob)
        return (blob, string_store, pos)

    def json_represent(self, depth=0):
        """Ouput JSON"""
        result = '%s: ["strings", ' % json.dumps(self.name)
        result += ", ".join([json.dumps(stri) for stri in self.strings])
        result += "]"
        return result

    def to_raw(self):
        """Return RAW value representation"""
        return "".join([chars + "\0" for chars in self.strings])

    def __str__(self):
        """String representation"""
        return "Property(%s,Strings:%s)" % (self.name, self.strings)

    def __getitem__(self, index):
        """Get strings, returns a string"""
        return self.strings[index]

    def __len__(self):
        """Get strings count"""
        return len(self.strings)

    def __eq__(self, node):
        """Check node equality
           check properties are the same (same values)
        """
        if not FdtProperty.__eq__(self, node):
            return False
        if self.__len__() != len(node):
            return False
        for index in range(self.__len__()):
            if self.strings[index] != node[index]:
                return False
        return True


class FdtPropertyWords(FdtProperty):
    """Property with words as value"""

    def __init__(self, name, words):
        """Init with words"""
        FdtProperty.__init__(self, name)
        for word in words:
            if not 0 <= word <= 4294967295:
                raise Exception(
                    (
                        "Invalid word value %d, requires "
                        + "0 <= number <= 4294967295"
                    )
                    % word
                )
        if not len(words):
            raise Exception("Invalid Words")
        self.words = words

    @classmethod
    def init_raw(cls, name, raw_value):
        """Init from raw"""
        if len(raw_value) % 4 == 0:
            words = [
                unpack(">I", raw_value[i : i + 4])[0]
                for i in range(0, len(raw_value), 4)
            ]
            return cls(name, words)
        else:
            raise Exception("Invalid raw Words")

    def dts_represent(self, depth=0):
        """Get dts string representation"""
        return (
            INDENT * depth
            + self.name
            + " = <"
            + " ".join(["0x%08x" % word for word in self.words])
            + ">;"
        )

    def dtb_represent(self, string_store, pos=0, version=17):
        """Get blob representation"""
        # # print "%x:%s" % (pos, self)
        strpos = string_store.find(self.name + "\0")
        if strpos < 0:
            strpos = len(string_store)
            string_store += self.name + "\0"
        blob = pack(">III", FDT_PROP, len(self.words) * 4, strpos) + pack(
            ""
        ).join([pack(">I", word) for word in self.words])
        pos += len(blob)
        return (blob, string_store, pos)

    def json_represent(self, depth=0):
        """Ouput JSON"""
        result = '%s: ["words", "' % json.dumps(self.name)
        result += '", "'.join(["0x%08x" % word for word in self.words])
        result += '"]'
        return result

    def to_raw(self):
        """Return RAW value representation"""
        return "".join([pack(">I", word) for word in self.words])

    def __str__(self):
        """String representation"""
        return "Property(%s,Words:%s)" % (self.name, self.words)

    def __getitem__(self, index):
        """Get words, returns a word integer"""
        return self.words[index]

    def __len__(self):
        """Get words count"""
        return len(self.words)

    def __eq__(self, node):
        """Check node equality
           check properties are the same (same values)
        """
        if not FdtProperty.__eq__(self, node):
            return False
        if self.__len__() != len(node):
            return False
        for index in range(self.__len__()):
            if self.words[index] != node[index]:
                return False
        return True


class FdtPropertyBytes(FdtProperty):
    """Property with signed bytes as value"""

    def __init__(self, name, bytez):
        """Init with bytes"""
        FdtProperty.__init__(self, name)
        for byte in bytez:
            if not -128 <= byte <= 127:
                raise Exception(
                    (
                        "Invalid value for byte %d, "
                        + "requires -128 <= number <= 127"
                    )
                    % byte
                )
        if not bytez:
            raise Exception("Invalid Bytes")
        self.bytes = bytez

    @classmethod
    def init_raw(cls, name, raw_value):
        """Init from raw"""
        return cls(name, unpack("b" * len(raw_value), raw_value))

    def dts_represent(self, depth=0):
        """Get dts string representation"""
        return (
            INDENT * depth
            + self.name
            + " = ["
            + " ".join(
                ["%02x" % (byte & int("ffffffff", 16)) for byte in self.bytes]
            )
            + "];"
        )

    def dtb_represent(self, string_store, pos=0, version=17):
        """Get blob representation"""
        # print "%x:%s" % (pos, self)
        strpos = string_store.find(self.name + "\0")
        if strpos < 0:
            strpos = len(string_store)
            string_store += self.name + "\0"
        blob = pack(">III", FDT_PROP, len(self.bytes), strpos)
        blob += pack("").join([pack(">b", byte) for byte in self.bytes])
        if len(blob) % 4:
            blob += pack("b", 0) * (4 - (len(blob) % 4))
        pos += len(blob)
        return (blob, string_store, pos)

    def json_represent(self, depth=0):
        """Ouput JSON"""
        result = '%s: ["bytes", "' % json.dumps(self.name)
        result += '", "'.join(["%02x" % byte for byte in self.bytes])
        result += '"]'
        return result

    def to_raw(self):
        """Return RAW value representation"""
        return "".join([pack(">b", byte) for byte in self.bytes])

    def __str__(self):
        """String representation"""
        return "Property(%s,Bytes:%s)" % (self.name, self.bytes)

    def __getitem__(self, index):
        """Get bytes, returns a byte"""
        return self.bytes[index]

    def __len__(self):
        """Get strings count"""
        return len(self.bytes)

    def __eq__(self, node):
        """Check node equality
           check properties are the same (same values)
        """
        if not FdtProperty.__eq__(self, node):
            return False
        if self.__len__() != len(node):
            return False
        for index in range(self.__len__()):
            if self.bytes[index] != node[index]:
                return False
        return True


class FdtNop(object):  # pylint: disable-msg=R0903
    """Nop child representation"""

    def __init__(self):
        """Init with nothing"""

    def get_name(self):  # pylint: disable-msg=R0201
        """Return name"""
        return None

    def __str__(self):
        """String representation"""
        return ""

    def dts_represent(self, depth=0):  # pylint: disable-msg=R0201
        """Get dts string representation"""
        return INDENT * depth + "// [NOP]"

    def dtb_represent(self, string_store, pos=0, version=17):
        """Get blob representation"""
        # print "%x:%s" % (pos, self)
        pos += 4
        return (pack(">I", FDT_NOP), string_store, pos)


class FdtNode(object):
    """Node representation"""

    @staticmethod
    def __validate_dt_name(name):
        """Checks the name validity"""
        return not any([True for char in name if char not in string.printable])

    def __init__(self, name):
        """Init node with name"""
        self.name = name
        self.subdata = []
        self.parent = None
        if not FdtNode.__validate_dt_name(self.name):
            raise Exception("Invalid name '%s'" % self.name)

    def get_name(self):
        """Get property name"""
        return self.name

    def __check_name_duplicate(self, name):
        """Checks if name is not in a subnode"""
        for data in self.subdata:
            if not isinstance(data, FdtNop) and data.get_name() == name:
                return True
        return False

    def add_subnode(self, node):
        """Add child, deprecated use append()"""
        self.append(node)

    def add_raw_attribute(self, name, raw_value):
        """Construct a raw attribute and add to child"""
        self.append(FdtProperty.new_raw_property(name, raw_value))

    def set_parent_node(self, node):
        """Set parent node, None and FdtNode accepted"""
        if node is not None and not isinstance(node, FdtNode):
            raise Exception("Invalid object type")
        self.parent = node

    def get_parent_node(self):
        """Get parent node"""
        return self.parent

    def __str__(self):
        """String representation"""
        return "Node(%s)" % self.name

    def dts_represent(self, depth=0):
        """Get dts string representation"""
        result = ("\n").join(
            [sub.dts_represent(depth + 1) for sub in self.subdata]
        )
        if len(result) > 0:
            result += "\n"
        return (
            INDENT * depth
            + self.name
            + " {\n"
            + result
            + INDENT * depth
            + "};"
        )

    def dtb_represent(self, strings_store, pos=0, version=17):
        """Get blob representation
           Pass string storage as strings_store, pos for current node start
           and version as current dtb version
        """
        # print "%x:%s" % (pos, self)
        strings = strings_store
        if self.get_name() == "/":
            blob = pack(">II", FDT_BEGIN_NODE, 0)
        else:
            blob = pack(">I", FDT_BEGIN_NODE)
            blob += self.get_name().encode("ascii") + pack("b", 0)
        if len(blob) % 4:
            blob += pack("b", 0) * (4 - (len(blob) % 4))
        pos += len(blob)
        for sub in self.subdata:
            (data, strings, pos) = sub.dtb_represent(strings, pos, version)
            blob += data
        pos += 4
        blob += pack(">I", FDT_END_NODE)
        return (blob, strings, pos)

    def json_represent(self, depth=0):
        """Get dts string representation"""
        result = (",\n" + INDENT * (depth + 1)).join(
            [
                sub.json_represent(depth + 1)
                for sub in self.subdata
                if not isinstance(sub, FdtNop)
            ]
        )
        if len(result) > 0:
            result = INDENT + result + "\n" + INDENT * depth
        if self.get_name() == "/":
            return "{\n" + INDENT * (depth) + result + "}"
        else:
            return (
                json.dumps(self.name)
                + ": {\n"
                + INDENT * (depth)
                + result
                + "}"
            )

    def __getitem__(self, index):
        """Get subnodes, returns either a Node, a Property or a Nop"""
        return self.subdata[index]

    def __setitem__(self, index, subnode):
        """Set node at index, replacing previous subnode,
           must not be a duplicate name
        """
        if self.subdata[
            index
        ].get_name() != subnode.get_name() and self.__check_name_duplicate(
            subnode.get_name()
        ):
            raise Exception("%s : %s subnode already exists" % (self, subnode))
        if not isinstance(subnode, (FdtNode, FdtProperty, FdtNop)):
            raise Exception("Invalid object type")
        self.subdata[index] = subnode

    def __len__(self):
        """Get strings count"""
        return len(self.subdata)

    def __ne__(self, node):
        """Check node inequality
           i.e. is subnodes are the same, in either order
           and properties are the same (same values)
           The FdtNop is excluded from the check
        """
        return not self.__eq__(node)

    def __eq__(self, node):
        """Check node equality
           i.e. is subnodes are the same, in either order
           and properties are the same (same values)
           The FdtNop is excluded from the check
        """
        if not isinstance(node, FdtNode):
            raise Exception("Invalid object type")
        if self.name != node.get_name():
            return False
        curnames = set(
            [
                subnode.get_name()
                for subnode in self.subdata
                if not isinstance(subnode, FdtNop)
            ]
        )
        cmpnames = set(
            [
                subnode.get_name()
                for subnode in node
                if not isinstance(subnode, FdtNop)
            ]
        )
        if curnames != cmpnames:
            return False
        for subnode in [
            subnode
            for subnode in self.subdata
            if not isinstance(subnode, FdtNop)
        ]:
            index = node.index(subnode.get_name())
            if subnode != node[index]:
                return False
        return True

    def append(self, subnode):
        """Append subnode, same as add_subnode"""
        if self.__check_name_duplicate(subnode.get_name()):
            raise Exception("%s : %s subnode already exists" % (self, subnode))
        if not isinstance(subnode, (FdtNode, FdtProperty, FdtNop)):
            raise Exception("Invalid object type")
        self.subdata.append(subnode)

    def pop(self, index=-1):
        """Remove and returns subnode at index, default the last"""
        return self.subdata.pop(index)

    def insert(self, index, subnode):
        """Insert subnode before index, must not be a duplicate name"""
        if self.__check_name_duplicate(subnode.get_name()):
            raise Exception("%s : %s subnode already exists" % (self, subnode))
        if not isinstance(subnode, (FdtNode, FdtProperty, FdtNop)):
            raise Exception("Invalid object type")
        self.subdata.insert(index, subnode)

    def _find(self, name):
        """Find name in subnodes"""
        for i in range(0, len(self.subdata)):
            if (
                not isinstance(self.subdata[i], FdtNop)
                and name == self.subdata[i].get_name()
            ):
                return i
        return None

    def remove(self, name):
        """Remove subnode with the name
           Raises ValueError is not present
        """
        index = self._find(name)
        if index is None:
            raise ValueError("Not present")
        return self.subdata.pop(index)

    def index(self, name):
        """Returns position of subnode with the name
           Raises ValueError is not present
        """
        index = self._find(name)
        if index is None:
            raise ValueError("Not present")
        return index

    def merge(self, node):
        """Merge two nodes and subnodes
           Replace current properties with the given properties
        """
        if not isinstance(node, FdtNode):
            raise Exception("Can only merge with a FdtNode")
        for subnode in [
            obj for obj in node if isinstance(obj, (FdtNode, FdtProperty))
        ]:
            index = self._find(subnode.get_name())
            if index is None:
                dup = deepcopy(subnode)
                if isinstance(subnode, FdtNode):
                    dup.set_parent_node(self)
                self.append(dup)
            elif isinstance(subnode, FdtNode):
                self.subdata[index].merge(subnode)
            else:
                self.subdata[index] = copy(subnode)

    def walk(self):
        """Walk into subnodes and yield paths and objects
           Returns set with (path string, node object)
        """
        node = self
        start = 0
        hist = []
        curpath = []

        while True:
            for index in range(start, len(node)):
                if isinstance(node[index], (FdtNode, FdtProperty)):
                    yield (
                        "/" + "/".join(curpath + [node[index].get_name()]),
                        node[index],
                    )
                if isinstance(node[index], FdtNode):
                    if len(node[index]):
                        hist.append((node, index + 1))
                        curpath.append(node[index].get_name())
                        node = node[index]
                        start = 0
                        index = -1
                        break
            if index >= 0:
                if len(hist):
                    (node, start) = hist.pop()
                    curpath.pop()
                else:
                    break


class Fdt(object):
    """Flattened Device Tree representation"""

    def __init__(self, version=17, last_comp_version=16, boot_cpuid_phys=0):
        """Init FDT object with version and boot values"""
        self.header = {
            "magic": FDT_MAGIC,
            "totalsize": 0,
            "off_dt_struct": 0,
            "off_dt_strings": 0,
            "off_mem_rsvmap": 0,
            "version": version,
            "last_comp_version": last_comp_version,
            "boot_cpuid_phys": boot_cpuid_phys,
            "size_dt_strings": 0,
            "size_dt_struct": 0,
        }
        self.rootnode = None
        self.prenops = None
        self.postnops = None
        self.reserve_entries = None

    def add_rootnode(self, rootnode, prenops=None, postnops=None):
        """Add root node"""
        self.rootnode = rootnode
        self.prenops = prenops
        self.postnops = postnops

    def get_rootnode(self):
        """Get root node"""
        return self.rootnode

    def add_reserve_entries(self, reserve_entries):
        """Add reserved entries as list of dict with
           'address' and 'size' keys"""
        self.reserve_entries = reserve_entries

    def to_dts(self):
        """Export to DTS representation in string format"""
        result = "/dts-v1/;\n"
        result += "// version:\t\t%d\n" % self.header["version"]
        result += (
            "// last_comp_version:\t%d\n" % self.header["last_comp_version"]
        )
        if self.header["version"] >= 2:
            result += (
                "// boot_cpuid_phys:\t0x%x\n" % self.header["boot_cpuid_phys"]
            )
        result += "\n"
        if self.reserve_entries is not None:
            for entry in self.reserve_entries:
                result += "/memreserve/ "
                if entry["address"]:
                    result += "%#x " % entry["address"]
                else:
                    result += "0 "
                if entry["size"]:
                    result += "%#x" % entry["size"]
                else:
                    result += "0"
                result += ";\n"
        if self.prenops:
            result += "\n".join([nop.dts_represent() for nop in self.prenops])
            result += "\n"
        if self.rootnode is not None:
            result += self.rootnode.dts_represent()
        if self.postnops:
            result += "\n"
            result += "\n".join([nop.dts_represent() for nop in self.postnops])
        return result

    def to_dtb(self):
        """Export to Blob format"""
        if self.rootnode is None:
            return None
        blob_reserve_entries = pack("")
        if self.reserve_entries is not None:
            for entry in self.reserve_entries:
                blob_reserve_entries += pack(
                    ">QQ", entry["address"], entry["size"]
                )
        blob_reserve_entries += pack(">QQ", 0, 0)
        header_size = 7 * 4
        if self.header["version"] >= 2:
            header_size += 4
        if self.header["version"] >= 3:
            header_size += 4
        if self.header["version"] >= 17:
            header_size += 4
        header_adjust = pack("")
        if header_size % 8 != 0:
            header_adjust = pack("b", 0) * (8 - (header_size % 8))
            header_size += len(header_adjust)
        dt_start = header_size + len(blob_reserve_entries)
        # print "dt_start %d" % dt_start
        (blob_dt, blob_strings, dt_pos) = self.rootnode.dtb_represent(
            "", dt_start, self.header["version"]
        )
        if self.prenops is not None:
            blob_dt = (
                pack("").join(
                    [nop.dtb_represent("")[0] for nop in self.prenops]
                )
                + blob_dt
            )
        if self.postnops is not None:
            blob_dt += pack("").join(
                [nop.dtb_represent("")[0] for nop in self.postnops]
            )
        blob_dt += pack(">I", FDT_END)
        self.header["size_dt_strings"] = len(blob_strings)
        self.header["size_dt_struct"] = len(blob_dt)
        self.header["off_mem_rsvmap"] = header_size
        self.header["off_dt_struct"] = dt_start
        self.header["off_dt_strings"] = dt_start + len(blob_dt)
        self.header["totalsize"] = dt_start + len(blob_dt) + len(blob_strings)
        blob_header = pack(
            ">IIIIIII",
            self.header["magic"],
            self.header["totalsize"],
            self.header["off_dt_struct"],
            self.header["off_dt_strings"],
            self.header["off_mem_rsvmap"],
            self.header["version"],
            self.header["last_comp_version"],
        )
        if self.header["version"] >= 2:
            blob_header += pack(">I", self.header["boot_cpuid_phys"])
        if self.header["version"] >= 3:
            blob_header += pack(">I", self.header["size_dt_strings"])
        if self.header["version"] >= 17:
            blob_header += pack(">I", self.header["size_dt_struct"])
        return (
            blob_header
            + header_adjust
            + blob_reserve_entries
            + blob_dt
            + blob_strings.encode("ascii")
        )

    def to_json(self):
        """Ouput JSON"""
        if self.rootnode is None:
            return None
        return self.rootnode.json_represent()

    def resolve_path(self, path):
        """Resolve path like /memory/reg and return either a FdtNode,
            a FdtProperty or None"""
        if self.rootnode is None:
            return None
        if not path.startswith("/"):
            return None
        if len(path) > 1 and path.endswith("/"):
            path = path[:-1]
        if path == "/":
            return self.rootnode
        curnode = self.rootnode
        for subpath in path[1:].split("/"):
            found = None
            if not isinstance(curnode, FdtNode):
                return None
            for node in curnode:
                if subpath == node.get_name():
                    found = node
                    break
            if found is None:
                return None
            curnode = found
        return curnode


def _add_json_to_fdtnode(node, subjson):
    """Populate FdtNode with JSON dict items"""
    for (key, value) in subjson.items():
        if isinstance(value, dict):
            subnode = FdtNode(key)
            subnode.set_parent_node(node)
            node.append(subnode)
            _add_json_to_fdtnode(subnode, value)
        elif isinstance(value, list):
            if len(value) < 2:
                raise Exception("Invalid list for %s" % key)
            if value[0] == "words":
                words = [int(word, 16) for word in value[1:]]
                node.append(FdtPropertyWords(key, words))
            elif value[0] == "bytes":
                bytez = [int(byte, 16) for byte in value[1:]]
                node.append(FdtPropertyBytes(key, bytez))
            elif value[0] == "strings":
                node.append(FdtPropertyStrings(key, [s for s in value[1:]]))
            else:
                raise Exception("Invalid list for %s" % key)
        elif value is None:
            node.append(FdtProperty(key))
        else:
            raise Exception("Invalid value for %s" % key)


def FdtJsonParse(buf):
    """Import FDT from JSON representation, see JSONDeviceTree.md for
       structure and encoding
       Returns an Fdt object
    """
    tree = json.loads(buf)

    root = FdtNode("/")

    _add_json_to_fdtnode(root, tree)

    fdt = Fdt()
    fdt.add_rootnode(root)
    return fdt


def FdtFsParse(path):
    """Parse device tree filesystem and return a Fdt instance
       Should be /proc/device-tree on a device, or the fusemount.py
       mount point.
    """
    root = FdtNode("/")

    if path.endswith("/"):
        path = path[:-1]

    nodes = {path: root}

    for subpath, subdirs, files in os.walk(path):
        if subpath not in nodes.keys():
            raise Exception("os.walk error")
        cur = nodes[subpath]
        for f in files:
            with open(subpath + "/" + f, "rb") as content_file:
                content = content_file.read()
            prop = FdtProperty.new_raw_property(f, content)
            cur.add_subnode(prop)
        for subdir in subdirs:
            subnode = FdtNode(subdir)
            cur.add_subnode(subnode)
            subnode.set_parent_node(cur)
            nodes[subpath + "/" + subdir] = subnode

    fdt = Fdt()
    fdt.add_rootnode(root)
    return fdt


class FdtBlobParse(object):  # pylint: disable-msg=R0903
    """Parse from file input"""

    __fdt_header_format = ">IIIIIII"
    __fdt_header_names = (
        "magic",
        "totalsize",
        "off_dt_struct",
        "off_dt_strings",
        "off_mem_rsvmap",
        "version",
        "last_comp_version",
    )

    __fdt_reserve_entry_format = ">QQ"
    __fdt_reserve_entry_names = ("address", "size")

    __fdt_dt_cell_format = ">I"
    __fdt_dt_prop_format = ">II"
    __fdt_dt_tag_name = {
        FDT_BEGIN_NODE: "node_begin",
        FDT_END_NODE: "node_end",
        FDT_PROP: "prop",
        FDT_NOP: "nop",
        FDT_END: "end",
    }

    def __extract_fdt_header(self):
        """Extract DTB header"""
        header = Struct(self.__fdt_header_format)
        header_entry = Struct(">I")
        data = self.infile.read(header.size)
        result = dict(zip(self.__fdt_header_names, header.unpack_from(data)))
        if result["version"] >= 2:
            data = self.infile.read(header_entry.size)
            result["boot_cpuid_phys"] = header_entry.unpack_from(data)[0]
        if result["version"] >= 3:
            data = self.infile.read(header_entry.size)
            result["size_dt_strings"] = header_entry.unpack_from(data)[0]
        if result["version"] >= 17:
            data = self.infile.read(header_entry.size)
            result["size_dt_struct"] = header_entry.unpack_from(data)[0]
        return result

    def __extract_fdt_reserve_entries(self):
        """Extract reserved memory entries"""
        header = Struct(self.__fdt_reserve_entry_format)
        entries = []
        self.infile.seek(self.fdt_header["off_mem_rsvmap"])
        while True:
            data = self.infile.read(header.size)
            result = dict(
                zip(self.__fdt_reserve_entry_names, header.unpack_from(data))
            )
            if result["address"] == 0 and result["size"] == 0:
                return entries
            entries.append(result)

    def __extract_fdt_nodename(self):
        """Extract node name"""
        data = ""
        pos = self.infile.tell()
        while True:
            byte = self.infile.read(1)
            if ord(byte) == 0:
                break
            data += byte.decode("ascii")
        align_pos = pos + len(data) + 1
        align_pos = ((align_pos) + ((4) - 1)) & ~((4) - 1)
        self.infile.seek(align_pos)
        return data

    def __extract_fdt_string(self, prop_string_pos):
        """Extract string from string pool"""
        data = ""
        pos = self.infile.tell()
        self.infile.seek(self.fdt_header["off_dt_strings"] + prop_string_pos)
        while True:
            byte = self.infile.read(1)
            if ord(byte) == 0:
                break
            data += byte.decode("ascii")
        self.infile.seek(pos)
        return data

    def __extract_fdt_prop(self):
        """Extract property"""
        prop = Struct(self.__fdt_dt_prop_format)
        pos = self.infile.tell()
        data = self.infile.read(prop.size)
        (prop_size, prop_string_pos) = prop.unpack_from(data)

        prop_start = pos + prop.size
        if self.fdt_header["version"] < 16 and prop_size >= 8:
            prop_start = ((prop_start) + ((8) - 1)) & ~((8) - 1)

        self.infile.seek(prop_start)
        value = self.infile.read(prop_size)

        align_pos = self.infile.tell()
        align_pos = ((align_pos) + ((4) - 1)) & ~((4) - 1)
        self.infile.seek(align_pos)

        return (self.__extract_fdt_string(prop_string_pos), value)

    def __extract_fdt_dt(self):
        """Extract tags"""
        cell = Struct(self.__fdt_dt_cell_format)
        tags = []
        self.infile.seek(self.fdt_header["off_dt_struct"])
        while True:
            data = self.infile.read(cell.size)
            if len(data) < cell.size:
                break
            tag, = cell.unpack_from(data)
            # print "*** %s" % self.__fdt_dt_tag_name.get(tag, '')
            if self.__fdt_dt_tag_name.get(tag, "") in "node_begin":
                name = self.__extract_fdt_nodename()
                if len(name) == 0:
                    name = "/"
                tags.append((tag, name))
            elif self.__fdt_dt_tag_name.get(tag, "") in ("node_end", "nop"):
                tags.append((tag, ""))
            elif self.__fdt_dt_tag_name.get(tag, "") in "end":
                tags.append((tag, ""))
                break
            elif self.__fdt_dt_tag_name.get(tag, "") in "prop":
                propdata = self.__extract_fdt_prop()
                tags.append((tag, propdata))
            else:
                print("Unknown Tag %d" % tag)
        return tags

    def __init__(self, infile):
        """Init with file input"""
        self.infile = infile
        self.fdt_header = self.__extract_fdt_header()
        if self.fdt_header["magic"] != FDT_MAGIC:
            raise Exception("Invalid Magic")
        if self.fdt_header["version"] > FDT_MAX_VERSION:
            raise Exception("Invalid Version %d" % self.fdt_header["version"])
        if self.fdt_header["last_comp_version"] > FDT_MAX_VERSION - 1:
            raise Exception(
                "Invalid last compatible Version %d"
                % self.fdt_header["last_comp_version"]
            )
        self.fdt_reserve_entries = self.__extract_fdt_reserve_entries()
        self.fdt_dt_tags = self.__extract_fdt_dt()

    def __to_nodes(self):
        """Represent fdt as Node and properties structure
           Returns a set with the pre-node Nops, the Root Node,
            and the post-node Nops.
        """
        prenops = []
        postnops = []
        rootnode = None
        curnode = None
        for tag in self.fdt_dt_tags:
            if self.__fdt_dt_tag_name.get(tag[0], "") in "node_begin":
                newnode = FdtNode(tag[1])
                if rootnode is None:
                    rootnode = newnode
                if curnode is not None:
                    curnode.add_subnode(newnode)
                    newnode.set_parent_node(curnode)
                curnode = newnode
            elif self.__fdt_dt_tag_name.get(tag[0], "") in "node_end":
                if curnode is not None:
                    curnode = curnode.get_parent_node()
            elif self.__fdt_dt_tag_name.get(tag[0], "") in "nop":
                if curnode is not None:
                    curnode.add_subnode(FdtNop())
                elif rootnode is not None:
                    postnops.append(FdtNop())
                else:
                    prenops.append(FdtNop())
            elif self.__fdt_dt_tag_name.get(tag[0], "") in "prop":
                if curnode is not None:
                    curnode.add_raw_attribute(tag[1][0], tag[1][1])
            elif self.__fdt_dt_tag_name.get(tag[0], "") in "end":
                continue
        return (prenops, rootnode, postnops)

    def to_fdt(self):
        """Create a fdt object
            Returns a Fdt object
        """
        if self.fdt_header["version"] >= 2:
            boot_cpuid_phys = self.fdt_header["boot_cpuid_phys"]
        else:
            boot_cpuid_phys = 0
        fdt = Fdt(
            version=self.fdt_header["version"],
            last_comp_version=self.fdt_header["last_comp_version"],
            boot_cpuid_phys=boot_cpuid_phys,
        )
        (prenops, rootnode, postnops) = self.__to_nodes()
        fdt.add_rootnode(rootnode, prenops=prenops, postnops=postnops)
        fdt.add_reserve_entries(self.fdt_reserve_entries)
        return fdt
