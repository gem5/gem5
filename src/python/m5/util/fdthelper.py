# Copyright (c) 2016,2019 ARM Limited
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Glenn Bergmans

from m5.ext.pyfdt import pyfdt
import re
import os
from m5.SimObject import SimObject
from m5.util import fatal


class FdtProperty(pyfdt.FdtProperty):
    """Create a property without values."""

    pass


class FdtPropertyWords(pyfdt.FdtPropertyWords):
    """Create a property with word (32-bit unsigned) values."""

    def __init__(self, name, words):
        if type(words) != list:
            words = [words]
        # Make sure all values are ints (use automatic base detection if the
        # type is str)
        words = [int(w, base=0) if type(w) == str else int(w) for w in words]
        super().__init__(name, words)


class FdtPropertyStrings(pyfdt.FdtPropertyStrings):
    """Create a property with string values."""

    def __init__(self, name, strings):
        if type(strings) == str:
            strings = [strings]
        strings = [
            str(string) for string in strings
        ]  # Make all values strings
        super().__init__(name, strings)


class FdtPropertyBytes(pyfdt.FdtPropertyBytes):
    """Create a property with integer (8-bit signed) values."""

    def __init__(self, name, values):
        if type(values) != list:
            values = [values]
        # Make sure all values are ints (use automatic base detection if the
        # type is str)
        values = [
            int(v, base=0) if isinstance(v, str) else int(v) for v in values
        ]
        super().__init__(name, values)


class FdtState:
    """Class for maintaining state while recursively generating a flattened
    device tree. The state tracks address, size and CPU address cell sizes, and
    maintains a dictionary of allocated phandles."""

    phandle_counter = 0
    phandles = dict()

    def __init__(self, **kwargs):
        """Instantiate values of this state. The state can only be initialized
        once."""

        self.addr_cells = kwargs.pop("addr_cells", 0)
        self.size_cells = kwargs.pop("size_cells", 0)
        self.cpu_cells = kwargs.pop("cpu_cells", 0)
        self.interrupt_cells = kwargs.pop("interrupt_cells", 0)

    def phandle(self, obj):
        """Return a unique phandle number for a key. The key can be a SimObject
        or any value that is castable to a string. If the phandle doesn't exist
        a new one is created, otherwise the existing one is returned."""

        if isinstance(obj, SimObject):
            key = str(id(obj))
        else:
            try:
                key = str(obj)
            except ValueError:
                raise ValueError("Phandle keys must be castable to str")

        if not key in FdtState.phandles:
            FdtState.phandle_counter += 1

        return FdtState.phandles.setdefault(key, FdtState.phandle_counter)

    def resetPhandles(self):
        FdtState.phandle_counter = 0
        FdtState.phandles = dict()

    def int_to_cells(self, value, cells):
        """Helper function for: generates a list of 32 bit cells from an int,
        used to split up addresses in appropriate 32 bit chunks."""
        value = int(value)

        if (value >> (32 * cells)) != 0:
            fatal("Value %d doesn't fit in %d cells" % (value, cells))

        return [
            (value >> 32 * (x - 1)) & 0xFFFFFFFF for x in range(cells, 0, -1)
        ]

    def addrCells(self, addr):
        """Format an integer type according to the address_cells value of this
        state."""
        return self.int_to_cells(addr, self.addr_cells)

    def CPUAddrCells(self, addr):
        """Format an integer type according to the cpu_cells value of this
        state."""
        return self.int_to_cells(addr, self.cpu_cells)

    def sizeCells(self, size):
        """Format an integer type according to the size_cells value of this
        state."""
        return self.int_to_cells(size, self.size_cells)

    def interruptCells(self, interrupt):
        """Format an integer type according to the interrupt_cells value
        of this state."""
        return self.int_to_cells(interrupt, self.interrupt_cells)

    def addrCellsProperty(self):
        """Return an #address-cells property with the value of this state."""
        return FdtPropertyWords("#address-cells", self.addr_cells)

    def sizeCellsProperty(self):
        """Return an #size-cells property with the value of this state."""
        return FdtPropertyWords("#size-cells", self.size_cells)

    def CPUCellsProperty(self):
        """Return an #address-cells property for cpu nodes with the value
        of this state."""
        return FdtPropertyWords("#address-cells", self.cpu_cells)

    def interruptCellsProperty(self):
        """Return an #interrupt-cells property for cpu nodes with the value
        of this state."""
        return FdtPropertyWords("#interrupt-cells", self.interrupt_cells)


class FdtNop(pyfdt.FdtNop):
    """Create an empty node."""

    pass


class FdtNode(pyfdt.FdtNode):
    def __init__(self, name, obj=None):
        """Create a new node and immediately set the phandle property, if obj
        is supplied"""
        super().__init__(name)
        if obj != None:
            self.appendPhandle(obj)

    def append(self, subnodes):
        """Change the behavior of the normal append to override if a node with
        the same name already exists or merge if the name exists and is a node
        type. Can also take a list of subnodes, that each get appended."""
        if not hasattr(subnodes, "__iter__"):
            subnodes = [subnodes]

        for subnode in subnodes:
            try:
                if not issubclass(type(subnode), pyfdt.FdtNop):
                    index = self.index(subnode.name)
                    item = self.pop(index)
                else:
                    item = None
            except ValueError:
                item = None

            if isinstance(item, pyfdt.FdtNode) and isinstance(
                subnode, pyfdt.FdtNode
            ):
                item.merge(subnode)
                subnode = item

            super().append(subnode)

    def appendList(self, subnode_list):
        """Append all properties/nodes in the iterable."""
        for subnode in subnode_list:
            self.append(subnode)

    def appendCompatible(self, compatible):
        """Append a compatible property with the supplied compatibility
        strings."""
        if isinstance(compatible, str):
            compatible = [compatible]
        self.append(FdtPropertyStrings("compatible", compatible))

    def appendPhandle(self, obj):
        """Append a phandle property to this node with the phandle of the
        supplied object."""
        # Create a bogus state because we only need the Phandle dictionary
        state = FdtState(addr_cells=1, size_cells=1, cpu_cells=1)

        phandle = state.phandle(obj)
        self.append(FdtPropertyWords("phandle", [phandle]))


class Fdt(pyfdt.Fdt):
    def sortNodes(self, node):
        """Move all properties to the beginning and subnodes to the end
        while maintaining the order of the subnodes. DTB files require the
        properties to go before the nodes, but the PyFdt doesn't account for
        defining nodes and properties in a random order."""
        properties = FdtNode(node.name)
        subnodes = FdtNode(node.name)

        while len(node):
            subnode = node.pop(0)
            if issubclass(type(subnode), pyfdt.FdtNode):
                subnode = self.sortNodes(subnode)
                subnodes.append(subnode)
            else:
                properties.append(subnode)

        properties.merge(subnodes)

        return properties

    def add_rootnode(self, rootnode, prenops=None, postnops=None):
        """First sort the device tree, so that properties are before nodes."""
        rootnode = self.sortNodes(rootnode)
        super().add_rootnode(rootnode, prenops, postnops)

    def writeDtbFile(self, filename):
        """Convert the device tree to DTB and write to a file."""
        filename = os.path.realpath(filename)
        try:
            with open(filename, "wb") as f:
                f.write(self.to_dtb())
            return filename
        except OSError:
            raise RuntimeError("Failed to open DTB output file")

    def writeDtsFile(self, filename):
        """Convert the device tree to DTS and write to a file."""
        filename = os.path.realpath(filename)
        try:
            with open(filename, "w") as f:
                f.write(self.to_dts())
            return filename
        except OSError:
            raise RuntimeError("Failed to open DTS output file")
