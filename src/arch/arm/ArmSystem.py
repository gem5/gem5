# Copyright (c) 2009, 2012-2013, 2015-2018 ARM Limited
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
# Authors: Ali Saidi
#          Glenn Bergmans

from m5.params import *
from m5.options import *
from m5.SimObject import *
from m5.util.fdthelper import *

from m5.objects.System import System
from m5.objects.ArmSemihosting import ArmSemihosting

class ArmMachineType(Enum):
    map = {
        'RealViewPBX' : 1901,
        'VExpress_EMM' : 2272,
        'VExpress_EMM64' : 2272,
        'DTOnly' : -1,
    }

class SveVectorLength(UInt8): min = 1; max = 16

class ArmSystem(System):
    type = 'ArmSystem'
    cxx_header = "arch/arm/system.hh"
    multi_proc = Param.Bool(True, "Multiprocessor system?")
    boot_loader = VectorParam.String([],
        "File that contains the boot loader code. Zero or more files may be "
        "specified. The first boot loader that matches the kernel's "
        "architecture will be used.")
    gic_cpu_addr = Param.Addr(0, "Addres of the GIC CPU interface")
    flags_addr = Param.Addr(0, "Address of the flags register for MP booting")
    have_security = Param.Bool(False,
        "True if Security Extensions are implemented")
    have_virtualization = Param.Bool(False,
        "True if Virtualization Extensions are implemented")
    have_crypto = Param.Bool(False,
        "True if Crypto Extensions is implemented")
    have_lpae = Param.Bool(True, "True if LPAE is implemented")
    reset_addr = Param.Addr(0x0,
        "Reset address (ARMv8)")
    auto_reset_addr = Param.Bool(False,
        "Determine reset address from kernel entry point if no boot loader")
    highest_el_is_64 = Param.Bool(False,
        "True if the register width of the highest implemented exception level "
        "is 64 bits (ARMv8)")
    phys_addr_range_64 = Param.UInt8(40,
        "Supported physical address range in bits when using AArch64 (ARMv8)")
    have_large_asid_64 = Param.Bool(False,
        "True if ASID is 16 bits in AArch64 (ARMv8)")
    have_sve = Param.Bool(True,
        "True if SVE is implemented (ARMv8)")
    sve_vl = Param.SveVectorLength(1,
        "SVE vector length in quadwords (128-bit)")

    semihosting = Param.ArmSemihosting(NULL,
        "Enable support for the Arm semihosting by settings this parameter")

    m5ops_base = Param.Addr(0,
        "Base of the 64KiB PA range used for memory-mapped m5ops. Set to 0 "
        "to disable.")

    def generateDeviceTree(self, state):
        # Generate a device tree root node for the system by creating the root
        # node and adding the generated subnodes of all children.
        # When a child needs to add multiple nodes, this is done by also
        # creating a node called '/' which will then be merged with the
        # root instead of appended.

        def generateMemNode(mem_range):
            node = FdtNode("memory@%x" % long(mem_range.start))
            node.append(FdtPropertyStrings("device_type", ["memory"]))
            node.append(FdtPropertyWords("reg",
                state.addrCells(mem_range.start) +
                state.sizeCells(mem_range.size()) ))
            return node

        root = FdtNode('/')
        root.append(state.addrCellsProperty())
        root.append(state.sizeCellsProperty())

        # Add memory nodes
        for mem_range in self.mem_ranges:
            root.append(generateMemNode(mem_range))

        for node in self.recurseDeviceTree(state):
            # Merge root nodes instead of adding them (for children
            # that need to add multiple root level nodes)
            if node.get_name() == root.get_name():
                root.merge(node)
            else:
                root.append(node)

        return root

class GenericArmSystem(ArmSystem):
    type = 'GenericArmSystem'
    cxx_header = "arch/arm/system.hh"
    machine_type = Param.ArmMachineType('DTOnly',
        "Machine id from http://www.arm.linux.org.uk/developer/machines/")
    atags_addr = Param.Addr("Address where default atags structure should " \
                                "be written")
    dtb_filename = Param.String("",
        "File that contains the Device Tree Blob. Don't use DTB if empty.")
    early_kernel_symbols = Param.Bool(False,
        "enable early kernel symbol tables before MMU")
    enable_context_switch_stats_dump = Param.Bool(False, "enable stats/task info dumping at context switch boundaries")

    panic_on_panic = Param.Bool(False, "Trigger a gem5 panic if the " \
                                    "guest kernel panics")
    panic_on_oops = Param.Bool(False, "Trigger a gem5 panic if the " \
                                   "guest kernel oopses")

    def generateDtb(self, outdir, filename):
        """
        Autogenerate DTB. Arguments are the folder where the DTB
        will be stored, and the name of the DTB file.
        """
        state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
        rootNode = self.generateDeviceTree(state)

        fdt = Fdt()
        fdt.add_rootnode(rootNode)
        dtb_filename = os.path.join(outdir, filename)
        self.dtb_filename = fdt.writeDtbFile(dtb_filename)

class LinuxArmSystem(GenericArmSystem):
    type = 'LinuxArmSystem'
    cxx_header = "arch/arm/linux/system.hh"

    @cxxMethod
    def dumpDmesg(self):
        """Dump dmesg from the simulated kernel to standard out"""
        pass

    # Have Linux systems for ARM auto-calc their load_addr_mask for proper
    # kernel relocation.
    load_addr_mask = 0x0

class FreebsdArmSystem(GenericArmSystem):
    type = 'FreebsdArmSystem'
    cxx_header = "arch/arm/freebsd/system.hh"
