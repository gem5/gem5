# Copyright (c) 2009, 2012-2013, 2015-2024 Arm Limited
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

from typing import Any

from m5.objects.ArmSemihosting import ArmSemihosting
from m5.objects.System import System
from m5.options import *
from m5.params import *
from m5.SimObject import *
from m5.util.fdthelper import *


class SveVectorLength(UInt8):
    min = 1
    max = 16


class SmeVectorLength(UInt8):
    min = 1
    max = 16

    def _check(self):
        super()._check()

        # SME needs to be a whole power of 2. We already know value is
        # not zero. Hence:
        if self.value & (self.value - 1) != 0:
            raise TypeError(
                "SME vector length is not a power of 2: %d" % self.value
            )


class ArmExtension(ScopedEnum):
    vals = [
        "FEAT_AES",
        "FEAT_PMULL",
        "FEAT_SHA1",
        "FEAT_SHA256",
        "FEAT_CRC32",
        # Armv8.1
        "FEAT_VHE",
        "FEAT_PAN",
        "FEAT_LSE",
        "FEAT_HPDS",
        "FEAT_VMID16",
        "FEAT_RDM",
        # Armv8.2
        "FEAT_SVE",
        "FEAT_UAO",
        "FEAT_LVA",  # Optional in Armv8.2
        "FEAT_LPA",  # Optional in Armv8.2
        "FEAT_F32MM",  # Optional in Armv8.2
        "FEAT_F64MM",  # Optional in Armv8.2
        "FEAT_I8MM",  # Optional in Armv8.2
        "FEAT_DOTPROD",  # Optional in Armv8.2
        # Armv8.3
        "FEAT_FCMA",
        "FEAT_JSCVT",
        "FEAT_PAuth",
        # Armv8.4
        "FEAT_SEL2",
        "FEAT_TLBIOS",
        "FEAT_TLBIRANGE",
        "FEAT_FLAGM",
        "FEAT_IDST",
        "FEAT_TTST",
        # Armv8.5
        "FEAT_FLAGM2",
        "FEAT_RNG",
        "FEAT_RNG_TRAP",
        "FEAT_EVT",
        # Armv8.6
        "FEAT_FGT",
        # Armv8.7
        "FEAT_HCX",
        "FEAT_XS",
        # Armv8.9
        "FEAT_SCTLR2",
        "FEAT_TCR2",
        # Armv9.2
        "FEAT_SME",  # Optional in Armv9.2
        # Others
        "SECURITY",
        "LPAE",
        "VIRTUALIZATION",
        "TME",
        "FEAT_MPAM",
    ]


class ArmRelease(SimObject):
    type = "ArmRelease"
    cxx_header = "arch/arm/system.hh"
    cxx_class = "gem5::ArmRelease"

    extensions = VectorParam.ArmExtension([], "ISA extensions")

    def add(self, new_ext: ArmExtension) -> None:
        """
        Add the provided extension (ArmExtension) to the system
        The method is discarding pre-existing values
        """
        if new_ext.value not in [ext.value for ext in self.extensions]:
            self.extensions.append(new_ext)

    def remove(self, ext: ArmExtension) -> None:
        """
        Remove the provided extension (ArmExtension) from the system
        """
        for curr_ext in list(self.extensions):
            if curr_ext.value == ext.value:
                self.extensions.remove(curr_ext)

    def has(self, new_ext: ArmExtension) -> bool:
        """
        Is the system implementing the provided extension (ArmExtension) ?
        """
        if new_ext.value not in [ext.value for ext in self.extensions]:
            return False
        else:
            return True

    @classmethod
    def for_kvm(cls) -> Any:
        """
        Generates an ArmRelease for KVM. It simply extracts EL2/EL3 support
        from the current cls object
        """
        release = cls()
        release.remove(ArmExtension("SECURITY"))
        release.remove(ArmExtension("VIRTUALIZATION"))
        return release


class Armv8(ArmRelease):
    extensions = [
        "LPAE",
        "VIRTUALIZATION",
        "SECURITY",
        "FEAT_AES",
        "FEAT_PMULL",
        "FEAT_SHA1",
        "FEAT_SHA256",
        "FEAT_CRC32",
    ]


class ArmDefaultRelease(Armv8):
    extensions = Armv8.extensions + [
        # Armv8.1
        "FEAT_LSE",
        "FEAT_PAN",
        "FEAT_HPDS",
        "FEAT_VMID16",
        "FEAT_RDM",
        # Armv8.2
        "FEAT_UAO",
        "FEAT_LVA",
        "FEAT_LPA",
        "FEAT_SVE",
        "FEAT_F32MM",
        "FEAT_F64MM",
        "FEAT_I8MM",
        "FEAT_DOTPROD",
        # Armv8.3
        "FEAT_FCMA",
        "FEAT_JSCVT",
        "FEAT_PAuth",
        # Armv8.4
        "FEAT_SEL2",
        "FEAT_TLBIOS",
        "FEAT_TLBIRANGE",
        "FEAT_FLAGM",
        "FEAT_IDST",
        "FEAT_TTST",
        # Armv8.5
        "FEAT_FLAGM2",
        "FEAT_EVT",
        # Armv8.6
        "FEAT_FGT",
        # Armv8.7
        "FEAT_HCX",
        "FEAT_XS",
        # Armv9.2
        "FEAT_SME",
    ]


class Armv81(Armv8):
    extensions = Armv8.extensions + [
        "FEAT_LSE",
        "FEAT_VHE",
        "FEAT_PAN",
        "FEAT_HPDS",
        "FEAT_VMID16",
        "FEAT_RDM",
    ]


class Armv82(Armv81):
    extensions = Armv81.extensions + [
        "FEAT_UAO",
        "FEAT_LVA",
        "FEAT_LPA",
        "FEAT_SVE",
        "FEAT_F32MM",
        "FEAT_F64MM",
        "FEAT_I8MM",
        "FEAT_DOTPROD",
    ]


class Armv83(Armv82):
    extensions = Armv82.extensions + ["FEAT_FCMA", "FEAT_JSCVT", "FEAT_PAuth"]


class Armv84(Armv83):
    extensions = Armv83.extensions + [
        "FEAT_SEL2",
        "FEAT_TLBIOS",
        "FEAT_TLBIRANGE",
        "FEAT_FLAGM",
        "FEAT_IDST",
        "FEAT_TTST",
    ]


class Armv85(Armv84):
    extensions = Armv84.extensions + [
        "FEAT_FLAGM2",
        "FEAT_RNG",
        "FEAT_RNG_TRAP",
        "FEAT_EVT",
    ]


class Armv86(Armv85):
    extensions = Armv85.extensions + [
        "FEAT_FGT",
    ]


class Armv87(Armv86):
    extensions = Armv86.extensions + [
        "FEAT_HCX",
        "FEAT_XS",
    ]


class Armv89(Armv87):
    extensions = Armv87.extensions + ["FEAT_SCTLR2", "FEAT_TCR2"]


class Armv92(Armv89):
    extensions = Armv89.extensions + ["FEAT_SME"]


class ArmAllRelease(ArmRelease):
    """
    A release containing any implemented extension.  It is alternatively
    possible to use the latest release (e.g. Armv92 as of now).  This could be
    preferrable for consistency across simulations.  However if users want to
    always be up to date with development, using ArmAllRelease will allow them
    to do so without the need to change their configuration script
    """

    extensions = ArmExtension.vals


class ArmSystem(System):
    type = "ArmSystem"
    cxx_header = "arch/arm/system.hh"
    cxx_class = "gem5::ArmSystem"

    release = Param.ArmRelease(ArmDefaultRelease(), "Arm Release")

    multi_proc = Param.Bool(True, "Multiprocessor system?")
    gic_cpu_addr = Param.Addr(0, "Addres of the GIC CPU interface")
    reset_addr = Param.Addr(0x0, "Reset address (ARMv8)")
    auto_reset_addr = Param.Bool(
        True,
        "Determine reset address from kernel entry point if no boot loader",
    )
    highest_el_is_64 = Param.Bool(
        True,
        "True if the register width of the highest implemented exception level "
        "is 64 bits (ARMv8)",
    )
    phys_addr_range_64 = Param.UInt8(
        40,
        "Supported physical address range in bits when using AArch64 (ARMv8)",
    )
    have_large_asid_64 = Param.Bool(
        False, "True if ASID is 16 bits in AArch64 (ARMv8)"
    )
    sve_vl = Param.SveVectorLength(
        1, "SVE vector length in quadwords (128-bit)"
    )
    sme_vl = Param.SveVectorLength(
        1, "SME vector length in quadwords (128-bit)"
    )
    semihosting = Param.ArmSemihosting(
        NULL,
        "Enable support for the Arm semihosting by settings this parameter",
    )

    # Set to true if simulation provides a PSCI implementation
    # This flag will be checked when auto-generating
    # a PSCI node. A client (e.g Linux) would then be able to
    # know if it can use the PSCI APIs
    _have_psci = False

    def generateDtb(self, filename):
        """
        Autogenerate DTB. Arguments are the folder where the DTB
        will be stored, and the name of the DTB file.
        """
        state = FdtState(addr_cells=2, size_cells=2, cpu_cells=1)
        rootNode = self.generateDeviceTree(state)

        fdt = Fdt()
        fdt.add_rootnode(rootNode)
        fdt.writeDtbFile(filename)

    def generateDeviceTree(self, state):
        # Generate a device tree root node for the system by creating the root
        # node and adding the generated subnodes of all children.
        # When a child needs to add multiple nodes, this is done by also
        # creating a node called '/' which will then be merged with the
        # root instead of appended.

        def generateMemNode(mem_range):
            node = FdtNode(f"memory@{int(mem_range.start):x}")
            node.append(FdtPropertyStrings("device_type", ["memory"]))
            node.append(
                FdtPropertyWords(
                    "reg",
                    state.addrCells(mem_range.start)
                    + state.sizeCells(mem_range.size()),
                )
            )
            return node

        root = FdtNode("/")
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
