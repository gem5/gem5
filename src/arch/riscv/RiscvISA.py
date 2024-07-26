# Copyright (c) 2012 ARM Limited
# Copyright (c) 2014 Sven Karlsson
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
# Copyright (c) 2016 RISC-V Foundation
# Copyright (c) 2016 The University of Virginia
# Copyright (c) 2023 The Regents of the University of California
# Copyright (c) 2024 University of Rostock
# All rights reserved.
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

from m5.objects.BaseISA import BaseISA
from m5.params import (
    Enum,
    Param,
    UInt32,
)


class RiscvVectorLength(UInt32):
    min = 8
    max = 65536

    def _check(self):
        super()._check()

        # VLEN needs to be a whole power of 2. We already know value is
        # not zero. Hence:
        if self.value & (self.value - 1) != 0:
            raise TypeError("VLEN is not a power of 2: %d" % self.value)


class RiscvVectorElementLength(UInt32):
    min = 8
    max = 64

    def _check(self):
        super()._check()

        # ELEN needs to be a whole power of 2. We already know value is
        # not zero. Hence:
        if self.value & (self.value - 1) != 0:
            raise TypeError("ELEN is not a power of 2: %d" % self.value)


class RiscvType(Enum):
    vals = ["RV32", "RV64"]


class PrivilegeModeSet(Enum):
    vals = [
        "M",  # Machine privilege mode only
        "MU",  # Machine and user privlege modes
        "MNU",  # MU privilege modes with user-mode trap
        "MSU",  # Machine, supervisor and user modes
        "MNSU",  # MSU privilege modes with user-mode trap
        "MHSU",  # Machine, hypervisor, supervisor and user modes
    ]


class RiscvISA(BaseISA):
    type = "RiscvISA"
    cxx_class = "gem5::RiscvISA::ISA"
    cxx_header = "arch/riscv/isa.hh"

    riscv_type = Param.RiscvType("RV64", "RV32 or RV64")

    enable_rvv = Param.Bool(True, "Enable vector extension")
    vlen = Param.RiscvVectorLength(
        256,
        "Length of each vector register in bits. \
        VLEN in Ch. 2 of RISC-V vector spec",
    )
    elen = Param.RiscvVectorElementLength(
        64,
        "Length of each vector element in bits. \
        ELEN in Ch. 2 of RISC-V vector spec",
    )
    privilege_mode_set = Param.PrivilegeModeSet(
        "MSU",  # MHSU to enable hypervisor (H-extension)
        # also change isa.cc:readMiscReg for MIDELEG
        # if working with old bbl bootloader
        "The combination of privilege modes \
        in Privilege Levels section of RISC-V privileged spec",
    )

    enable_Zicbom_fs = Param.Bool(True, "Enable Zicbom extension in FS mode")
    enable_Zicboz_fs = Param.Bool(True, "Enable Zicboz extension in FS mode")

    wfi_resume_on_pending = Param.Bool(
        False,
        "If wfi_resume_on_pending is set to True, the hart will resume "
        "execution when interrupt becomes pending. The local enabled status "
        "is not considered.\n"
        "If wfi_resume_on_pending is set to False, the hart will only "
        "resume the execution when an locally enabled interrupt becomes "
        "pending.",
    )

    def get_isa_string(self):
        isa_extensions = []
        # check for the base ISA type
        if self.riscv_type.value == "RV32":
            isa_extensions.append("rv32")
        elif self.riscv_type.value == "RV64":
            isa_extensions.append("rv64")
        # use imafdc by default
        isa_extensions.extend(["i", "m", "a", "f", "d", "c"])
        # check for the vector extension
        if self.enable_rvv.value == True:
            isa_extensions.append("v")

        # H-extension is enabled whenever we choose
        # MHSU privilege mode set
        if self.privilege_mode_set.value == "MHSU":
            isa_extensions.append("h")

        isa_string = "".join(isa_extensions)

        if self.enable_Zicbom_fs.value:
            isa_string += "_Zicbom"  # Cache-block Management Instructions
        if self.enable_Zicboz_fs.value:
            isa_string += "_Zicboz"  # Cache-block Zero Instruction
        isa_string += "_Zicntr"  # Performance Couter Spec
        isa_string += "_Zicsr"  # RMW CSR Instructions (Privileged Spec)
        isa_string += "_Zifencei"  # FENCE.I Instruction (Unprivileged Spec)
        isa_string += "_Zihpm"  # Performance Couter Spec
        isa_string += "_Zba"  # Address Generation
        isa_string += "_Zbb"  # Basic Bit Manipulation
        isa_string += "_Zbs"  # Single-bit Instructions

        return isa_string
