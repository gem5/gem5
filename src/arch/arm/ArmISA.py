# Copyright (c) 2012-2013, 2015-2019 ARM Limited
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

from m5.params import *
from m5.proxy import *

from m5.objects.ArmPMU import ArmPMU
from m5.objects.ArmSystem import SveVectorLength
from m5.objects.BaseISA import BaseISA
from m5.objects.ISACommon import VecRegRenameMode

# Enum for DecoderFlavor
class DecoderFlavor(Enum): vals = ['Generic']

class ArmISA(BaseISA):
    type = 'ArmISA'
    cxx_class = 'ArmISA::ISA'
    cxx_header = "arch/arm/isa.hh"

    system = Param.System(Parent.any, "System this ISA object belongs to")

    pmu = Param.ArmPMU(NULL, "Performance Monitoring Unit")
    decoderFlavor = Param.DecoderFlavor(
            'Generic', "Decoder flavor specification")

    # If no MIDR value is provided, 0x0 is treated by gem5 as follows:
    # When 'highest_el_is_64' (AArch64 support) is:
    #   True  -> Cortex-A57 TRM r0p0 MIDR is used
    #   False -> Cortex-A15 TRM r0p0 MIDR is used
    midr = Param.UInt32(0x0, "MIDR value")

    # See section B4.1.89 - B4.1.92 of the ARM ARM
    #  VMSAv7 support
    id_mmfr0 = Param.UInt32(0x10201103, "Memory Model Feature Register 0")
    id_mmfr1 = Param.UInt32(0x00000000, "Memory Model Feature Register 1")
    # no HW access | WFI stalling | ISB and DSB |
    # all TLB maintenance | no Harvard
    id_mmfr2 = Param.UInt32(0x01230000, "Memory Model Feature Register 2")
    # SuperSec | Coherent TLB | Bcast Maint |
    # BP Maint | Cache Maint Set/way | Cache Maint MVA
    id_mmfr3 = Param.UInt32(0x02102211, "Memory Model Feature Register 3")

    # See section B4.1.84 of ARM ARM
    # All values are latest for ARMv7-A profile
    id_isar0 = Param.UInt32(0x02101111, "Instruction Set Attribute Register 0")
    id_isar1 = Param.UInt32(0x02112111, "Instruction Set Attribute Register 1")
    id_isar2 = Param.UInt32(0x21232141, "Instruction Set Attribute Register 2")
    id_isar3 = Param.UInt32(0x01112131, "Instruction Set Attribute Register 3")
    id_isar4 = Param.UInt32(0x10010142, "Instruction Set Attribute Register 4")
    id_isar5 = Param.UInt32(0x10000000, "Instruction Set Attribute Register 5")

    fpsid = Param.UInt32(0x410430a0, "Floating-point System ID Register")

    # [31:0] is implementation defined
    id_aa64afr0_el1 = Param.UInt64(0x0000000000000000,
        "AArch64 Auxiliary Feature Register 0")
    # Reserved for future expansion
    id_aa64afr1_el1 = Param.UInt64(0x0000000000000000,
        "AArch64 Auxiliary Feature Register 1")

    # 1 CTX CMPs | 2 WRPs | 2 BRPs | !PMU | !Trace | Debug v8-A
    id_aa64dfr0_el1 = Param.UInt64(0x0000000000101006,
        "AArch64 Debug Feature Register 0")
    # Reserved for future expansion
    id_aa64dfr1_el1 = Param.UInt64(0x0000000000000000,
        "AArch64 Debug Feature Register 1")

    # !CRC32 | !SHA2 | !SHA1 | !AES
    id_aa64isar0_el1 = Param.UInt64(0x0000000000000000,
        "AArch64 Instruction Set Attribute Register 0")

    # GPI = 0x0 | GPA = 0x1| API=0x0 | APA=0x1 | FCMA
    id_aa64isar1_el1 = Param.UInt64(0x0000000001010010,
        "AArch64 Instruction Set Attribute Register 1")

    # 4K | 64K | !16K | !BigEndEL0 | !SNSMem | !BigEnd | 8b ASID | 40b PA
    id_aa64mmfr0_el1 = Param.UInt64(0x0000000000f00002,
        "AArch64 Memory Model Feature Register 0")
    # PAN | HPDS
    id_aa64mmfr1_el1 = Param.UInt64(0x0000000000101000,
        "AArch64 Memory Model Feature Register 1")
    id_aa64mmfr2_el1 = Param.UInt64(0x0000000000000000,
        "AArch64 Memory Model Feature Register 2")

    # Any access (read/write) to an unimplemented
    # Implementation Defined registers is not causing an Undefined Instruction.
    # It is rather executed as a NOP.
    impdef_nop = Param.Bool(False,
        "Any access to a MISCREG_IMPDEF_UNIMPL register is executed as NOP")

    # This is required because in SE mode a generic System SimObject is
    # allocated, instead of an ArmSystem
    sve_vl_se = Param.SveVectorLength(1,
        "SVE vector length in quadwords (128-bit), SE-mode only")
