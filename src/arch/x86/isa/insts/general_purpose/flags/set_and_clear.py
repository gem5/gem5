# Copyright (c) 2007-2008 The Hewlett-Packard Development Company
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

microcode = """
def macroop CLD {
    ruflags t1, dataSize=8
    limm t2, "~((uint64_t)DFBit)", dataSize=8
    and t1, t1, t2, dataSize=8
    wruflags t1, t0, dataSize=8
};

def macroop STD {
    ruflags t1, dataSize=8
    limm t2, "DFBit", dataSize=8
    or t1, t1, t2, dataSize=8
    wruflags t1, t0, dataSize=8
};

def macroop CLC {
    ruflags t1, dataSize=8
    limm t2, "~((uint64_t)CFBit)", dataSize=8
    and t1, t1, t2, dataSize=8
    wruflags t1, t0, dataSize=8
};

def macroop STC {
    ruflags t1, dataSize=8
    ori t1, t1, "CFBit", dataSize=8
    wruflags t1, t0, dataSize=8
};

def macroop CMC {
    ruflags t1, dataSize=8
    wruflagsi t1, "CFBit", dataSize=8
};

def macroop STI {
    rflags t1, dataSize=8

    # Extract the IOPL.
    srli t2, t1, 12, dataSize=8
    andi t2, t1, 0x3, dataSize=8

    # Find the CPL.
    rdm5reg t3, dataSize=8
    srli t3, t3, 4, dataSize=8
    andi t3, t3, 0x3, dataSize=8

    # if !(CPL > IOPL), jump down to setting IF.
    sub t0, t2, t3, flags=(ECF,), dataSize=8
    br label("setIf"), flags=(nCECF,)

    # if (CR4.PVI == 1 && CPL == 3) {
    # } else GP

    # Check CR4.PVI
    rdcr t4, cr4, dataSize=8
    andi t0, t4, 0x1, flags=(CEZF,)
    fault "std::make_shared<GeneralProtection>(0)", flags=(CEZF,)

    # Check CPL.
    andi t4, t3, 0x3, dataSize=8
    xori t4, t4, 0x3, dataSize=8, flags=(CEZF,)
    fault "std::make_shared<GeneralProtection>(0)", flags=(nCEZF,)

    #     if (RFLAGS.VIP == 1)
    #         EXCEPTION[#GP(0)]

    # Check RFLAGS.VIP == 1
    rflag t0, "VIPBit"
    fault "std::make_shared<GeneralProtection>(0)", flags=(nCEZF,)

    limm t2, "VIFBit", dataSize=8
    br label("setBitInT2")

setIf:
    limm t2, "IFBit", dataSize=8

setBitInT2:
    or t1, t1, t2, dataSize=8
    wrflags t1, t0, dataSize=8
};

def macroop STI_REAL {
    rflags t1, dataSize=8
    limm t2, "IFBit", dataSize=8
    or t1, t1, t2, dataSize=8
    wrflags t1, t0, dataSize=8
};

def macroop STI_VIRT {
    panic "Virtual mode sti isn't implemented!"
};

def macroop CLI {
    rflags t1, dataSize=8

    # Extract the IOPL.
    srli t2, t1, 12, dataSize=8
    andi t2, t1, 0x3, dataSize=8

    # Find the CPL.
    rdm5reg t3, dataSize=8
    srli t3, t3, 4, dataSize=8
    andi t3, t3, 0x3, dataSize=8

    # if !(CPL > IOPL), jump down to clearing IF.
    sub t0, t2, t3, flags=(ECF,), dataSize=8
    br label("maskIf"), flags=(nCECF,)

    # if (CR4.PVI == 1 && CPL == 3) {
    # } else GP

    # Check CR4.PVI
    rdcr t4, cr4, dataSize=8
    andi t0, t4, 0x1, flags=(CEZF,)
    fault "std::make_shared<GeneralProtection>(0)", flags=(CEZF,)

    # Check CPL.
    andi t4, t3, 0x3, dataSize=8
    xori t4, t4, 0x3, dataSize=8, flags=(CEZF,)
    fault "std::make_shared<GeneralProtection>(0)", flags=(nCEZF,)

    # RFLAGS.VIF = 0
    limm t2, "~((uint64_t)VIFBit)", dataSize=8
    br label("maskWithT2")

maskIf:
    limm t2, "~((uint64_t)IFBit)", dataSize=8

maskWithT2:
    and t1, t1, t2, dataSize=8
    wrflags t1, t0, dataSize=8
};

def macroop CLI_REAL {
    rflags t1, dataSize=8
    limm t2, "~((uint64_t)IFBit)", dataSize=8
    and t1, t1, t2, dataSize=8
    wrflags t1, t0, dataSize=8
};

def macroop CLI_VIRT {
    panic "Virtual mode cli isn't implemented!"
};
"""
