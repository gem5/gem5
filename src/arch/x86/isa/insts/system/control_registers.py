# Copyright (c) 2009 The Regents of The University of Michigan
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

microcode = """
def macroop CLTS {
    rdcr t1, cr0, dataSize=8
    andi t1, t1, 0xF7, dataSize=1
    wrcr cr0, t1, dataSize=8
};

def macroop LMSW_R {
    rdcr t1, cr0, dataSize=8
    # This logic sets MP, EM, and TS to whatever is in the operand. It will
    # set PE but not clear it.
    limm t2, "~0xeULL", dataSize=8
    and t1, t1, t2, dataSize=8
    andi t2, reg, 0xf, dataSize=8
    or t1, t1, t2, dataSize=8
    wrcr cr0, t1, dataSize=8
};

def macroop LMSW_M {
    ld t3, seg, sib, disp, dataSize=2
    rdcr t1, cr0, dataSize=8
    # This logic sets MP, EM, and TS to whatever is in the operand. It will
    # set PE but not clear it.
    limm t2, "~0xeULL", dataSize=8
    and t1, t1, t2, dataSize=8
    andi t2, t3, 0xf, dataSize=8
    or t1, t1, t2, dataSize=8
    wrcr cr0, t1, dataSize=8
};

def macroop LMSW_P {
    rdip t7, dataSize=asz
    ld t3, seg, riprel, disp, dataSize=2
    rdcr t1, cr0, dataSize=8
    # This logic sets MP, EM, and TS to whatever is in the operand. It will
    # set PE but not clear it.
    limm t2, "~0xeULL", dataSize=8
    and t1, t1, t2, dataSize=8
    andi t2, t3, 0xf, dataSize=8
    or t1, t1, t2, dataSize=8
    wrcr cr0, t1, dataSize=8
};

def macroop SMSW_R {
    rdcr reg, cr0
};

def macroop SMSW_M {
    rdcr t1, cr0
    st t1, seg, sib, disp, dataSize=2
};

def macroop SMSW_P {
    rdcr t1, cr0
    rdip t7, dataSize=asz
    st t1, seg, riprel, disp, dataSize=2
};
"""
